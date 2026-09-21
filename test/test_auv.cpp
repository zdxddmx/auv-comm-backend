/* test_auv.cpp — 单元测试（纯 C++17，无第三方框架）
 *
 *  T1 frame     CRC16 自检、编解码往返、载荷上限、坏 CRC、粘包/半包/重同步
 *  T2 acoustic  传播时延、发送占用、丢包、到达排序
 *  T3 link      内存链路收发、丢包注入、帧解析
 *  T4 seq       序号连续性：正常/丢包/乱序/重复 + 16 位环绕
 *  T5 router    汇聚、分发、心跳超时、指数退避重连、主备切换与切回
 *  T6 store     遥测/指令/事件入库、落盘、按设备聚合统计
 */
#include "auv.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

using namespace auv;

static int passed = 0, failed = 0;

#define CHECK(cond, ...) do {                       \
    if (cond) { passed++; std::printf("  [PASS] "); } \
    else      { failed++; std::printf("  [FAIL] "); } \
    std::printf(__VA_ARGS__);                         \
    std::printf("\n");                                \
} while (0)

#define NEAR(a, b, tol) (std::fabs((a) - (b)) <= (tol))

/* ----------------------------------------------------------------- T1 frame */
static void t1_frame()
{
    std::printf("T1 frame（帧协议与 CRC）\n");

    /* CRC-16/CCITT-FALSE 已知值："123456789" -> 0x29B1 */
    const char *s = "123456789";
    const uint16_t crc = crc16_ccitt(reinterpret_cast<const uint8_t *>(s), 9);
    CHECK(crc == 0x29B1, "CRC-16/CCITT-FALSE(\"123456789\") = 0x%04X（期望 0x29B1）", crc);

    Frame f;
    f.type = MsgType::Telemetry;
    f.seq = 0x1234;
    f.payload = {1, 2, 3, 4, 5};

    uint8_t buf[FRAME_HEADER_LEN + FRAME_MAX_PAYLOAD + FRAME_CRC_LEN];
    const size_t n = frame_encode(f, buf, sizeof(buf));
    CHECK(n == FRAME_HEADER_LEN + 5 + FRAME_CRC_LEN,
          "编码长度 = %zu（8 header + 5 payload + 2 crc）", n);
    CHECK(buf[0] == FRAME_SYNC0 && buf[1] == FRAME_SYNC1, "同步头正确 0x%02X%02X", buf[0], buf[1]);

    Frame g;
    CHECK(frame_decode(buf, n, g), "解码成功");
    CHECK(g.type == MsgType::Telemetry && g.seq == 0x1234 && g.payload.size() == 5,
          "字段一致：type=%s seq=%u len=%zu", msg_type_name(g.type), g.seq, g.payload.size());
    CHECK(g.payload == f.payload, "载荷一致");

    /* 坏 CRC */
    buf[n - 1] ^= 0xFF;
    CHECK(!frame_decode(buf, n, g), "CRC 被破坏 -> 解码失败");

    /* 载荷上限 */
    Frame big;
    big.payload.assign(FRAME_MAX_PAYLOAD + 1, 0xAA);
    CHECK(frame_encode(big, buf, sizeof(buf)) == 0, "载荷超限 -> 编码返回 0");

    /* 空载荷 */
    Frame empty;
    empty.type = MsgType::Heartbeat;
    const size_t ne = frame_encode(empty, buf, sizeof(buf));
    CHECK(ne == FRAME_HEADER_LEN + FRAME_CRC_LEN, "空载荷编码长度 = %zu", ne);
    CHECK(frame_decode(buf, ne, g) && g.payload.empty(), "空载荷解码成功");

    /* 流式解帧：粘包 */
    Frame a1, a2;
    a1.type = MsgType::Heartbeat;  a1.seq = 10;
    a2.type = MsgType::Event;      a2.seq = 11; a2.payload = {9, 9};
    uint8_t b1[64], b2[64];
    const size_t n1 = frame_encode(a1, b1, sizeof(b1));
    const size_t n2 = frame_encode(a2, b2, sizeof(b2));
    std::vector<uint8_t> glue(b1, b1 + n1);
    glue.insert(glue.end(), b2, b2 + n2);

    FrameParser p;
    p.feed(glue.data(), glue.size());
    Frame o;
    CHECK(p.next(o) && o.seq == 10, "粘包：第一帧 seq=%u", o.seq);
    CHECK(p.next(o) && o.seq == 11 && o.payload.size() == 2, "粘包：第二帧 seq=%u", o.seq);
    CHECK(!p.next(o), "粘包：无更多帧");

    /* 半包 */
    FrameParser p2;
    p2.feed(b1, n1 / 2);
    CHECK(!p2.next(o), "半包：数据不足时不吐帧");
    CHECK(p2.buffered() == n1 / 2, "半包：缓冲保留 %zu 字节", p2.buffered());
    p2.feed(b1 + n1 / 2, n1 - n1 / 2);
    CHECK(p2.next(o) && o.seq == 10, "半包：补全后解出 seq=%u", o.seq);

    /* 重同步：前面塞垃圾字节 */
    FrameParser p3;
    std::vector<uint8_t> junk = {0x00, 0xFF, 0x12, 0x34};
    junk.insert(junk.end(), b1, b1 + n1);
    p3.feed(junk.data(), junk.size());
    CHECK(p3.next(o) && o.seq == 10, "重同步后解出 seq=%u", o.seq);
    CHECK(p3.resyncs() >= 1, "重同步计数 = %u", p3.resyncs());

    /* CRC 错帧应被丢弃并计数 */
    FrameParser p4;
    uint8_t bad[64];
    std::memcpy(bad, b1, n1);
    bad[n1 - 1] ^= 0xFF;
    p4.feed(bad, n1);
    CHECK(!p4.next(o) && p4.crc_errors() == 1, "坏帧被丢弃，CRC 错误计数 = %u", p4.crc_errors());
}

/* -------------------------------------------------------------- T2 acoustic */
static void t2_acoustic()
{
    std::printf("T2 acoustic（水声信道）\n");

    AcousticConfig c;
    c.range_m = 1500.0;
    c.speed_of_sound = 1500.0;
    c.bitrate_bps = 9600.0;
    c.loss_ratio = 0.0;

    CHECK(NEAR(acoustic_propagation_delay(c), 1.0, 1e-9),
          "1500 m / 1500 m·s⁻¹ -> 单程时延 %.4f s", acoustic_propagation_delay(c));
    CHECK(NEAR(acoustic_tx_time(c, 120), 0.1, 1e-9),
          "120 B @9600 bps -> 占用 %.4f s", acoustic_tx_time(c, 120));

    AcousticChannel ch(c);
    std::vector<uint8_t> f(64, 0xAB);
    CHECK(ch.transmit(f), "无丢包配置下发帧成功");
    std::vector<uint8_t> out;
    CHECK(!ch.poll_ready(0.5, out), "0.5 s 时未到达（在途）");
    CHECK(ch.poll_ready(1.2, out) && out.size() == 64, "1.2 s 时到达");
    CHECK(ch.sent() == 1 && ch.dropped() == 0, "sent=%u dropped=%u", ch.sent(), ch.dropped());

    /* 全丢包配置 */
    AcousticConfig c2 = c;
    c2.loss_ratio = 1.0;
    AcousticChannel ch2(c2);
    CHECK(!ch2.transmit(f), "丢包率 100% -> 全部丢弃");
    CHECK(ch2.dropped() == 1, "dropped = %u", ch2.dropped());

    /* 低带宽串行占用：连发两帧，第二帧应更晚到达 */
    AcousticChannel ch3(c);
    std::vector<uint8_t> f2(960, 0xCD);           /* 960 B @9600bps = 0.8 s */
    ch3.transmit(f2);
    ch3.transmit(f2);
    std::vector<uint8_t> o1, o2;
    CHECK(ch3.poll_ready(2.0, o1), "第 1 帧在 2.0 s 前到达");
    CHECK(!ch3.poll_ready(2.0, o2), "第 2 帧被第 1 帧的信道占用推迟");
    CHECK(ch3.poll_ready(3.0, o2), "第 2 帧在 3.0 s 前到达");
}

/* ------------------------------------------------------------------ T3 link */
static void t3_link()
{
    std::printf("T3 link（内存链路）\n");

    MemoryLink link("tcp", LinkKind::Tcp, 0.0, 0.0, 3);
    Frame f;
    f.type = MsgType::Command;
    f.seq = 7;
    f.payload = {0xAA, 0xBB};

    CHECK(link.send(f), "发送成功");
    CHECK(link.stat().tx_frames == 1 && link.stat().tx_bytes > 0,
          "发送统计 tx_frames=%llu tx_bytes=%llu",
          (unsigned long long)link.stat().tx_frames,
          (unsigned long long)link.stat().tx_bytes);

    Frame o;
    CHECK(!link.recv(o), "未 tick 前接收队列为空");
    link.tick(0.0);
    CHECK(link.recv(o) && o.seq == 7, "tick 后可接收 seq=%u", o.seq);

    /* 断链 */
    link.set_connected(false);
    CHECK(!link.send(f) && link.stat().dropped == 1, "断链后发送被丢弃");
    link.tick(0.0);
    CHECK(!link.connected(), "链路状态为 down");

    /* 丢包注入 */
    MemoryLink lossy("udp", LinkKind::Udp, 0.0, 1.0, 5);
    CHECK(!lossy.send(f) && lossy.stat().dropped == 1, "丢包率 100% -> 发送丢弃");

    /* 对端注入 */
    MemoryLink rx("s", LinkKind::Serial, 0.0, 0.0, 9);
    rx.inject(f);
    CHECK(rx.recv(o) && o.seq == 7, "对端注入可收到 seq=%u", o.seq);
}

/* ------------------------------------------------------------------- T4 seq */
static void t4_seq()
{
    std::printf("T4 seq（序号连续性检测）\n");

    SeqTracker t;
    CHECK(t.observe(100) == 0, "首帧建立基线");
    CHECK(t.observe(101) == 0 && t.observe(102) == 0, "连续帧正常");
    CHECK(t.received == 3, "received = %llu", (unsigned long long)t.received);

    uint16_t missing = 0;
    const int st = t.observe(106, &missing);
    CHECK(st == 1 && missing == 3, "跳号 -> 检测到丢包 %u 帧（103,104,105）", missing);
    CHECK(t.lost == 3, "lost = %llu", (unsigned long long)t.lost);

    CHECK(t.observe(105) == 2, "回退序号 -> 乱序");
    CHECK(t.reordered == 1, "reordered = %llu", (unsigned long long)t.reordered);

    CHECK(t.observe(106) == 3, "重复序号 -> 重复");
    CHECK(t.duplicated == 1, "duplicated = %llu", (unsigned long long)t.duplicated);

    /* 16 位环绕 */
    SeqTracker w;
    w.observe(65534);
    CHECK(w.observe(65535) == 0, "65534 -> 65535 正常");
    CHECK(w.observe(0) == 0, "65535 -> 0 环绕正常");
    CHECK(w.observe(1) == 0, "0 -> 1 正常");
    CHECK(w.lost == 0, "环绕时未误判丢包");

    /* 丢包率计算 */
    SeqTracker r;
    r.observe(1);
    r.observe(3);                    /* 丢 1 帧 */
    CHECK(NEAR(r.loss_ratio(), 1.0 / 3.0, 1e-9), "丢包率 = %.3f",
          r.loss_ratio());
}

/* ---------------------------------------------------------------- T5 router */
static void t5_router()
{
    std::printf("T5 router（通信中间件与可靠性）\n");

    auto primary = std::make_shared<MemoryLink>("udp-shore", LinkKind::Udp, 0.0, 0.0, 1);
    auto backup  = std::make_shared<MemoryLink>("acoustic", LinkKind::Acoustic, 0.0, 0.0, 2);

    RouterConfig cfg;
    cfg.heartbeat_timeout_s = 1.0;
    Router rt(cfg, {primary, backup}, "udp-shore");

    CHECK(rt.active_link() == "udp-shore", "初始活动链路 = %s", rt.active_link().c_str());

    /* 指令分发 */
    CHECK(rt.send_command("AUV-01", MsgType::Command, {1, 2, 3}), "下发指令成功");
    CHECK(rt.commands_sent() == 1, "指令计数 = %llu",
          (unsigned long long)rt.commands_sent());

    /* 数据汇聚：注入两帧遥测 */
    auto mk = [](const std::string &id, uint16_t seq) {
        Frame f;
        f.type = MsgType::Telemetry;
        f.seq = seq;
        f.payload.push_back(static_cast<uint8_t>(id.size()));
        f.payload.insert(f.payload.end(), id.begin(), id.end());
        const float d = 12.5f, b = 88.0f, h = 45.0f;
        for (const float *v : {&d, &b, &h}) {
            const uint8_t *p = reinterpret_cast<const uint8_t *>(v);
            f.payload.insert(f.payload.end(), p, p + 4);
        }
        return f;
    };
    primary->inject(mk("AUV-01", 1));
    primary->inject(mk("AUV-02", 2));
    primary->tick(0.0);
    /* poll() 返回"本 tick 收到帧总数"（含回环的指令帧），
     * 这里真正要验证的是**按设备汇聚**的结果 */
    const uint32_t got = rt.poll(0.1);
    CHECK(got >= 2, "收到帧数 = %u（2 帧遥测 + 回环指令）", got);
    CHECK(rt.devices().size() == 2, "汇聚到 %zu 台设备（不因指令帧产生假设备）",
          rt.devices().size());
    CHECK(NEAR(rt.devices().at("AUV-01").depth_m, 12.5, 1e-3),
          "AUV-01 深度 = %.2f m", rt.devices().at("AUV-01").depth_m);
    CHECK(NEAR(rt.devices().at("AUV-02").battery, 88.0, 1e-3),
          "AUV-02 电量 = %.1f%%", rt.devices().at("AUV-02").battery);

    /* 主链路故障 -> 切换 */
    primary->set_connected(false);
    primary->tick(0.0);
    rt.tick(0.2, 0.05);
    CHECK(rt.health().at("udp-shore").up == false, "主链路被标记 down");
    rt.tick(0.4, 0.05);                    /* 触发 failover */
    CHECK(rt.active_link() == "acoustic", "故障后切换到 = %s", rt.active_link().c_str());
    CHECK(rt.health().at("udp-shore").failovers >= 1, "切换次数 = %u",
          rt.health().at("udp-shore").failovers);

    /* 备用链路仍可下发 */
    CHECK(rt.send_command("AUV-01", MsgType::Command, {9}), "备用链路下发成功");

    /* 指数退避重连：传输层恢复后，tick 推进应触发重连
     * 注意：重连后必须**持续有数据**，否则心跳超时会再次把链路标 down
     * （这正是可靠性设计想表达的行为） */
    primary->set_connected(true);
    bool reconnected = false;
    for (int i = 0; i < 40 && !reconnected; i++) {
        rt.tick(0.4 + i * 0.1, 0.05);
        if (rt.health().at("udp-shore").up) reconnected = true;
    }
    CHECK(reconnected, "主链路在退避后被重连（reconnects=%u）",
          rt.health().at("udp-shore").reconnects);
    CHECK(rt.health().at("udp-shore").reconnects >= 1, "重连次数 = %u",
          rt.health().at("udp-shore").reconnects);

    /* 重连后持续给数据，链路应稳定保持 up */
    primary->inject(mk("AUV-01", 100));
    primary->tick(0.0);
    rt.poll(8.0);
    rt.tick(8.5, 0.05);
    CHECK(rt.health().at("udp-shore").up == true, "持续有数据时链路保持 up");

    /* 主链路恢复后切回（failback）
     * 注意 tick 的时间要紧接着最后一次收数据，否则会先被心跳超时判失联 */
    rt.tick(8.6, 0.05);
    CHECK(rt.active_link() == "udp-shore", "恢复后切回 = %s", rt.active_link().c_str());

    /* 心跳超时判定 */
    auto l3 = std::make_shared<MemoryLink>("serial", LinkKind::Serial, 0.0, 0.0, 3);
    Router rt2(cfg, {l3}, "serial");
    l3->inject(mk("AUV-09", 1));
    l3->tick(0.0);
    rt2.poll(0.0);
    rt2.tick(5.0, 0.05);                       /* 超过 1 s 心跳超时 */
    CHECK(rt2.health().at("serial").up == false, "心跳超时 -> 链路 down");

    /* 日志非空且含关键事件 */
    const auto &lg = rt.log();
    bool has_failover = false, has_failback = false;
    for (const auto &s : lg) {
        if (s.find("FAILOVER") != std::string::npos) has_failover = true;
        if (s.find("FAILBACK") != std::string::npos) has_failback = true;
    }
    CHECK(has_failover, "日志含 FAILOVER 事件");
    CHECK(has_failback, "日志含 FAILBACK 事件");
}

/* ----------------------------------------------------------------- T6 store */
static void t6_store()
{
    std::printf("T6 store（数据存储与统计）\n");

    Store st("test_journal.csv");
    for (int i = 0; i < 5; i++) {
        st.insert_telemetry(TelemetryRow{static_cast<double>(i) * 0.1, "AUV-01",
                                         10.0 + i, 90.0 - i, 45.0, "udp-shore"});
    }
    st.insert_telemetry(TelemetryRow{0.5, "AUV-02", 20.0, 80.0, 90.0, "acoustic"});
    st.insert_command(1.0, "AUV-01", "DIVE 15");
    st.insert_link_event(2.0, "udp-shore", "FAILOVER -> acoustic");

    CHECK(st.telemetry_count() == 6, "遥测 %zu 条", st.telemetry_count());
    CHECK(st.command_count() == 1, "指令 %zu 条", st.command_count());
    CHECK(st.event_count() == 1, "链路事件 %zu 条", st.event_count());

    CHECK(st.flush(), "落盘成功");

    auto sum = st.depth_summary();
    CHECK(sum.size() == 2, "聚合到 %zu 台设备", sum.size());
    CHECK(sum.at("AUV-01").first == 5, "AUV-01 遥测 %zu 条", sum.at("AUV-01").first);
    CHECK(NEAR(sum.at("AUV-01").second, 12.0, 1e-9),
          "AUV-01 平均深度 = %.2f m（(10+11+12+13+14)/5）", sum.at("AUV-01").second);
    CHECK(NEAR(sum.at("AUV-02").second, 20.0, 1e-9), "AUV-02 平均深度 = %.2f m",
          sum.at("AUV-02").second);

    std::remove("test_journal.csv");
}

int main()
{
    std::printf("==== auv-comm-backend 单元测试 ====\n\n");
    t1_frame();
    t2_acoustic();
    t3_link();
    t4_seq();
    t5_router();
    t6_store();
    std::printf("\n==== 结果：%d passed, %d failed ====\n", passed, failed);
    return failed ? 1 : 0;
}
