/* main.cpp — 仿真主程序：跑一个"岸基 ↔ 多台 AUV"的通信后台场景
 *
 * 场景：
 *   1. 岸基通过主链路（UDP）向 2 台 AUV 下发指令
 *   2. AUV 回传遥测（心跳 + 深度/电量/航向）
 *   3. 主链路故障 -> 自动切换到备用链路（水声链路，高延迟 + 丢包 + 低带宽）
 *   4. 主链路恢复 -> 自动切回
 *   5. 全程统计：帧数、CRC 错误、丢包/乱序、重连与切换次数
 */
#include "auv.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

using namespace auv;

static std::vector<uint8_t> make_telemetry(const std::string &id,
                                           double depth, double batt, double head)
{
    std::vector<uint8_t> p;
    p.push_back(static_cast<uint8_t>(id.size()));
    p.insert(p.end(), id.begin(), id.end());
    const float d = static_cast<float>(depth), b = static_cast<float>(batt),
                h = static_cast<float>(head);
    const uint8_t *pd = reinterpret_cast<const uint8_t *>(&d);
    const uint8_t *pb = reinterpret_cast<const uint8_t *>(&b);
    const uint8_t *ph = reinterpret_cast<const uint8_t *>(&h);
    p.insert(p.end(), pd, pd + 4);
    p.insert(p.end(), pb, pb + 4);
    p.insert(p.end(), ph, ph + 4);
    return p;
}

int main(int argc, char **argv)
{
    const std::string scen = (argc > 1) ? argv[1] : "normal";

    /* 主链路：UDP，低延迟无丢包；备用链路：水声，高延迟 + 丢包 + 低带宽 */
    auto udp = std::make_shared<MemoryLink>("udp-shore", LinkKind::Udp, 0.02, 0.0, 7);
    auto sonar = std::make_shared<MemoryLink>("acoustic-backup", LinkKind::Acoustic,
                                              1.40, 0.05, 11);

    RouterConfig cfg;
    cfg.heartbeat_timeout_s = 3.0;   /* 必须大于水声 RTT(约 2.8 s) */
    Router router(cfg, {udp, sonar}, "udp-shore");

    Store store("auv_comm_journal.csv");

    /* 声学信道参数（用于报告打印） */
    AcousticConfig ac;
    ac.range_m = 2000.0;
    ac.bitrate_bps = 9600.0;
    AcousticChannel chan(ac);
    std::fprintf(stderr,
                 "[acoustic] range=%.0f m, speed=%.0f m/s -> one-way delay=%.3f s, "
                 "bitrate=%.0f bps, 1 帧(64B) 占用 %.0f ms\n",
                 ac.range_m, ac.speed_of_sound,
                 acoustic_propagation_delay(ac), ac.bitrate_bps,
                 acoustic_tx_time(ac, 64) * 1000.0);

    const std::string devs[2] = {"AUV-01", "AUV-02"};
    const double dt = 0.05;
    double now = 0.0;
    const double total = 12.0;
    int step = 0;
    size_t logged_ = 0;
    unsigned seq_auv = 0;

    std::printf("t_s,link,devices,telemetry,seq_lost,seq_reorder,reconnects,failovers\n");

    while (now < total) {
        /* 事件注入 */
        if (scen == "failover") {
            if (std::fabs(now - 4.0) < dt / 2) {
                udp->set_connected(false);
                std::fprintf(stderr, "[t=%.1fs] 主链路故障注入\n", now);
            }
            if (std::fabs(now - 8.0) < dt / 2) {
                udp->set_connected(true);
                std::fprintf(stderr, "[t=%.1fs] 主链路恢复\n", now);
            }
        }

        /* 岸基每 1 s 下发一次指令，并把指令写入存储 */
        if (step % 20 == 0) {
            std::vector<uint8_t> cmd = {'D', 'I', 'V', 'E', static_cast<uint8_t>(10 + step % 5)};
            for (const auto &d : devs) {
                if (router.send_command(d, MsgType::Command, cmd)) {
                    char text[64];
                    std::snprintf(text, sizeof(text), "DIVE %d", 10 + step % 5);
                    store.insert_command(now, d, text);
                }
            }
        }

        /* AUV 每 0.4 s 回传遥测（走当前活动链路的对端） */
        if (step % 8 == 0) {
            for (size_t i = 0; i < 2; i++) {
                Frame f;
                f.type = MsgType::Telemetry;
                f.seq = static_cast<uint16_t>(seq_auv++);
                f.payload = make_telemetry(devs[i],
                                           12.5 + 3.0 * std::sin(now + i),
                                           88.0 - 0.5 * now,
                                           45.0 * i + 10.0 * std::cos(now));
                if (router.active_link() == "udp-shore") udp->inject(f);
                else                                     sonar->inject(f);
            }
        }

        /* 主循环：推进链路 -> 轮询收帧 -> 可靠性 tick */
        udp->tick(dt);
        sonar->tick(dt);
        const uint32_t got = router.poll(now);
        router.tick(now, dt);

        /* 把本轮新增的可靠性日志转成链路事件入库（只记关键事件） */
        {
            const auto &lg = router.log();
            while (logged_ < lg.size()) {
                const std::string &line = lg[logged_++];
                const char *keys[] = {"LINK DOWN", "LINK RECONNECT", "FAILOVER", "FAILBACK"};
                for (const char *k : keys) {
                    if (line.rfind(k, 0) == 0 || line.find(k) != std::string::npos) {
                        store.insert_link_event(now, router.active_link(), line);
                        break;
                    }
                }
            }
        }

        /* 遥测入库 */
        for (const auto &kv : router.devices()) {
            if (kv.second.frames) {
                store.insert_telemetry(TelemetryRow{now, kv.first,
                    kv.second.depth_m, kv.second.battery, kv.second.heading_deg,
                    kv.second.active_link});
            }
        }
        (void)got;

        if (step % 20 == 0) {
            std::printf("%.2f,%s,%zu,%zu,%llu,%llu,%u,%u\n",
                        now, router.active_link().c_str(),
                        router.devices().size(), router.seq().received,
                        static_cast<unsigned long long>(router.seq().lost),
                        static_cast<unsigned long long>(router.seq().reordered),
                        router.health().at("udp-shore").reconnects,
                        router.health().at("udp-shore").failovers);
        }

        now += dt;
        step++;
    }

    /* 汇总 */
    store.flush();
    std::fprintf(stderr, "\n===== 汇总 =====\n");
    std::fprintf(stderr, "活动链路        : %s\n", router.active_link().c_str());
    std::fprintf(stderr, "下发指令        : %llu 条\n",
                 static_cast<unsigned long long>(router.commands_sent()));
    std::fprintf(stderr, "接收帧          : %llu 帧（期望序号连续性检测）\n",
                 static_cast<unsigned long long>(router.seq().received));
    std::fprintf(stderr, "序号丢包/乱序/重复: %llu / %llu / %llu（丢包率 %.1f%%）\n",
                 static_cast<unsigned long long>(router.seq().lost),
                 static_cast<unsigned long long>(router.seq().reordered),
                 static_cast<unsigned long long>(router.seq().duplicated),
                 router.seq().loss_ratio() * 100.0);
    for (const auto &kv : router.health()) {
        std::fprintf(stderr, "链路 %-16s up=%d reconnects=%u failovers=%u\n",
                     kv.first.c_str(), kv.second.up ? 1 : 0,
                     kv.second.reconnects, kv.second.failovers);
    }
    for (const auto &kv : router.devices()) {
        std::fprintf(stderr, "设备 %-8s 帧=%-4u 深度=%.2f m 电量=%.1f%% 航向=%.0f° 链路=%s\n",
                     kv.first.c_str(), kv.second.frames, kv.second.depth_m,
                     kv.second.battery, kv.second.heading_deg,
                     kv.second.active_link.c_str());
    }
    std::fprintf(stderr, "遥测入库 %zu 条 · 指令 %zu 条 · 链路事件 %zu 条 -> %s\n",
                 store.telemetry_count(), store.command_count(),
                 store.event_count(), "auv_comm_journal.csv");

    const size_t nlog = router.log().size();
    std::fprintf(stderr, "\n最近 10 条日志：\n");
    for (size_t i = (nlog > 10 ? nlog - 10 : 0); i < nlog; i++)
        std::fprintf(stderr, "  %s\n", router.log()[i].c_str());
    return 0;
}
