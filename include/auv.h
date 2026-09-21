/*
 * auv.h — 岸基与水下航行器（AUV/ROV）通信后台
 *
 * 对齐真实岗位的技术结构（岸基通信后台软件）：
 *   frame     帧协议：同步头 + 长度 + 序号 + 载荷 + CRC16，编解码与校验
 *   acoustic  水声信道仿真：传播延迟 / 丢包 / 带宽限制（水声通信的真实约束）
 *   link      链路抽象：UDP / TCP / 串口 / 水声仿真链路，统一读写接口
 *   router    通信中间件：多航行器数据汇聚、指令分发、链路管理
 *   reliable  ★ 可靠性：主备链路冗余切换、断线重连（指数退避）、
 *              序号连续性检测（发现丢包/乱序）、事件与帧日志
 *   store     数据存储：遥测 / 指令 / 链路事件（journal 落盘；可选 SQLite）
 *
 * 纯 C++17，零第三方依赖（SQLite 后端为可选编译项）。
 */
#ifndef AUV_H
#define AUV_H

#include <cstdint>
#include <cstddef>
#include <deque>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace auv {

/* ============================================================ 帧协议 frame */
constexpr uint8_t  FRAME_SYNC0 = 0xEB;
constexpr uint8_t  FRAME_SYNC1 = 0x90;
constexpr size_t   FRAME_MAX_PAYLOAD = 1024;
constexpr size_t   FRAME_HEADER_LEN  = 8;   /* sync0 sync1 type seq_lo seq_hi len_lo len_hi */
constexpr size_t   FRAME_CRC_LEN     = 2;

enum class MsgType : uint8_t {
    Heartbeat = 0x01,   /* 心跳       */
    Telemetry = 0x02,   /* 遥测上报    */
    Command   = 0x03,   /* 岸基指令    */
    Ack       = 0x04,   /* 指令确认    */
    Event     = 0x05,   /* 事件通知    */
    Bulk      = 0x06,   /* 批量数据    */
};

const char *msg_type_name(MsgType t);

struct Frame {
    MsgType  type = MsgType::Heartbeat;
    uint16_t seq  = 0;
    std::vector<uint8_t> payload;
};

uint16_t crc16_ccitt(const uint8_t *data, size_t len);

/* 编码：写入 out，返回总字节数；载荷超长返回 0 */
size_t frame_encode(const Frame &f, uint8_t *out, size_t out_len);
/* 解码：成功返回 true 并填充 f；buf 必须正好是一帧（含 CRC） */
bool   frame_decode(const uint8_t *buf, size_t len, Frame &f);

/* 流式解帧器：喂入任意分片，逐帧吐出（处理粘包/半包） */
class FrameParser {
public:
    void feed(const uint8_t *data, size_t len);
    bool next(Frame &out);
    uint32_t crc_errors() const { return crc_errors_; }
    uint32_t resyncs()    const { return resyncs_; }
    size_t   buffered()   const { return buf_.size(); }
private:
    std::vector<uint8_t> buf_;
    uint32_t crc_errors_ = 0;
    uint32_t resyncs_    = 0;
};

/* ================================================== 水声信道仿真 acoustic */
struct AcousticConfig {
    double speed_of_sound = 1500.0;  /* m/s（海水声速 ~1500） */
    double range_m        = 2000.0;  /* 通信距离 m            */
    double bitrate_bps    = 9600.0;  /* 水声链路速率（低）      */
    double loss_ratio     = 0.05;    /* 基础丢包率             */
    double jitter_ratio   = 0.20;    /* 延迟抖动比例            */
    uint32_t seed         = 20260919;
};

/* 传播时延（单程，秒）= range / speed_of_sound */
double acoustic_propagation_delay(const AcousticConfig &c);
/* 发一帧的占用时间（秒）= bits / bitrate */
double acoustic_tx_time(const AcousticConfig &c, size_t frame_bytes);

class AcousticChannel {
public:
    explicit AcousticChannel(AcousticConfig c = {});
    /* 送入一个待发送帧；返回是否被"信道"丢弃 */
    bool transmit(const std::vector<uint8_t> &bytes);
    /* 取出一个已到达的帧（按到达时刻排序）；无则返回空 */
    bool poll_ready(double now, std::vector<uint8_t> &out);
    uint32_t sent()   const { return sent_; }
    uint32_t dropped()const { return dropped_; }
    double   avg_delay() const;
private:
    struct Pkt { double arrive_at; std::vector<uint8_t> bytes; };
    AcousticConfig cfg_;
    std::deque<Pkt> inflight_;
    uint32_t sent_ = 0, dropped_ = 0;
    double total_delay_ = 0.0;
    uint64_t rng_state_;
    double   next_free_ = 0.0;      /* 串行发送占用（低带宽）*/
    double   urand();
};

/* ============================================================ 链路 link */
enum class LinkKind { Udp, Tcp, Serial, Acoustic };

const char *link_kind_name(LinkKind k);

struct LinkStat {
    uint64_t tx_frames = 0, rx_frames = 0;
    uint64_t tx_bytes  = 0, rx_bytes  = 0;
    uint64_t crc_errors = 0, dropped = 0;
    double   rtt_ms = 0.0;
    bool     up = false;
};

/* 链路抽象：真实工程里 Udp/Tcp/Serial 各自实现，这里统一成同一接口 */
class Link {
public:
    virtual ~Link() = default;
    virtual LinkKind kind() const = 0;
    virtual const std::string &name() const = 0;
    virtual bool send(const Frame &f) = 0;
    virtual bool recv(Frame &out) = 0;
    virtual bool connected() const = 0;
    virtual void tick(double dt) = 0;             /* 推进链路状态（重连等） */
    virtual const LinkStat &stat() const = 0;
    virtual void set_up(bool up) { force_up_ = up; }
protected:
    bool force_up_ = false;
};

/* 通用内存链路：可注入延迟/丢包，用于测试与仿真 */
class MemoryLink : public Link {
public:
    MemoryLink(std::string name, LinkKind kind, double delay_s = 0.0,
               double loss_ratio = 0.0, uint32_t seed = 1);
    LinkKind kind() const override { return kind_; }
    const std::string &name() const override { return name_; }
    bool send(const Frame &f) override;
    bool recv(Frame &out) override;
    bool connected() const override { return up_; }
    void tick(double dt) override;
    const LinkStat &stat() const override { return stat_; }
    /* 测试辅助：把帧直接投进接收队列（模拟对端发来） */
    void inject(const Frame &f);
    /* 传输层可用性（网线/电台是否在线）——与"会话是否建立"是两件事 */
    void set_connected(bool up) {
        transport_up_ = up;
        if (!up) { up_ = false; stat_.up = false; }
    }
    /* 重连尝试：只有传输层在线才可能重建会话 */
    void set_up(bool up) override {
        force_up_ = up;
        up_ = up && transport_up_;
        stat_.up = up_;
    }
private:
    struct Pending { double due; std::vector<uint8_t> bytes; };
    std::string name_;
    LinkKind    kind_;
    double      delay_s_, loss_ratio_;
    uint32_t    seed_;
    bool        up_ = true;              /* 会话是否建立 */
    bool        transport_up_ = true;    /* 传输层是否在线 */
    std::deque<Pending> tx_q_;
    FrameParser parser_;
    std::deque<Frame> rx_q_;
    LinkStat    stat_;
    double      urand();
};

/* ============================================== 可靠性 reliable（冗余/重连） */
struct LinkHealth {
    std::string name;
    bool        up = false;
    double      last_rx_at = -1.0;
    uint32_t    consecutive_fail = 0;
    double      next_retry_at = 0.0;
    double      backoff_s = 0.5;      /* 指数退避起点 */
    uint32_t    reconnects = 0;
    uint32_t    failovers = 0;        /* 从主链路切走的次数 */
};

/* 序号连续性统计：检测丢包与乱序 */
struct SeqTracker {
    bool     started = false;
    uint16_t expect = 0;
    uint64_t received = 0, lost = 0, reordered = 0, duplicated = 0;
    /* 返回：0 正常，1 发现丢包，2 乱序，3 重复 */
    int observe(uint16_t seq, uint16_t *missing = nullptr);
    double loss_ratio() const {
        uint64_t total = received + lost;
        return total ? static_cast<double>(lost) / total : 0.0;
    }
};

/* ======================================== 通信中间件 router（汇聚/分发/管理） */
struct DeviceRecord {
    std::string id;
    double   last_seen = -1.0;
    uint32_t frames = 0;
    double   depth_m = 0.0, battery = 0.0, heading_deg = 0.0;
    std::string active_link;
};

struct RouterConfig {
    double   heartbeat_timeout_s = 3.0;   /* 超时判定失联   */
    uint32_t max_reconnect_tries = 5;
    double   backoff_max_s = 8.0;
    bool     prefer_primary = true;       /* 主链路恢复后自动切回 */
};

class Router {
public:
    Router(RouterConfig cfg, std::vector<std::shared_ptr<Link>> links,
           std::string primary_name = "");
    /* 岸基下发指令：选链路 -> 编码 -> 发送 -> 记录日志 */
    bool send_command(const std::string &device_id, MsgType type,
                      const std::vector<uint8_t> &payload);
    /* 轮询所有链路收帧，做校验/序号跟踪/设备状态更新；返回本 tick 收到帧数 */
    uint32_t poll(double now);
    /* 推进链路状态：心跳超时判定、指数退避重连、主备切换 */
    void     tick(double now, double dt);

    const std::map<std::string, DeviceRecord>  &devices() const { return devices_; }
    const std::map<std::string, LinkHealth>    &health()  const { return health_; }
    const SeqTracker                           &seq()     const { return seq_; }
    const std::string                          &active_link() const { return active_; }
    const std::vector<std::string>             &log()     const { return log_; }
    uint64_t commands_sent() const { return commands_sent_; }
    uint64_t acks_recv()     const { return acks_recv_; }

private:
    RouterConfig cfg_;
    std::vector<std::shared_ptr<Link>> links_;
    std::map<std::string, LinkHealth> health_;
    std::map<std::string, DeviceRecord> devices_;
    SeqTracker seq_;
    std::string primary_name_;      /* 主链路名（用于故障恢复后切回） */
    std::string active_;
    std::vector<std::string> log_;
    uint16_t next_seq_ = 0;
    uint64_t commands_sent_ = 0, acks_recv_ = 0;
    std::shared_ptr<Link> find(const std::string &name) const;
    void log_line(const std::string &s);
};

/* ============================================================== 存储 store */
struct TelemetryRow {
    double t;
    std::string device_id;
    double depth_m, battery, heading_deg;
    std::string link;
};

class Store {
public:
    explicit Store(std::string path) : path_(std::move(path)) {}
    void insert_telemetry(const TelemetryRow &r);
    void insert_command(double t, const std::string &device_id,
                        const std::string &text);
    void insert_link_event(double t, const std::string &link,
                           const std::string &event);
    size_t telemetry_count() const { return telemetry_.size(); }
    size_t command_count()   const { return commands_.size(); }
    size_t event_count()     const { return events_.size(); }
    const std::vector<TelemetryRow> &telemetry() const { return telemetry_; }
    /* 落盘为 journal 文件（可用 SQLite 后端替换，见 README） */
    bool flush();
    /* 简单统计：按设备聚合遥测条数与平均深度 */
    std::map<std::string, std::pair<size_t, double>> depth_summary() const;
private:
    std::string path_;
    std::vector<TelemetryRow> telemetry_;
    std::vector<std::tuple<double, std::string, std::string>> commands_;
    std::vector<std::tuple<double, std::string, std::string>> events_;
};

} /* namespace auv */

#endif /* AUV_H */
