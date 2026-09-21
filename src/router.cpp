/* router.cpp — 通信中间件：多航行器数据汇聚、指令分发、链路管理与可靠性
 *
 * 可靠性三件事（对应岗位职责第 4 条）：
 *   1) 冗余设计  —— 主链路 + 备用链路；主链路不可用时自动切换
 *   2) 断线重连  —— 心跳超时判定失联，指数退避重连，主链路恢复后自动切回
 *   3) 数据校验  —— CRC16（帧层）+ 序号连续性检测（发现丢包/乱序/重复）+ 事件日志
 */
#include "auv.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <sstream>

namespace auv {

/* ------------------------------------------------------------ SeqTracker */
int SeqTracker::observe(uint16_t seq, uint16_t *missing)
{
    if (missing) *missing = 0;
    if (!started) {
        started = true;
        expect = static_cast<uint16_t>(seq + 1);
        received++;
        return 0;
    }
    if (seq == expect) {
        expect = static_cast<uint16_t>(expect + 1);
        received++;
        return 0;
    }
    /* 用 16 位环绕差值判断前后 */
    const uint16_t diff = static_cast<uint16_t>(seq - expect);   /* 有符号语义 */
    if (diff == 0xFFFF) {                     /* seq + 1 == expect：重复帧 */
        duplicated++;
        return 3;
    }
    if (diff < 0x8000) {                      /* seq 在 expect 之后：中间有丢包 */
        const uint16_t lost_n = diff;
        lost += lost_n;
        if (missing) *missing = lost_n;
        expect = static_cast<uint16_t>(seq + 1);
        received++;
        return 1;
    }
    /* seq 在 expect 之前：乱序到达 */
    reordered++;
    return 2;
}

/* ---------------------------------------------------------------- Router */
Router::Router(RouterConfig cfg, std::vector<std::shared_ptr<Link>> links,
               std::string primary_name)
    : cfg_(cfg), links_(std::move(links))
{
    for (auto &l : links_) {
        LinkHealth h;
        h.name = l->name();
        h.up = l->connected();
        h.backoff_s = 0.5;
        health_[h.name] = h;
    }
    primary_name_ = primary_name;
    if (!primary_name.empty() && health_.count(primary_name)) {
        active_ = primary_name;
    } else if (!links_.empty()) {
        active_ = links_.front()->name();
    }
    health_[active_].failovers = 0;
    log_line("router started, primary=" + std::string(active_.empty() ? "none" : active_)
             + ", links=" + std::to_string(links_.size()));
}

std::shared_ptr<Link> Router::find(const std::string &name) const
{
    for (auto &l : links_) if (l->name() == name) return l;
    return nullptr;
}

void Router::log_line(const std::string &s)
{
    log_.push_back(s);
    if (log_.size() > 512) log_.erase(log_.begin());
}

bool Router::send_command(const std::string &device_id, MsgType type,
                          const std::vector<uint8_t> &payload)
{
    auto link = find(active_);
    if (!link || !link->connected()) {
        log_line("send failed: no active link (" + active_ + ")");
        return false;
    }
    Frame f;
    f.type = type;
    f.seq = next_seq_++;
    f.payload = payload;
    const bool ok = link->send(f);
    if (ok) commands_sent_++;
    std::ostringstream os;
    os << (ok ? "TX " : "TX-FAIL ") << link_kind_name(link->kind())
       << " link=" << active_ << " dev=" << device_id
       << " type=" << msg_type_name(type) << " seq=" << f.seq
       << " bytes=" << payload.size();
    log_line(os.str());
    return ok;
}

uint32_t Router::poll(double now)
{
    uint32_t got = 0;
    for (auto &l : links_) {
        Frame f;
        while (l->recv(f)) {
            got++;
            auto &h = health_[l->name()];
            h.up = true;
            h.last_rx_at = now;
            h.consecutive_fail = 0;
            h.backoff_s = 0.5;                       /* 有数据即重置退避 */

            uint16_t missing = 0;
            const int st = seq_.observe(f.seq, &missing);
            if (st == 1) {
                log_line("SEQ GAP on " + l->name() + " lost=" + std::to_string(missing)
                         + " expect=" + std::to_string(seq_.expect));
            } else if (st == 2) {
                log_line("SEQ REORDER on " + l->name() + " seq=" + std::to_string(f.seq));
            } else if (st == 3) {
                log_line("SEQ DUP on " + l->name() + " seq=" + std::to_string(f.seq));
            }

            /* 数据汇聚：只有遥测/心跳/事件帧带 [id_len][id...] 设备头，
             * Command / Ack 的载荷格式不同，不能按设备解析（否则会生成假设备）。 */
            const bool is_device_frame = (f.type == MsgType::Telemetry
                                       || f.type == MsgType::Heartbeat
                                       || f.type == MsgType::Event);
            if (!is_device_frame) {
                if (f.type == MsgType::Ack) acks_recv_++;
                continue;
            }
            std::string dev = "unknown";
            if (f.payload.size() >= 1) {
                /* 载荷约定：[id_len][id...][depth:f32][battery:f32][heading:f32] */
                const size_t idlen = f.payload[0];
                if (f.payload.size() >= 1 + idlen) {
                    dev.assign(reinterpret_cast<const char *>(f.payload.data() + 1), idlen);
                }
            }
            auto &d = devices_[dev];
            d.id = dev;
            d.last_seen = now;
            d.frames++;
            d.active_link = l->name();
            if (f.payload.size() >= 1 + f.payload[0] + 12) {
                const uint8_t *p = f.payload.data() + 1 + f.payload[0];
                float depth, batt, head;
                std::memcpy(&depth, p, 4);
                std::memcpy(&batt, p + 4, 4);
                std::memcpy(&head, p + 8, 4);
                d.depth_m = depth;
                d.battery = batt;
                d.heading_deg = head;
            }
            if (f.type == MsgType::Ack) acks_recv_++;
        }
    }
    return got;
}

void Router::tick(double now, double dt)
{
    (void)dt;
    for (auto &kv : health_) {
        auto &h = kv.second;
        auto link = find(h.name);
        if (!link) continue;

        /* 链路物理状态为 down -> 标记失联并**按退避**安排重连
         * （关键：首次重试排在 now + backoff_s，不能在同一个 tick 内立刻重连，
         *   否则退避形同虚设） */
        if (!link->connected()) {
            if (h.up) {
                h.up = false;
                h.backoff_s = 0.5;                    /* 新一轮退避起点 */
                log_line("LINK DOWN " + h.name + " (transport closed)");
            }
            if (h.next_retry_at == 0.0) h.next_retry_at = now + h.backoff_s;
        }

        /* 心跳超时 -> 判定失联 */
        if (h.up && h.last_rx_at >= 0.0
            && now - h.last_rx_at > cfg_.heartbeat_timeout_s) {
            h.up = false;
            h.backoff_s = 0.5;
            log_line("LINK DOWN " + h.name + " (heartbeat timeout)");
            if (h.next_retry_at == 0.0) h.next_retry_at = now + h.backoff_s;
        }

        /* 断线重连：指数退避（0.5s -> 1s -> 2s -> ... 上限 backoff_max_s）
         * 关键：尝试后必须**回读链路是否真的可用**，否则会把"假成功"记成重连，
         *      在传输层仍然断开时反复震荡。 */
        if (!h.up && h.next_retry_at > 0.0 && now >= h.next_retry_at) {
            const double used_backoff = h.backoff_s;
            link->set_up(true);
            if (link->connected()) {
                h.up = true;
                h.next_retry_at = 0.0;
                h.reconnects++;
                h.backoff_s = 0.5;                 /* 成功后重置退避 */
                log_line("LINK RECONNECT " + h.name
                         + " (attempt " + std::to_string(h.reconnects) + ")");
            } else {
                /* 传输层仍不可用：退避翻倍后继续重试 */
                h.next_retry_at = now + h.backoff_s;
                h.backoff_s = std::min(h.backoff_s * 2.0, cfg_.backoff_max_s);
                log_line("RECONNECT PENDING " + h.name
                         + " (transport down, next in "
                         + std::to_string(h.backoff_s) + "s)");
            }
            (void)used_backoff;
        }
    }

    /* 冗余切换：当前链路不可用则切到其它可用链路 */
    auto cur = health_.find(active_);
    if (cur != health_.end() && !cur->second.up) {
        for (auto &kv : health_) {
            if (kv.first != active_ && kv.second.up) {
                cur->second.failovers++;
                log_line("FAILOVER " + active_ + " -> " + kv.first);
                active_ = kv.first;
                break;
            }
        }
    } else if (cfg_.prefer_primary && !primary_name_.empty()
               && active_ != primary_name_) {
        /* 主链路恢复后切回 */
        auto p = health_.find(primary_name_);
        if (p != health_.end() && p->second.up) {
            log_line("FAILBACK " + active_ + " -> " + primary_name_);
            active_ = primary_name_;
        }
    }
}

/* ---------------------------------------------------------------- Store */
void Store::insert_telemetry(const TelemetryRow &r) { telemetry_.push_back(r); }

void Store::insert_command(double t, const std::string &device_id,
                           const std::string &text)
{
    commands_.emplace_back(t, device_id, text);
}

void Store::insert_link_event(double t, const std::string &link,
                              const std::string &event)
{
    events_.emplace_back(t, link, event);
}

bool Store::flush()
{
    FILE *fp = std::fopen(path_.c_str(), "w");
    if (!fp) return false;

    std::fprintf(fp, "# AUV 通信后台 journal (j 形同 CSV，可用 SQLite 后端替换)\n");
    std::fprintf(fp, "# telemetry: t,device,depth_m,battery,heading_deg,link\n");
    for (const auto &r : telemetry_) {
        std::fprintf(fp, "T,%.6f,%s,%.3f,%.2f,%.1f,%s\n",
                     r.t, r.device_id.c_str(), r.depth_m, r.battery,
                     r.heading_deg, r.link.c_str());
    }
    std::fprintf(fp, "# commands: t,device,text\n");
    for (const auto &c : commands_) {
        std::fprintf(fp, "C,%.6f,%s,%s\n",
                     std::get<0>(c), std::get<1>(c).c_str(), std::get<2>(c).c_str());
    }
    std::fprintf(fp, "# link_events: t,link,event\n");
    for (const auto &e : events_) {
        std::fprintf(fp, "E,%.6f,%s,%s\n",
                     std::get<0>(e), std::get<1>(e).c_str(), std::get<2>(e).c_str());
    }
    std::fclose(fp);
    return true;
}

std::map<std::string, std::pair<size_t, double>> Store::depth_summary() const
{
    std::map<std::string, std::pair<size_t, double>> out;
    for (const auto &r : telemetry_) {
        auto &v = out[r.device_id];
        v.first += 1;
        v.second += r.depth_m;
    }
    for (auto &kv : out) {
        if (kv.second.first) kv.second.second /= static_cast<double>(kv.second.first);
    }
    return out;
}

const char *link_kind_name(LinkKind k)
{
    switch (k) {
    case LinkKind::Udp:      return "UDP";
    case LinkKind::Tcp:      return "TCP";
    case LinkKind::Serial:   return "SERIAL";
    case LinkKind::Acoustic: return "ACOUSTIC";
    }
    return "?";
}

} /* namespace auv */
