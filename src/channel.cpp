/* channel.cpp — 水声信道仿真 + 通用内存链路
 *
 * 水声通信的真实约束（这也是本科目与普通网络服务最大的区别）：
 *   · 声速约 1500 m/s —— 2 km 距离单程就要 1.3 s，RTT 超过 2.6 s
 *   · 带宽低（常见 9.6 kbps 量级）—— 一帧几百字节就要占用百毫秒级信道
 *   · 丢包与抖动明显，且受多径与噪声影响
 * 因此上层必须做：序号连续性检测、超时重传/确认、冗余链路切换，而不能假设"网络很快"。
 */
#include "auv.h"
#include <algorithm>
#include <cmath>

namespace auv {

/* ------------------------------------------------------------ 参数计算 */
double acoustic_propagation_delay(const AcousticConfig &c)
{
    if (c.speed_of_sound <= 0.0) return 0.0;
    return c.range_m / c.speed_of_sound;
}

double acoustic_tx_time(const AcousticConfig &c, size_t frame_bytes)
{
    if (c.bitrate_bps <= 0.0) return 0.0;
    return (frame_bytes * 8.0) / c.bitrate_bps;
}

/* ------------------------------------------------------- AcousticChannel */
AcousticChannel::AcousticChannel(AcousticConfig c)
    : cfg_(c), rng_state_(c.seed ? c.seed : 1)
{
}

double AcousticChannel::urand()
{
    /* xorshift64*：确定性、可复现 */
    rng_state_ ^= rng_state_ >> 12;
    rng_state_ ^= rng_state_ << 25;
    rng_state_ ^= rng_state_ >> 27;
    const uint64_t x = rng_state_ * 2685821657736338717ULL;
    return static_cast<double>(x >> 11) / 9007199254740992.0;   /* [0,1) */
}

double AcousticChannel::avg_delay() const
{
    return sent_ ? total_delay_ / sent_ : 0.0;
}

bool AcousticChannel::transmit(const std::vector<uint8_t> &bytes)
{
    sent_++;
    if (urand() < cfg_.loss_ratio) {           /* 信道丢包 */
        dropped_++;
        return false;
    }

    /* 低带宽串行占用：一帧必须等前一个发完才能开始 */
    const double tx = acoustic_tx_time(cfg_, bytes.size());
    const double start = std::max(next_free_, 0.0);
    next_free_ = start + tx;

    /* 传播延迟 + 抖动 */
    const double base = acoustic_propagation_delay(cfg_);
    const double jitter = base * cfg_.jitter_ratio * (urand() * 2.0 - 1.0);
    const double delay = std::max(0.0, base + jitter);

    const double arrive = start + tx + delay;
    inflight_.push_back(Pkt{arrive, bytes});
    std::sort(inflight_.begin(), inflight_.end(),
              [](const Pkt &a, const Pkt &b) { return a.arrive_at < b.arrive_at; });
    total_delay_ += delay;
    return true;
}

bool AcousticChannel::poll_ready(double now, std::vector<uint8_t> &out)
{
    if (inflight_.empty()) return false;
    if (inflight_.front().arrive_at > now) return false;
    out = inflight_.front().bytes;
    inflight_.pop_front();
    return true;
}

/* ------------------------------------------------------------ MemoryLink */
MemoryLink::MemoryLink(std::string name, LinkKind kind, double delay_s,
                       double loss_ratio, uint32_t seed)
    : name_(std::move(name)), kind_(kind), delay_s_(delay_s),
      loss_ratio_(loss_ratio), seed_(seed ? seed : 1)
{
    stat_.up = true;
}

double MemoryLink::urand()
{
    seed_ ^= seed_ >> 12;
    seed_ ^= seed_ << 25;
    seed_ ^= seed_ >> 27;
    const uint32_t x = seed_ * 0x2545F491u;
    return static_cast<double>(x >> 8) / 16777216.0;
}

bool MemoryLink::send(const Frame &f)
{
    if (!up_) { stat_.dropped++; return false; }

    uint8_t buf[FRAME_HEADER_LEN + FRAME_MAX_PAYLOAD + FRAME_CRC_LEN];
    const size_t n = frame_encode(f, buf, sizeof(buf));
    if (n == 0) return false;

    stat_.tx_frames++;
    stat_.tx_bytes += n;

    if (loss_ratio_ > 0.0 && urand() < loss_ratio_) {   /* 注入丢包 */
        stat_.dropped++;
        return false;
    }
    tx_q_.push_back(Pending{delay_s_, std::vector<uint8_t>(buf, buf + n)});
    return true;
}

bool MemoryLink::recv(Frame &out)
{
    if (rx_q_.empty()) return false;
    out = rx_q_.front();
    rx_q_.pop_front();
    stat_.rx_frames++;
    return true;
}

void MemoryLink::tick(double dt)
{
    /* 发送队列 -> 解析器（模拟字节流到达） */
    while (!tx_q_.empty() && tx_q_.front().due <= 0.0) {
        parser_.feed(tx_q_.front().bytes.data(), tx_q_.front().bytes.size());
        tx_q_.pop_front();
    }
    for (auto &p : tx_q_) p.due -= dt;

    Frame f;
    while (parser_.next(f)) rx_q_.push_back(f);

    stat_.crc_errors = parser_.crc_errors();
    stat_.up = up_;
    if (!up_) return;
    /* 收到数据即视为链路存活 */
    if (!rx_q_.empty()) stat_.rtt_ms = delay_s_ * 2000.0;
}

void MemoryLink::inject(const Frame &f)
{
    uint8_t buf[FRAME_HEADER_LEN + FRAME_MAX_PAYLOAD + FRAME_CRC_LEN];
    const size_t n = frame_encode(f, buf, sizeof(buf));
    if (n) parser_.feed(buf, n);
    Frame tmp;
    while (parser_.next(tmp)) rx_q_.push_back(tmp);
}

} /* namespace auv */
