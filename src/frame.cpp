/* frame.cpp — 帧协议：同步头 + 长度 + 序号 + 载荷 + CRC16，含流式解帧 */
#include "auv.h"
#include <cstring>

namespace auv {

const char *msg_type_name(MsgType t)
{
    switch (t) {
    case MsgType::Heartbeat: return "HEARTBEAT";
    case MsgType::Telemetry: return "TELEMETRY";
    case MsgType::Command:   return "COMMAND";
    case MsgType::Ack:       return "ACK";
    case MsgType::Event:     return "EVENT";
    case MsgType::Bulk:      return "BULK";
    }
    return "?";
}

/* CRC-16/CCITT-FALSE：poly 0x1021, init 0xFFFF —— 工业串口/水下遥测常用口径 */
uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; b++) {
            if (crc & 0x8000) crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
            else              crc = static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

size_t frame_encode(const Frame &f, uint8_t *out, size_t out_len)
{
    const size_t plen = f.payload.size();
    if (plen > FRAME_MAX_PAYLOAD) return 0;
    const size_t total = FRAME_HEADER_LEN + plen + FRAME_CRC_LEN;
    if (out_len < total) return 0;

    out[0] = FRAME_SYNC0;
    out[1] = FRAME_SYNC1;
    out[2] = static_cast<uint8_t>(f.type);
    out[3] = static_cast<uint8_t>(f.seq & 0xFF);
    out[4] = static_cast<uint8_t>((f.seq >> 8) & 0xFF);
    out[5] = static_cast<uint8_t>(plen & 0xFF);
    out[6] = static_cast<uint8_t>((plen >> 8) & 0xFF);
    out[7] = 0;                                  /* 预留（版本/标志位） */
    if (plen) std::memcpy(out + FRAME_HEADER_LEN, f.payload.data(), plen);

    const uint16_t crc = crc16_ccitt(out, FRAME_HEADER_LEN + plen);
    out[FRAME_HEADER_LEN + plen]     = static_cast<uint8_t>(crc & 0xFF);
    out[FRAME_HEADER_LEN + plen + 1] = static_cast<uint8_t>((crc >> 8) & 0xFF);
    return total;
}

bool frame_decode(const uint8_t *buf, size_t len, Frame &f)
{
    if (len < FRAME_HEADER_LEN + FRAME_CRC_LEN) return false;
    if (buf[0] != FRAME_SYNC0 || buf[1] != FRAME_SYNC1) return false;

    const uint16_t plen = static_cast<uint16_t>(buf[5] | (buf[6] << 8));
    if (plen > FRAME_MAX_PAYLOAD) return false;
    if (len != FRAME_HEADER_LEN + plen + FRAME_CRC_LEN) return false;

    const uint16_t crc_rx = static_cast<uint16_t>(
        buf[FRAME_HEADER_LEN + plen] | (buf[FRAME_HEADER_LEN + plen + 1] << 8));
    if (crc_rx != crc16_ccitt(buf, FRAME_HEADER_LEN + plen)) return false;

    f.type = static_cast<MsgType>(buf[2]);
    f.seq  = static_cast<uint16_t>(buf[3] | (buf[4] << 8));
    f.payload.assign(buf + FRAME_HEADER_LEN, buf + FRAME_HEADER_LEN + plen);
    return true;
}

/* ------------------------------------------------------------- FrameParser */
void FrameParser::feed(const uint8_t *data, size_t len)
{
    buf_.insert(buf_.end(), data, data + len);
}

bool FrameParser::next(Frame &out)
{
    while (true) {
        if (buf_.size() < 2) return false;

        /* 找同步头，丢弃前面的垃圾字节（重同步） */
        size_t i = 0;
        while (i + 1 < buf_.size()
               && !(buf_[i] == FRAME_SYNC0 && buf_[i + 1] == FRAME_SYNC1)) {
            i++;
        }
        if (i > 0) {
            buf_.erase(buf_.begin(), buf_.begin() + static_cast<long>(i));
            resyncs_++;
        }
        if (buf_.size() < FRAME_HEADER_LEN) return false;

        const uint16_t plen = static_cast<uint16_t>(buf_[5] | (buf_[6] << 8));
        if (plen > FRAME_MAX_PAYLOAD) {           /* 长度非法：跳过一个字节重找 */
            buf_.erase(buf_.begin());
            resyncs_++;
            continue;
        }
        const size_t need = FRAME_HEADER_LEN + plen + FRAME_CRC_LEN;
        if (buf_.size() < need) return false;     /* 半包：等更多数据 */

        std::vector<uint8_t> one(buf_.begin(), buf_.begin() + static_cast<long>(need));
        buf_.erase(buf_.begin(), buf_.begin() + static_cast<long>(need));

        if (frame_decode(one.data(), one.size(), out)) return true;
        crc_errors_++;                            /* CRC 错：丢弃并继续 */
    }
}

} /* namespace auv */
