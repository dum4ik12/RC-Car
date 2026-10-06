#pragma once

#include <cstddef>
#include <cstdint>

/**
 * CRSF frame layout and protocol constants (TBS CRSF spec, ExpressLRS 3.x).
 *
 *   [sync/addr] [len] [type] [payload ...] [crc8]
 *      1 byte   1 byte 1 byte  len-2 bytes  1 byte
 *
 * - len counts the bytes after the len byte: type + payload + crc, 2..62.
 * - crc8 (CRC-8/DVB-S2) covers type + payload only.
 * - Multi-byte fields are big-endian, except the packed RC channels.
 */
namespace crsf {

inline constexpr uint32_t kDefaultBaudRate = 420000;  // ExpressLRS default, 8N1, not inverted

inline constexpr uint8_t kSyncByte     = 0xC8;  // flight-controller address: frames addressed to us start with it
inline constexpr size_t  kMaxFrameSize = 64;
inline constexpr size_t  kHeaderSize   = 2;  // sync + len
inline constexpr uint8_t kMinLength    = 2;  // type + crc (empty payload)
inline constexpr uint8_t kMaxLength    = static_cast<uint8_t>(kMaxFrameSize - kHeaderSize);
inline constexpr size_t  kMaxPayload   = kMaxLength - 2;

enum class Address : uint8_t {
    Broadcast        = 0x00,
    FlightController = 0xC8,
    Radio            = 0xEA,
    Receiver         = 0xEC,
    TxModule         = 0xEE,
};

enum class FrameType : uint8_t {
    Gps        = 0x02,  // to the radio, 15 B
    Battery    = 0x08,  // to the radio, 8 B
    LinkStats  = 0x14,  // receiver -> us, 10 B
    RcChannels = 0x16,  // receiver -> us, 22 B
    Attitude   = 0x1E,  // to the radio, 6 B
    FlightMode = 0x21,  // to the radio, NUL-terminated string
};

// 0x16 RC channels packed: 16 channels x 11 bits, LSB-first.
inline constexpr size_t   kChannelCount    = 16;
inline constexpr size_t   kChannelsPayload = 22;
inline constexpr uint16_t kTicksMask       = 0x7FF;
inline constexpr uint16_t kTicksMin        = 172;   // ~988 us
inline constexpr uint16_t kTicksCenter     = 992;   // 1500 us
inline constexpr uint16_t kTicksMax        = 1811;  // ~2012 us
inline constexpr uint16_t kUsCenter        = 1500;

// 0x14 link statistics.
inline constexpr size_t kLinkStatsPayload = 10;

}  // namespace crsf
