#pragma once

#include <cstddef>
#include <cstdint>

#include "Crc8.h"
#include "Frame.h"

namespace crsf {

/**
 * Byte-fed CRSF frame parser: O(1) per byte, fixed 64-byte buffer, no heap.
 *
 * feed() returns true when a complete, CRC-valid frame is ready. Read type(),
 * payload() and payloadSize() BEFORE feeding the next byte, because the buffer
 * is reused. Callers must check payloadSize() against what the frame type
 * requires; the parser only validates the length range and the CRC.
 *
 * Resync: garbage is dropped until a sync byte appears. A frame that fails its
 * length or CRC check is discarded and counted in errors().
 */
class Parser {
public:
    constexpr Parser() noexcept = default;

    constexpr bool feed(uint8_t byte) noexcept {
        if (pos_ == 0) {
            if (byte == kSyncByte) {
                buf_[pos_++] = byte;
            }
            return false;
        }
        if (pos_ == 1) {
            if (byte < kMinLength || byte > kMaxLength) {
                ++errors_;
                // A bad length that is itself a sync byte most likely starts the real frame.
                pos_ = (byte == kSyncByte) ? 1 : 0;
                return false;
            }
            buf_[pos_++] = byte;
            return false;
        }
        buf_[pos_++]       = byte;
        const size_t total = static_cast<size_t>(buf_[1]) + kHeaderSize;
        if (pos_ < total) {
            return false;
        }
        pos_ = 0;
        // CRC covers type + payload: bytes [2, total - 1).
        uint8_t crc = 0;
        for (size_t i = kHeaderSize; i + 1 < total; ++i) {
            crc = kCrc8Table[static_cast<uint8_t>(crc ^ buf_[i])];
        }
        if (crc != buf_[total - 1]) {
            ++errors_;
            return false;
        }
        return true;
    }

    /** Drop any partial frame (e.g. after a UART overflow). */
    constexpr void reset() noexcept { pos_ = 0; }

    constexpr uint8_t        type() const noexcept { return buf_[2]; }
    constexpr const uint8_t* payload() const noexcept { return &buf_[3]; }
    constexpr size_t         payloadSize() const noexcept { return static_cast<size_t>(buf_[1]) - 2; }
    constexpr uint8_t        payloadByte(size_t index) const noexcept { return buf_[3 + index]; }
    constexpr uint32_t       errors() const noexcept { return errors_; }

private:
    uint8_t  buf_[kMaxFrameSize]{};
    size_t   pos_    = 0;
    uint32_t errors_ = 0;
};

}  // namespace crsf
