#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "Channels.h"
#include "Crc8.h"
#include "Frame.h"

/**
 * Frame builders. The RC-channels builder is what the tests and the self-test feed
 * into the parser; telemetry builders (battery, flight mode) belong here too when
 * the TX side gets wired.
 */
namespace crsf {

inline constexpr size_t kRcChannelsFrameSize = kHeaderSize + 1 + kChannelsPayload + 1;  // 26

using RcChannelsFrame = std::array<uint8_t, kRcChannelsFrameSize>;

/** Build a complete 0x16 frame (sync, len, type, 22-byte payload, crc) from 16 channel values. */
constexpr RcChannelsFrame buildRcChannelsFrame(const Channels& ticks) noexcept {
    RcChannelsFrame frame{};
    frame[0] = kSyncByte;
    frame[1] = static_cast<uint8_t>(1 + kChannelsPayload + 1);  // type + payload + crc = 24
    frame[2] = static_cast<uint8_t>(FrameType::RcChannels);

    const ChannelsPayload payload = packChannels(ticks);
    for (size_t i = 0; i < kChannelsPayload; ++i) {
        frame[kHeaderSize + 1 + i] = payload[i];
    }

    uint8_t crc = 0;
    for (size_t i = kHeaderSize; i + 1 < kRcChannelsFrameSize; ++i) {
        crc = kCrc8Table[static_cast<uint8_t>(crc ^ frame[i])];
    }
    frame[kRcChannelsFrameSize - 1] = crc;
    return frame;
}

}  // namespace crsf
