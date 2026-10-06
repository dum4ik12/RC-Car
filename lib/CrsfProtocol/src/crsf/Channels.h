#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "Frame.h"

namespace crsf {

using Channels        = std::array<uint16_t, kChannelCount>;   // raw 11-bit ticks per channel
using ChannelsPayload = std::array<uint8_t, kChannelsPayload>;  // packed 0x16 payload

/** 11-bit ticks -> pulse width in µs: 172 -> 988, 992 -> 1500, 1811 -> 2011 (truncating). */
constexpr uint16_t ticksToUs(uint16_t ticks) noexcept {
    return static_cast<uint16_t>(kUsCenter + (static_cast<int32_t>(ticks) - kTicksCenter) * 5 / 8);
}

/** Pulse width in µs -> 11-bit ticks (inverse of ticksToUs, truncating). */
constexpr uint16_t usToTicks(uint16_t us) noexcept {
    return static_cast<uint16_t>(kTicksCenter + (static_cast<int32_t>(us) - kUsCenter) * 8 / 5);
}

/** Unpack a 22-byte 0x16 payload. @p payload must hold at least kChannelsPayload bytes. */
constexpr void unpackChannels(const uint8_t* payload, Channels& ticks) noexcept {
    uint32_t bits     = 0;
    uint32_t bitCount = 0;
    for (uint16_t& channel : ticks) {
        while (bitCount < 11) {
            bits |= static_cast<uint32_t>(*payload++) << bitCount;
            bitCount += 8;
        }
        channel = static_cast<uint16_t>(bits & kTicksMask);
        bits >>= 11;
        bitCount -= 11;
    }
}

constexpr Channels unpackChannels(const ChannelsPayload& payload) noexcept {
    Channels ticks{};
    unpackChannels(payload.data(), ticks);
    return ticks;
}

/** Pack 16 channels into a 0x16 payload (the inverse of unpackChannels; used by tests and telemetry). */
constexpr ChannelsPayload packChannels(const Channels& ticks) noexcept {
    ChannelsPayload payload{};
    uint32_t        bits     = 0;
    uint32_t        bitCount = 0;
    size_t          out      = 0;
    for (const uint16_t channel : ticks) {
        bits |= static_cast<uint32_t>(channel & kTicksMask) << bitCount;
        bitCount += 11;
        while (bitCount >= 8) {
            payload[out++] = static_cast<uint8_t>(bits & 0xFF);
            bits >>= 8;
            bitCount -= 8;
        }
    }
    return payload;  // 16 x 11 = 176 bits = 22 bytes exactly, nothing is left over
}

static_assert(ticksToUs(kTicksMin) == 988, "ticksToUs(172)");
static_assert(ticksToUs(kTicksCenter) == 1500, "ticksToUs(992)");
static_assert(ticksToUs(kTicksMax) == 2011, "ticksToUs(1811)");
static_assert(usToTicks(1500) == kTicksCenter, "usToTicks(1500)");

namespace detail {
constexpr bool channelsRoundTrip() noexcept {
    const Channels in{{172, 992, 1811, 0, 2047, 1000, 1500, 500, 191, 1792, 992, 992, 172, 1811, 1024, 1}};
    const Channels out = unpackChannels(packChannels(in));
    for (size_t i = 0; i < kChannelCount; ++i) {
        if (in[i] != out[i]) {
            return false;
        }
    }
    return true;
}
}  // namespace detail

static_assert(detail::channelsRoundTrip(), "packChannels/unpackChannels round trip");

}  // namespace crsf
