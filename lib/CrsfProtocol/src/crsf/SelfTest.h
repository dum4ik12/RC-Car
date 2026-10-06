#pragma once

#include <cstddef>
#include <cstdint>

#include "Builder.h"
#include "Channels.h"
#include "Parser.h"

/**
 * Compile-time parser checks. Every function is constexpr, so a
 * `static_assert(crsf::selftest::run())` verifies the parser in the firmware
 * build itself, with no host toolchain needed. The native Unity tests repeat
 * these cases with more variations.
 */
namespace crsf::selftest {

inline constexpr Channels kTestChannels{{172, 992, 1811, 1500, 500, 1000, 992, 992,
                                         172, 1811, 700, 1300, 992, 992, 992, 992}};

/** A valid frame yields exactly one frame, on its last byte, with the channels intact. */
constexpr bool validFrameIsParsedOnce() noexcept {
    const RcChannelsFrame frame = buildRcChannelsFrame(kTestChannels);
    Parser                parser;
    int                   frames = 0;
    for (size_t i = 0; i < frame.size(); ++i) {
        if (parser.feed(frame[i])) {
            ++frames;
            if (i != frame.size() - 1) {
                return false;
            }
        }
    }
    if (frames != 1 || parser.errors() != 0) {
        return false;
    }
    if (parser.type() != static_cast<uint8_t>(FrameType::RcChannels) || parser.payloadSize() != kChannelsPayload) {
        return false;
    }
    Channels ticks{};
    unpackChannels(parser.payload(), ticks);
    for (size_t i = 0; i < kChannelCount; ++i) {
        if (ticks[i] != kTestChannels[i]) {
            return false;
        }
    }
    return true;
}

/** One flipped payload bit: no frame, one error. */
constexpr bool corruptedFrameIsRejected() noexcept {
    RcChannelsFrame frame = buildRcChannelsFrame(kTestChannels);
    frame[10] ^= 0x04;
    Parser parser;
    for (const uint8_t byte : frame) {
        if (parser.feed(byte)) {
            return false;
        }
    }
    return parser.errors() == 1;
}

/** Garbage, a fake sync byte with a bad length, then a valid frame: the frame still parses. */
constexpr bool resyncsAfterGarbage() noexcept {
    const uint8_t garbage[] = {0x00, 0xFF, kSyncByte, 0x00, 0x13, kSyncByte, 0x7F, 0x42};
    Parser        parser;
    for (const uint8_t byte : garbage) {
        if (parser.feed(byte)) {
            return false;
        }
    }
    const RcChannelsFrame frame  = buildRcChannelsFrame(kTestChannels);
    int                   frames = 0;
    for (const uint8_t byte : frame) {
        if (parser.feed(byte)) {
            ++frames;
        }
    }
    return frames == 1 && parser.payloadSize() == kChannelsPayload;
}

/** Two frames back to back are both delivered. */
constexpr bool backToBackFrames() noexcept {
    const RcChannelsFrame frame  = buildRcChannelsFrame(kTestChannels);
    Parser                parser;
    int                   frames = 0;
    for (int n = 0; n < 2; ++n) {
        for (const uint8_t byte : frame) {
            if (parser.feed(byte)) {
                ++frames;
            }
        }
    }
    return frames == 2 && parser.errors() == 0;
}

/** A sync byte in the length slot starts a new frame instead of being dropped. */
constexpr bool syncByteAsLengthRestartsFrame() noexcept {
    const RcChannelsFrame frame  = buildRcChannelsFrame(kTestChannels);
    Parser                parser;
    int                   frames = 0;
    if (parser.feed(kSyncByte)) {  // stray sync, immediately followed by the real frame
        return false;
    }
    for (const uint8_t byte : frame) {
        if (parser.feed(byte)) {
            ++frames;
        }
    }
    return frames == 1;
}

constexpr bool run() noexcept {
    return validFrameIsParsedOnce() && corruptedFrameIsRejected() && resyncsAfterGarbage() && backToBackFrames() &&
           syncByteAsLengthRestartsFrame();
}

}  // namespace crsf::selftest
