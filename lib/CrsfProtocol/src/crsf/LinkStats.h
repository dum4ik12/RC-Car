#pragma once

#include <cstdint>

#include "Frame.h"

namespace crsf {

/** 0x14 link statistics. RSSI fields are positive numbers of -dBm (70 means -70 dBm). */
struct LinkStats {
    uint8_t uplinkRssi1;    // antenna 1, -dBm
    uint8_t uplinkRssi2;    // antenna 2, -dBm
    uint8_t uplinkLq;       // link quality, %
    int8_t  uplinkSnr;      // dB
    uint8_t activeAntenna;  // 0 / 1
    uint8_t rfMode;         // packet-rate enum
    uint8_t uplinkTxPower;  // enum
    uint8_t downlinkRssi;   // -dBm
    uint8_t downlinkLq;     // %
    int8_t  downlinkSnr;    // dB
};

/** Decode a 0x14 payload. @p payload must hold at least kLinkStatsPayload bytes. */
constexpr LinkStats decodeLinkStats(const uint8_t* payload) noexcept {
    LinkStats stats{};
    stats.uplinkRssi1   = payload[0];
    stats.uplinkRssi2   = payload[1];
    stats.uplinkLq      = payload[2];
    stats.uplinkSnr     = static_cast<int8_t>(payload[3]);
    stats.activeAntenna = payload[4];
    stats.rfMode        = payload[5];
    stats.uplinkTxPower = payload[6];
    stats.downlinkRssi  = payload[7];
    stats.downlinkLq    = payload[8];
    stats.downlinkSnr   = static_cast<int8_t>(payload[9]);
    return stats;
}

}  // namespace crsf
