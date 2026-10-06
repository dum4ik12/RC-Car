#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace crsf {

/**
 * CRC-8/DVB-S2 as used by CRSF: polynomial 0xD5, init 0x00, not reflected, no final XOR.
 * The lookup table is computed at compile time; the hot path is one table lookup per byte.
 */
inline constexpr uint8_t kCrc8Poly = 0xD5;

constexpr std::array<uint8_t, 256> makeCrc8Table(uint8_t poly) noexcept {
    std::array<uint8_t, 256> table{};
    for (int i = 0; i < 256; ++i) {
        uint8_t crc = static_cast<uint8_t>(i);
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ poly) : static_cast<uint8_t>(crc << 1);
        }
        table[static_cast<size_t>(i)] = crc;
    }
    return table;
}

inline constexpr std::array<uint8_t, 256> kCrc8Table = makeCrc8Table(kCrc8Poly);

/** CRC of @p size bytes, optionally continuing from a previous @p crc. */
constexpr uint8_t crc8(const uint8_t* data, size_t size, uint8_t crc = 0) noexcept {
    for (size_t i = 0; i < size; ++i) {
        crc = kCrc8Table[static_cast<uint8_t>(crc ^ data[i])];
    }
    return crc;
}

/** CRC of a string literal, without its terminating NUL. Handy for compile-time checks. */
template <size_t N>
constexpr uint8_t crc8(const char (&text)[N]) noexcept {
    uint8_t crc = 0;
    for (size_t i = 0; i + 1 < N; ++i) {
        crc = kCrc8Table[static_cast<uint8_t>(crc ^ static_cast<uint8_t>(text[i]))];
    }
    return crc;
}

static_assert(crc8("123456789") == 0xBC, "CRC-8/DVB-S2 check value must be 0xBC");

}  // namespace crsf
