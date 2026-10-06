# CRSF protocol reference (for ELRS receivers)

Sources: TBS CRSF spec (github.com/tbs-fpv/tbs-crsf-spec, `crsf.md`), ExpressLRS docs (expresslrs.org), and the Betaflight `rx/crsf.c` implementation.

## Serial link
- ELRS receiver ↔ controller: **420000 baud, 8N1, not inverted**, full duplex (TX and RX on separate wires), 3.3 V logic.
- The TBS spec lists 416666 baud for full duplex, but the ELRS default is 420000. Use 420000 unless the receiver was reconfigured.

## Frame layout
```
[sync/addr] [len] [type] [payload ... ] [crc8]
   1 byte   1 byte 1 byte  len-2 bytes   1 byte
```
- `sync`: **0xC8** (flight-controller address). ELRS receivers use 0xC8 for frames sent to the controller, and our telemetry frames also start with 0xC8.
- `len` = number of bytes **after** the len byte (type + payload + crc). Valid range is **2..62**. The whole frame is at most **64 bytes**.
- `crc8`: polynomial **0xD5** (CRC-8/DVB-S2), init 0x00, not reflected, no final XOR. It covers **type + payload** only, not sync or len.
  - Check value: `crc8("123456789") == 0xBC`.
- Multi-byte fields are **big-endian**, except the packed RC channels (see below).
- Frame types **≥ 0x28** use an extended header: `[sync][len][type][dest addr][origin addr][payload][crc]`.

### Addresses
| Addr | Device |
|---|---|
| 0x00 | broadcast |
| 0xC8 | flight controller (us) |
| 0xEA | radio handset |
| 0xEC | receiver |
| 0xEE | TX module |

### Frame types
| Type | Name | Dir | Payload |
|---|---|---|---|
| 0x02 | GPS | → radio | 15 B |
| 0x08 | Battery sensor | → radio | 8 B |
| 0x14 | Link statistics | receiver → us | 10 B |
| 0x16 | RC channels packed | receiver → us | 22 B |
| 0x1E | Attitude | → radio | 6 B |
| 0x21 | Flight mode | → radio | null-terminated string |
| 0x28+ | Device ping / info / parameters (extended header) | both | varies |

## 0x16 RC channels packed (22-byte payload)
- 16 channels × 11 bits = 176 bits, packed **LSB-first**: channel 0 takes bits 0–10 of the payload, channel 1 bits 11–21, and so on. (In Betaflight this is a little-endian bitfield struct.)
- Values run from 172 (min) through 992 (center) to 1811 (max), which is about 988 / 1500 / 2012 µs.
- `us = 1500 + (ticks - 992) * 5 / 8` and `ticks = 992 + (us - 1500) * 8 / 5`.
- With integer math (C++ truncates toward zero): 172 → 988, 992 → 1500, 1811 → 2011.

## 0x14 Link statistics (10-byte payload, in order)
| # | Field | Unit |
|---|---|---|
| 0 | uplink RSSI antenna 1 | −dBm (positive number, so 70 means −70 dBm) |
| 1 | uplink RSSI antenna 2 | −dBm |
| 2 | uplink link quality | % (0–100) |
| 3 | uplink SNR | dB, int8 |
| 4 | active antenna | 0/1 |
| 5 | RF profile / mode | enum (packet rate) |
| 6 | uplink TX power | enum |
| 7 | downlink RSSI | −dBm |
| 8 | downlink link quality | % |
| 9 | downlink SNR | dB, int8 |

## 0x08 Battery sensor (8-byte payload, big-endian)
| Field | Size | Unit as used by Betaflight / EdgeTX |
|---|---|---|
| voltage | uint16 | **0.1 V** (126 = 12.6 V) |
| current | uint16 | **0.1 A** |
| capacity used | uint24 | mAh |
| remaining | uint8 | % |

The current TBS spec text lists different voltage/current LSBs. Real radios display the 0.1-unit values above, so verify on the handset (EdgeTX shows `RxBt`).

## Failsafe behavior
- On link loss the ELRS receiver **stops sending 0x16 frames** (and telemetry). The controller has to detect that with a timeout.
- The TBS spec recommends 1 s before failsafe, which is meant for aircraft. For this car, use **250 ms**.

---

## Code (pure C++17, drop into `lib/CrsfProtocol/src/`)

### CRC8 (table computed at compile time, one lookup per byte)
```cpp
#include <array>
#include <cstddef>
#include <cstdint>

namespace crsf {

constexpr std::array<uint8_t, 256> makeCrc8Table(uint8_t poly) {
    std::array<uint8_t, 256> table{};
    for (int i = 0; i < 256; ++i) {
        uint8_t crc = static_cast<uint8_t>(i);
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ poly) : static_cast<uint8_t>(crc << 1);
        }
        table[i] = crc;
    }
    return table;
}

inline constexpr auto kCrc8Table = makeCrc8Table(0xD5);

constexpr uint8_t crc8(const uint8_t* data, size_t size) noexcept {
    uint8_t crc = 0;
    while (size--) {
        crc = kCrc8Table[crc ^ *data++];
    }
    return crc;
}

}  // namespace crsf
```

### Channel unpack and µs conversion
```cpp
namespace crsf {

constexpr size_t   kChannelCount     = 16;
constexpr size_t   kChannelsPayload  = 22;
constexpr uint16_t kTicksCenter      = 992;

inline void unpackChannels(const uint8_t* payload, uint16_t (&ticks)[kChannelCount]) noexcept {
    uint32_t bits     = 0;
    uint32_t bitCount = 0;
    for (uint16_t& ch : ticks) {
        while (bitCount < 11) {
            bits |= static_cast<uint32_t>(*payload++) << bitCount;
            bitCount += 8;
        }
        ch = static_cast<uint16_t>(bits & 0x7FF);
        bits >>= 11;
        bitCount -= 11;
    }
}

constexpr uint16_t ticksToUs(uint16_t ticks) noexcept {
    return static_cast<uint16_t>(1500 + (static_cast<int32_t>(ticks) - kTicksCenter) * 5 / 8);
}

}  // namespace crsf
```

### Parser (byte-fed, O(1) per byte, no heap)
```cpp
namespace crsf {

constexpr uint8_t kSyncByte     = 0xC8;
constexpr uint8_t kMaxFrameSize = 64;
enum class FrameType : uint8_t { Gps = 0x02, Battery = 0x08, LinkStats = 0x14, RcChannels = 0x16 };

// feed() returns true when a complete CRC-valid frame is ready. The caller must read
// type()/payload() BEFORE feeding the next byte (the buffer is reused).
class Parser {
public:
    bool feed(uint8_t byte) noexcept {
        if (pos_ == 0) {
            if (byte == kSyncByte) buf_[pos_++] = byte;
            return false;
        }
        if (pos_ == 1) {
            if (byte < 2 || byte > kMaxFrameSize - 2) { pos_ = 0; ++errors_; return false; }
            buf_[pos_++] = byte;
            return false;
        }
        buf_[pos_++] = byte;
        const size_t total = static_cast<size_t>(buf_[1]) + 2;
        if (pos_ < total) return false;
        pos_ = 0;
        if (crc8(&buf_[2], buf_[1] - 1) != buf_[total - 1]) { ++errors_; return false; }
        return true;
    }
    void           reset() noexcept { pos_ = 0; }
    uint8_t        type() const noexcept { return buf_[2]; }
    const uint8_t* payload() const noexcept { return &buf_[3]; }
    size_t         payloadSize() const noexcept { return static_cast<size_t>(buf_[1]) - 2; }
    uint32_t       errors() const noexcept { return errors_; }

private:
    uint8_t  buf_[kMaxFrameSize]{};
    size_t   pos_    = 0;
    uint32_t errors_ = 0;
};

}  // namespace crsf
```
Possible improvement: on a CRC error, rescan the buffered bytes for the next 0xC8 instead of dropping them. ELRS sends frames in bursts with gaps, so the simple version resyncs within one frame anyway.

### Battery telemetry frame builder
```cpp
// namespace crsf. Returns the frame size (12). Units: 0.1 V, 0.1 A, mAh, %.
inline size_t buildBatteryFrame(uint8_t (&f)[12], uint16_t deciVolts, uint16_t deciAmps,
                                uint32_t usedMah, uint8_t remainingPct) noexcept {
    f[0]  = kSyncByte;
    f[1]  = 10;                                  // type + 8 payload + crc
    f[2]  = static_cast<uint8_t>(FrameType::Battery);
    f[3]  = static_cast<uint8_t>(deciVolts >> 8);
    f[4]  = static_cast<uint8_t>(deciVolts);
    f[5]  = static_cast<uint8_t>(deciAmps >> 8);
    f[6]  = static_cast<uint8_t>(deciAmps);
    f[7]  = static_cast<uint8_t>(usedMah >> 16);
    f[8]  = static_cast<uint8_t>(usedMah >> 8);
    f[9]  = static_cast<uint8_t>(usedMah);
    f[10] = remainingPct;
    f[11] = crc8(&f[2], 9);
    return sizeof(f);
}
```

---

## ESP-IDF UART setup (in `CrsfReceiver::begin()`)
```cpp
#include <driver/uart.h>

uart_config_t cfg{};
cfg.baud_rate  = 420000;
cfg.data_bits  = UART_DATA_8_BITS;
cfg.parity     = UART_PARITY_DISABLE;
cfg.stop_bits  = UART_STOP_BITS_1;
cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
cfg.source_clk = UART_SCLK_APB;

// RX ring buffer 1024 B, no TX ring buffer (frames ≤ 64 B fit the 128 B HW FIFO), 16-event queue.
// In real code check every esp_err_t below and return it (with log_e) on failure.
uart_driver_install(UART_NUM_2, 1024, 0, 16, &eventQueue_, 0);
uart_param_config(UART_NUM_2, &cfg);
uart_set_pin(UART_NUM_2, /*tx*/ GPIO_NUM_17, /*rx*/ GPIO_NUM_13, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
uart_set_rx_timeout(UART_NUM_2, 3);   // IRQ after ~3 symbol times of silence ≈ end of a burst
```

### rx task loop
```cpp
uart_event_t event;
uint8_t      chunk[128];
for (;;) {
    if (xQueueReceive(eventQueue_, &event, portMAX_DELAY) != pdTRUE) continue;
    switch (event.type) {
    case UART_DATA: {
        const int n = uart_read_bytes(UART_NUM_2, chunk, event.size < sizeof(chunk) ? event.size : sizeof(chunk), 0);
        for (int i = 0; i < n; ++i) {
            if (parser_.feed(chunk[i])) handleFrame();   // publish mailbox + xTaskNotifyGive(consumer)
        }
        break;
    }
    case UART_FIFO_OVF:
    case UART_BUFFER_FULL:
        uart_flush_input(UART_NUM_2);
        xQueueReset(eventQueue_);
        parser_.reset();
        ++overflows_;
        break;
    default:
        break;   // frame/parity errors: the CRC check rejects the garbage
    }
}
```

## Native unit-test vectors (Unity, `test/test_crsf/`)
- `crsf::crc8((const uint8_t*)"123456789", 9) == 0xBC`
- Pack 16 known tick values (write a small `packChannels` helper in the test), then unpack and compare. Include the 172, 992 and 1811 edge values.
- `ticksToUs(172) == 988`, `ticksToUs(992) == 1500`, `ticksToUs(1811) == 2011`.
- Feed a valid 0x16 frame: exactly one `feed()` returns true, on the last byte.
- The same frame with one payload bit flipped: no frame, and `errors()` increments.
- Garbage bytes (including a fake 0xC8 followed by a bad length), then a valid frame: the valid frame is still parsed.
- A frame split across two feed loops: parsed once.
- Two frames back-to-back in one buffer: both parsed.
