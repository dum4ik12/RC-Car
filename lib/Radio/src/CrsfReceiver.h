#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include <driver/gpio.h>
#include <driver/uart.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <crsf/Frame.h>
#include <crsf/LinkStats.h>
#include <crsf/Parser.h>

/**
 * @brief ExpressLRS / CRSF receiver on an ESP32 UART, event-driven.
 *
 * begin() installs the IDF UART driver with an event queue and starts the crsfRx
 * task. The task sleeps until the UART reports data (the RX-timeout interrupt
 * fires after a few idle symbols, i.e. once per frame burst), parses the bytes
 * and, for every CRC-valid RC frame, converts the channels to µs, stamps the tick
 * time, attaches the newest link statistics and publishes the result into a
 * one-slot mailbox. The consumer task (if set) gets a task notification.
 * receive() hands each frame out once; a frame nobody fetched is overwritten.
 *
 * Link loss: an ELRS receiver stops sending RC frames (default "No Pulses"
 * failsafe mode) and link statistics. The consumer detects that from the frame
 * timestamps; this class only reports what arrived and when.
 *
 * Thread-safety: receive(), counters() and setConsumer() may be called from any
 * task. sendFrame() is for a single telemetry writer task.
 */
class CrsfReceiver {
public:
    struct Config {
        gpio_num_t  rxPin;                           // ESP32 RX <- receiver TX
        gpio_num_t  txPin            = GPIO_NUM_NC;  // ESP32 TX -> receiver RX (telemetry); NC = receive-only
        uart_port_t uart             = UART_NUM_2;
        uint32_t    baudRate         = crsf::kDefaultBaudRate;
        uint32_t    taskStackBytes   = 4096;
        UBaseType_t taskPriority     = 10;
        BaseType_t  taskCore         = 1;
        uint16_t    rxBufferBytes    = 1024;  // driver ring buffer; must exceed the 128 B hardware FIFO
        uint8_t     eventQueueDepth  = 16;
        uint8_t     rxTimeoutSymbols = 3;     // idle symbols before the RX-timeout interrupt
    };

    /** The newest RC frame plus the newest link statistics seen before it. Times are FreeRTOS tick-ms. */
    struct RcFrame {
        uint32_t        receivedMs;                       // when the CRC-valid 0x16 frame completed
        uint16_t        channelsUs[crsf::kChannelCount];  // ~988..2012 µs, 1500 = centre
        uint32_t        linkStatsMs;                      // when linkStats arrived (valid if hasLinkStats)
        bool            hasLinkStats;
        crsf::LinkStats linkStats;
    };

    struct Counters {
        uint32_t rcFrames;
        uint32_t linkStatsFrames;
        uint32_t otherFrames;
        uint32_t parserErrors;  // bad length byte or CRC
        uint32_t badLength;     // known type with the wrong payload size
        uint32_t overflows;     // UART FIFO or ring-buffer overflow
        uint32_t lineErrors;    // framing / parity errors
    };

    explicit CrsfReceiver(const Config& config) noexcept;
    ~CrsfReceiver();

    CrsfReceiver(const CrsfReceiver&)            = delete;
    CrsfReceiver& operator=(const CrsfReceiver&) = delete;

    /** Create the mailbox, install the UART driver and start the rx task. */
    [[nodiscard]] esp_err_t begin();

    /** Task to notify (xTaskNotifyGive) after every published RC frame. nullptr disables. */
    void setConsumer(TaskHandle_t task) noexcept { consumer_.store(task, std::memory_order_release); }

    /** Fetch the newest frame if one arrived since the last call. Never blocks. */
    bool receive(RcFrame& out) noexcept;

    Counters counters() const noexcept;

    /** Telemetry hook: write one complete CRSF frame to the receiver. Single writer task. */
    esp_err_t sendFrame(const uint8_t* frame, size_t size);

    const Config& config() const noexcept { return config_; }
    bool          isRunning() const noexcept { return started_; }

private:
    static constexpr size_t   kChunkSize    = 128;  // == UART hardware FIFO size
    static constexpr uint32_t kMinTaskStack = 2048;

    [[noreturn]] static void taskEntry(void* arg);
    [[noreturn]] void        run();
    void                     feed(const uint8_t* data, size_t size);
    void                     handleFrame();
    void                     teardown();

    const Config config_;
    crsf::Parser parser_;
    RcFrame      frame_{};  // assembled by the rx task only

    QueueHandle_t events_  = nullptr;  // owned by the UART driver
    QueueHandle_t mailbox_ = nullptr;
    TaskHandle_t  task_    = nullptr;
    bool          started_ = false;

    std::atomic<TaskHandle_t> consumer_{nullptr};
    std::atomic<uint32_t>     rcFrames_{0};
    std::atomic<uint32_t>     linkStatsFrames_{0};
    std::atomic<uint32_t>     otherFrames_{0};
    std::atomic<uint32_t>     parserErrors_{0};
    std::atomic<uint32_t>     badLength_{0};
    std::atomic<uint32_t>     overflows_{0};
    std::atomic<uint32_t>     lineErrors_{0};
};
