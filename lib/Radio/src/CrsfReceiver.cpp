#include "CrsfReceiver.h"

#include <esp32-hal-log.h>

#include <crsf/Channels.h>
#include <crsf/SelfTest.h>

static_assert(crsf::selftest::run(), "CRSF parser self-test failed");
static_assert(configTICK_RATE_HZ == 1000, "RcFrame timestamps assume a 1 kHz FreeRTOS tick");

namespace {

esp_err_t logIfError(esp_err_t err, const char* what) {
    if (err != ESP_OK) {
        log_e("CRSF: %s failed: %s", what, esp_err_to_name(err));
    }
    return err;
}

}  // namespace

CrsfReceiver::CrsfReceiver(const Config& config) noexcept : config_(config) {}

CrsfReceiver::~CrsfReceiver() {
    teardown();
}

esp_err_t CrsfReceiver::begin() {
    if (started_) {
        return ESP_ERR_INVALID_STATE;
    }
    if (config_.rxPin == GPIO_NUM_NC || config_.taskStackBytes < kMinTaskStack || config_.eventQueueDepth == 0 ||
        config_.rxBufferBytes <= kChunkSize) {
        log_e("CRSF: invalid config (rx GPIO%d, stack %u B, queue %u, buffer %u B)", static_cast<int>(config_.rxPin),
              static_cast<unsigned>(config_.taskStackBytes), static_cast<unsigned>(config_.eventQueueDepth),
              static_cast<unsigned>(config_.rxBufferBytes));
        return ESP_ERR_INVALID_ARG;
    }

    mailbox_ = xQueueCreate(1, sizeof(RcFrame));
    if (mailbox_ == nullptr) {
        log_e("CRSF: mailbox allocation failed");
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = logIfError(uart_driver_install(config_.uart, config_.rxBufferBytes, 0, config_.eventQueueDepth,
                                                   &events_, 0),
                               "uart_driver_install");
    if (err != ESP_OK) {
        teardown();
        return err;
    }

    uart_config_t uart{};
    uart.baud_rate           = static_cast<int>(config_.baudRate);
    uart.data_bits           = UART_DATA_8_BITS;
    uart.parity              = UART_PARITY_DISABLE;
    uart.stop_bits           = UART_STOP_BITS_1;
    uart.flow_ctrl           = UART_HW_FLOWCTRL_DISABLE;
    uart.rx_flow_ctrl_thresh = 0;
    uart.source_clk          = UART_SCLK_APB;
    err                      = logIfError(uart_param_config(config_.uart, &uart), "uart_param_config");
    if (err != ESP_OK) {
        teardown();
        return err;
    }

    const int txPin = config_.txPin == GPIO_NUM_NC ? UART_PIN_NO_CHANGE : static_cast<int>(config_.txPin);
    err = logIfError(uart_set_pin(config_.uart, txPin, config_.rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
                     "uart_set_pin");
    if (err != ESP_OK) {
        teardown();
        return err;
    }

    // After uart_param_config: the driver scales the timeout by the configured symbol length.
    err = logIfError(uart_set_rx_timeout(config_.uart, config_.rxTimeoutSymbols), "uart_set_rx_timeout");
    if (err != ESP_OK) {
        teardown();
        return err;
    }

    // Drop whatever the FIFO collected while the baud rate was still wrong.
    err = logIfError(uart_flush_input(config_.uart), "uart_flush_input");
    if (err != ESP_OK) {
        teardown();
        return err;
    }

    if (xTaskCreatePinnedToCore(taskEntry, "crsfRx", config_.taskStackBytes, this, config_.taskPriority, &task_,
                                config_.taskCore) != pdPASS) {
        log_e("CRSF: rx task creation failed");
        task_ = nullptr;
        teardown();
        return ESP_ERR_NO_MEM;
    }
    started_ = true;

    log_i("CRSF on UART%d: RX GPIO%d, TX %s%d, %u baud, task crsfRx prio %u on core %d",
          static_cast<int>(config_.uart), static_cast<int>(config_.rxPin),
          config_.txPin == GPIO_NUM_NC ? "unused " : "GPIO", static_cast<int>(config_.txPin),
          static_cast<unsigned>(config_.baudRate), static_cast<unsigned>(config_.taskPriority),
          static_cast<int>(config_.taskCore));
    return ESP_OK;
}

void CrsfReceiver::teardown() {
    if (task_ != nullptr) {
        vTaskDelete(task_);
        task_ = nullptr;
    }
    if (uart_is_driver_installed(config_.uart)) {
        uart_driver_delete(config_.uart);  // also frees the event queue
    }
    events_ = nullptr;
    if (mailbox_ != nullptr) {
        vQueueDelete(mailbox_);
        mailbox_ = nullptr;
    }
    started_ = false;
}

void CrsfReceiver::taskEntry(void* arg) {
    static_cast<CrsfReceiver*>(arg)->run();
}

void CrsfReceiver::run() {
    uart_event_t event;
    uint8_t      chunk[kChunkSize];
    for (;;) {
        if (xQueueReceive(events_, &event, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        switch (event.type) {
            case UART_DATA: {
                // Drain everything that is buffered, not just event.size: the ISR drops events
                // silently when the queue is full, and their bytes are still in the ring buffer.
                int n = 0;
                while ((n = uart_read_bytes(config_.uart, chunk, sizeof(chunk), 0)) > 0) {
                    feed(chunk, static_cast<size_t>(n));
                }
                break;
            }
            case UART_FIFO_OVF:
            case UART_BUFFER_FULL:
                // Lost bytes: start over from a clean slate. The next frame resyncs the parser.
                uart_flush_input(config_.uart);
                xQueueReset(events_);
                parser_.reset();
                overflows_.fetch_add(1, std::memory_order_relaxed);
                break;
            case UART_FRAME_ERR:
            case UART_PARITY_ERR:
                lineErrors_.fetch_add(1, std::memory_order_relaxed);  // the CRC rejects the damaged frame
                break;
            default:
                break;
        }
    }
}

void CrsfReceiver::feed(const uint8_t* data, size_t size) {
    for (size_t i = 0; i < size; ++i) {
        if (parser_.feed(data[i])) {
            handleFrame();
        }
    }
    parserErrors_.store(parser_.errors(), std::memory_order_relaxed);
}

void CrsfReceiver::handleFrame() {
    const uint32_t nowMs = xTaskGetTickCount();

    switch (static_cast<crsf::FrameType>(parser_.type())) {
        case crsf::FrameType::RcChannels: {
            if (parser_.payloadSize() != crsf::kChannelsPayload) {
                badLength_.fetch_add(1, std::memory_order_relaxed);
                return;
            }
            crsf::Channels ticks{};
            crsf::unpackChannels(parser_.payload(), ticks);
            for (size_t i = 0; i < crsf::kChannelCount; ++i) {
                frame_.channelsUs[i] = crsf::ticksToUs(ticks[i]);
            }
            frame_.receivedMs = nowMs;
            // frame_.linkStats / linkStatsMs / hasLinkStats already hold the newest statistics.
            rcFrames_.fetch_add(1, std::memory_order_relaxed);

            xQueueOverwrite(mailbox_, &frame_);
            if (const TaskHandle_t consumer = consumer_.load(std::memory_order_acquire)) {
                xTaskNotifyGive(consumer);
            }
            return;
        }
        case crsf::FrameType::LinkStats:
            if (parser_.payloadSize() < crsf::kLinkStatsPayload) {
                badLength_.fetch_add(1, std::memory_order_relaxed);
                return;
            }
            frame_.linkStats    = crsf::decodeLinkStats(parser_.payload());
            frame_.linkStatsMs  = nowMs;
            frame_.hasLinkStats = true;
            linkStatsFrames_.fetch_add(1, std::memory_order_relaxed);
            return;
        default:
            otherFrames_.fetch_add(1, std::memory_order_relaxed);
            return;
    }
}

bool CrsfReceiver::receive(RcFrame& out) noexcept {
    return mailbox_ != nullptr && xQueueReceive(mailbox_, &out, 0) == pdTRUE;
}

CrsfReceiver::Counters CrsfReceiver::counters() const noexcept {
    Counters counters{};
    counters.rcFrames        = rcFrames_.load(std::memory_order_relaxed);
    counters.linkStatsFrames = linkStatsFrames_.load(std::memory_order_relaxed);
    counters.otherFrames     = otherFrames_.load(std::memory_order_relaxed);
    counters.parserErrors    = parserErrors_.load(std::memory_order_relaxed);
    counters.badLength       = badLength_.load(std::memory_order_relaxed);
    counters.overflows       = overflows_.load(std::memory_order_relaxed);
    counters.lineErrors      = lineErrors_.load(std::memory_order_relaxed);
    return counters;
}

esp_err_t CrsfReceiver::sendFrame(const uint8_t* frame, size_t size) {
    if (!started_) {
        return ESP_ERR_INVALID_STATE;
    }
    if (config_.txPin == GPIO_NUM_NC) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (frame == nullptr || size < crsf::kHeaderSize + crsf::kMinLength || size > crsf::kMaxFrameSize) {
        return ESP_ERR_INVALID_ARG;
    }
    // No TX ring buffer: the bytes go straight into the 128 B FIFO, which any CRSF frame fits.
    const int written = uart_write_bytes(config_.uart, frame, size);
    return written == static_cast<int>(size) ? ESP_OK : ESP_FAIL;
}
