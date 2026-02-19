#ifdef USE_ESP32_FRAMEWORK_ESP_IDF
#include "LinBusListener.h"
#include "esphome/core/log.h"
#include "soc/uart_reg.h"
#include <cinttypes>

#include "esphome/components/uart/uart_component_esp_idf.h"
#define ESPHOME_UART uart::IDFUARTComponent

namespace esphome {
namespace truma_inetbox {

static const char *const TAG = "truma_inetbox.LinBusListener";

#define QUEUE_WAIT_BLOCKING (TickType_t) portMAX_DELAY

void LinBusListener::setup_framework() {
  auto uartComp = static_cast<ESPHOME_UART *>(this->parent_);
  uint32_t target_baud = uartComp->get_baud_rate();

  // -------------------------------------------------------------------------
  // ESPHome removed get_hw_serial_number() and get_uart_event_queue() in 2026.x
  // Discover the UART port ourselves: scan all ports for one that has a driver
  // installed and whose baud rate matches within 5% (hardware dividers mean
  // uart_get_baudrate() may not return exactly the configured value).
  // -------------------------------------------------------------------------
  uart_port_t uart_num = (uart_port_t) -1;

  for (int i = 0; i < SOC_UART_NUM; i++) {
    if (uart_is_driver_installed((uart_port_t) i)) {
      uint32_t baud = 0;
      if (uart_get_baudrate((uart_port_t) i, &baud) == ESP_OK) {
        // Allow 5% tolerance to account for integer clock divider rounding
        uint32_t diff = (baud > target_baud) ? (baud - target_baud) : (target_baud - baud);
        if (diff * 20 <= target_baud) {  // diff <= 5% of target
          uart_num = (uart_port_t) i;
          ESP_LOGI(TAG, "Found UART%d (reported %" PRIu32 " baud, target %" PRIu32 " baud)",
                   (int) uart_num, baud, target_baud);
          break;
        }
      }
    }
  }

  if (uart_num == (uart_port_t) -1) {
    ESP_LOGE(TAG, "Could not find a UART port matching %" PRIu32 " baud - LIN bus will not work!", target_baud);
    return;
  }

  // -------------------------------------------------------------------------
  // ESPHome now installs the UART driver without an event queue (queue_size=0)
  // unless wake_loop_on_rx is enabled, and even then the queue is private.
  // We need UART_BREAK events for LIN framing, so we reinstall the driver with
  // our own event queue. uart_driver_delete removes only the FreeRTOS driver
  // layer; hardware registers (pins, baud rate) are not affected.
  // -------------------------------------------------------------------------
  esp_err_t err = uart_driver_delete(uart_num);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "uart_driver_delete(UART%d) failed: %s (continuing anyway)", (int) uart_num, esp_err_to_name(err));
  }

  err = uart_driver_install(uart_num,
                            2048,  // RX ring buffer - generous for LIN frames
                            0,     // TX ring buffer 0 = blocking TX
                            20,    // event queue depth
                            &this->uartEventQueue_,
                            0);    // interrupt flags
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "uart_driver_install(UART%d) failed: %s", (int) uart_num, esp_err_to_name(err));
    return;
  }

  // -------------------------------------------------------------------------
  // Re-apply uart_param_config after driver reinstall.
  // Although hardware registers survive uart_driver_delete, calling this
  // explicitly ensures correctness across all ESP-IDF versions.
  // LIN bus is always 9600 8N2 so we can safely hardcode parity/stop_bits.
  // -------------------------------------------------------------------------
  uart_config_t uart_config = {};
  uart_config.baud_rate     = (int) target_baud;
  uart_config.data_bits     = UART_DATA_8_BITS;
  uart_config.parity        = UART_PARITY_DISABLE;
  uart_config.stop_bits     = UART_STOP_BITS_2;
  uart_config.flow_ctrl     = UART_HW_FLOWCTRL_DISABLE;
  uart_config.source_clk    = UART_SCLK_DEFAULT;

  err = uart_param_config(uart_num, &uart_config);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "uart_param_config(UART%d) failed: %s", (int) uart_num, esp_err_to_name(err));
  }

  // -------------------------------------------------------------------------
  // Tweak FIFO interrupt thresholds.
  // Default RXFIFO_FULL threshold is 120 - way too high for short LIN frames.
  // Setting to 1 ensures onReceive_ fires after the very first byte arrives.
  // -------------------------------------------------------------------------
  uart_intr_config_t uart_intr = {};
  uart_intr.intr_enable_mask          = UART_RXFIFO_FULL_INT_ENA_M | UART_RXFIFO_TOUT_INT_ENA_M;
  uart_intr.rxfifo_full_thresh        = 1;
  uart_intr.rx_timeout_thresh         = 10;
  uart_intr.txfifo_empty_intr_thresh  = 10;

  err = uart_intr_config(uart_num, &uart_intr);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "uart_intr_config(UART%d) failed: %s", (int) uart_num, esp_err_to_name(err));
  }

  ESP_LOGI(TAG, "LIN bus driver installed on UART%d with event queue", (int) uart_num);

  // -------------------------------------------------------------------------
  // UART event task - receives hardware UART events and drives LIN parsing
  // -------------------------------------------------------------------------
  xTaskCreatePinnedToCore(LinBusListener::uartEventTask_,
                          "uart_event_task",
                          ARDUINO_SERIAL_EVENT_TASK_STACK_SIZE,
                          this,
                          24,
                          &this->uartEventTaskHandle_,
                          ARDUINO_SERIAL_EVENT_TASK_RUNNING_CORE);

  if (this->uartEventTaskHandle_ == NULL) {
    ESP_LOGE(TAG, "UART%d event task failed to create!", (int) uart_num);
  } else {
    ESP_LOGI(TAG, "UART%d event task started", (int) uart_num);
  }

  // -------------------------------------------------------------------------
  // LIN message processing task - handles decoded LIN frames
  // -------------------------------------------------------------------------
  xTaskCreatePinnedToCore(LinBusListener::eventTask_,
                          "lin_event_task",
                          ARDUINO_SERIAL_EVENT_TASK_STACK_SIZE,
                          this,
                          2,
                          &this->eventTaskHandle_,
                          ARDUINO_SERIAL_EVENT_TASK_RUNNING_CORE);

  if (this->eventTaskHandle_ == NULL) {
    ESP_LOGE(TAG, "LIN message task failed to create!");
  } else {
    ESP_LOGI(TAG, "LIN message task started");
  }
}

void LinBusListener::uartEventTask_(void *args) {
  LinBusListener *instance = (LinBusListener *) args;
  uart_event_t event;

  for (;;) {
    if (xQueueReceive(instance->uartEventQueue_, (void *) &event, QUEUE_WAIT_BLOCKING)) {
      if (event.type == UART_DATA && instance->available() > 0) {
        instance->onReceive_();
      } else if (event.type == UART_BREAK) {
        // A valid BREAK is followed immediately by SYNC+PID (which triggers
        // onReceive_ first), so only reset to BREAK if not waiting for SYNC.
        if (instance->current_state_ != READ_STATE_SYNC) {
          instance->current_state_ = READ_STATE_BREAK;
        }
      }
    }
  }
  vTaskDelete(NULL);
}

void LinBusListener::eventTask_(void *args) {
  LinBusListener *instance = (LinBusListener *) args;
  for (;;) {
    instance->process_lin_msg_queue(QUEUE_WAIT_BLOCKING);
  }
}

}  // namespace truma_inetbox
}  // namespace esphome

#undef QUEUE_WAIT_BLOCKING
#undef ESPHOME_UART

#endif  // USE_ESP32_FRAMEWORK_ESP_IDF
