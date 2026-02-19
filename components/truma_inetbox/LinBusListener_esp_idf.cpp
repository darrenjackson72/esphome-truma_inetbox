#ifdef USE_ESP32_FRAMEWORK_ESP_IDF
#include "LinBusListener.h"
#include "esphome/core/log.h"
#include "soc/uart_reg.h"

#include "esphome/components/uart/uart_component_esp_idf.h"
#define ESPHOME_UART uart::IDFUARTComponent

namespace esphome {
namespace truma_inetbox {

static const char *const TAG = "truma_inetbox.LinBusListener";

#define QUEUE_WAIT_BLOCKING (TickType_t) portMAX_DELAY

void LinBusListener::setup_framework() {
  auto uartComp = static_cast<ESPHOME_UART *>(this->parent_);

  // -------------------------------------------------------------------------
  // ESPHome removed get_hw_serial_number() and get_uart_event_queue() from
  // IDFUARTComponent. We discover the UART port ourselves by iterating all
  // available ports and finding the one that:
  //   a) has a driver installed, AND
  //   b) has a baud rate matching the ESPHome UART component
  // -------------------------------------------------------------------------
  uart_port_t uart_num = (uart_port_t) -1;
  uint32_t target_baud = uartComp->get_baud_rate();

  for (int i = 0; i < SOC_UART_NUM; i++) {
    if (uart_is_driver_installed((uart_port_t) i)) {
      uint32_t baud = 0;
      if (uart_get_baudrate((uart_port_t) i, &baud) == ESP_OK && baud == target_baud) {
        uart_num = (uart_port_t) i;
        break;
      }
    }
  }

  if (uart_num == (uart_port_t) -1) {
    ESP_LOGE(TAG, "Could not find UART port for baud rate %" PRIu32 "!", target_baud);
    return;
  }

  ESP_LOGD(TAG, "Found UART%d for LIN bus", (int) uart_num);

  // -------------------------------------------------------------------------
  // ESPHome installs the UART driver without an event queue (queue size = 0)
  // unless wake_loop_on_rx is enabled - and even then the queue is private.
  // We need UART_BREAK events for LIN bus framing, so we must reinstall the
  // driver ourselves with our own event queue.
  //
  // uart_driver_delete leaves all pin mux / UART register config intact, so
  // a reinstall just replaces the FreeRTOS driver layer - no data is lost at
  // this point in setup() before communication has started.
  // -------------------------------------------------------------------------
  size_t rx_buf_size = 2048;  // generous ring buffer for LIN frames

  esp_err_t err = uart_driver_delete(uart_num);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "uart_driver_delete failed: %s", esp_err_to_name(err));
    // Not fatal - may not have been installed yet, continue.
  }

  err = uart_driver_install(uart_num,
                            rx_buf_size,          // RX ring buffer
                            0,                    // TX ring buffer (blocking)
                            20,                   // event queue depth
                            &this->uartEventQueue_,
                            0);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
    return;
  }

  // Tweak FIFO interrupt thresholds so bytes are available immediately
  // (default RXFIFO_FULL threshold is 120 which is far too high for LIN).
  uart_intr_config_t uart_intr;
  uart_intr.intr_enable_mask = UART_RXFIFO_FULL_INT_ENA_M | UART_RXFIFO_TOUT_INT_ENA_M;
  uart_intr.rxfifo_full_thresh = 1;   // interrupt after every byte
  uart_intr.rx_timeout_thresh = 10;   // 10-symbol idle timeout
  uart_intr.txfifo_empty_intr_thresh = 10;
  uart_intr_config(uart_num, &uart_intr);

  // -------------------------------------------------------------------------
  // Spawn UART event task (reads hardware UART queue, triggers LIN parsing)
  // -------------------------------------------------------------------------
  xTaskCreatePinnedToCore(LinBusListener::uartEventTask_,
                          "uart_event_task",
                          ARDUINO_SERIAL_EVENT_TASK_STACK_SIZE,
                          this,
                          24,
                          &this->uartEventTaskHandle_,
                          ARDUINO_SERIAL_EVENT_TASK_RUNNING_CORE);

  if (this->uartEventTaskHandle_ == NULL) {
    ESP_LOGE(TAG, " -- UART%d Event Task not created!", (int) uart_num);
  }

  // -------------------------------------------------------------------------
  // Spawn LIN message event task (processes decoded LIN frames)
  // -------------------------------------------------------------------------
  xTaskCreatePinnedToCore(LinBusListener::eventTask_,
                          "lin_event_task",
                          ARDUINO_SERIAL_EVENT_TASK_STACK_SIZE,
                          this,
                          2,
                          &this->eventTaskHandle_,
                          ARDUINO_SERIAL_EVENT_TASK_RUNNING_CORE);

  if (this->eventTaskHandle_ == NULL) {
    ESP_LOGE(TAG, " -- LIN message Task not created!");
  }
}

void LinBusListener::uartEventTask_(void *args) {
  LinBusListener *instance = (LinBusListener *) args;
  uart_event_t event;

  for (;;) {
    // Block until a UART driver event arrives on OUR event queue
    // (uartEventQueue_ is now owned by LinBusListener, not ESPHome)
    if (xQueueReceive(instance->uartEventQueue_, (void *) &event, QUEUE_WAIT_BLOCKING)) {
      if (event.type == UART_DATA && instance->available() > 0) {
        instance->onReceive_();
      } else if (event.type == UART_BREAK) {
        // A valid BREAK is followed by SYNC+PID which triggers onReceive_ first,
        // so only move to BREAK state if we're not already expecting SYNC.
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
