#if defined(USE_ESP32) && !defined(USE_ESP32_FRAMEWORK_ARDUINO)
#include "LinBusListener.h"
#include "esphome/core/log.h"
#include <cinttypes>

#include "esphome/components/uart/uart_component_esp_idf.h"
#define ESPHOME_UART uart::IDFUARTComponent

namespace esphome {
namespace truma_inetbox {

static const char *const TAG = "truma_inetbox.LinBusListener";

#define QUEUE_WAIT_BLOCKING (TickType_t) portMAX_DELAY

void LinBusListener::setup_framework() {
  auto uartComp = static_cast<ESPHOME_UART *>(this->parent_);

  // -------------------------------------------------------------------------
  // ESPHome 2026.x: get_hw_serial_number() is public.
  // get_uart_event_queue() is available when wake_loop_on_rx: true is set in
  // YAML, which defines USE_UART_WAKE_LOOP_ON_RX and installs the driver with
  // an event queue. We use that queue directly — no driver reinstall needed,
  // which preserves ESPHome's TX path for LIN responses to the CP Plus.
  // -------------------------------------------------------------------------

#ifndef USE_UART_WAKE_LOOP_ON_RX
  this->setup_framework_status_ = "ERROR: wake_loop_on_rx not enabled - add 'wake_loop_on_rx: true' to uart: in YAML";
  ESP_LOGE(TAG, "wake_loop_on_rx is not enabled! Add 'wake_loop_on_rx: true' under uart: in your YAML config.");
  return;
#else
  uart_port_t uart_num = (uart_port_t) uartComp->get_hw_serial_number();
  this->uartEventQueue_ = *uartComp->get_uart_event_queue();

  ESP_LOGI(TAG, "Using UART%d with existing event queue", (int) uart_num);
  this->setup_framework_status_ = "OK: driver on UART" + std::to_string((int) uart_num);

  // -------------------------------------------------------------------------
  // UART event task — receives hardware UART events and drives LIN parsing.
  // Pinned to core 1 (app core) for deterministic LIN response timing.
  // -------------------------------------------------------------------------
  xTaskCreatePinnedToCore(LinBusListener::uartEventTask_,
                          "uart_event_task",
                          4096,
                          this,
                          24,
                          &this->uartEventTaskHandle_,
                          1);

  if (this->uartEventTaskHandle_ == NULL) {
    ESP_LOGE(TAG, "UART%d event task failed to create!", (int) uart_num);
  } else {
    ESP_LOGI(TAG, "UART%d event task started", (int) uart_num);
  }

  // -------------------------------------------------------------------------
  // LIN message processing task — handles decoded LIN frames.
  // -------------------------------------------------------------------------
  xTaskCreatePinnedToCore(LinBusListener::eventTask_,
                          "lin_event_task",
                          4096,
                          this,
                          2,
                          &this->eventTaskHandle_,
                          1);

  if (this->eventTaskHandle_ == NULL) {
    ESP_LOGE(TAG, "LIN message task failed to create!");
  } else {
    ESP_LOGI(TAG, "LIN message task started");
  }
#endif  // USE_UART_WAKE_LOOP_ON_RX
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

#endif  // USE_ESP32 && !USE_ESP32_FRAMEWORK_ARDUINO
