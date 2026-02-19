#ifdef USE_ESP32_FRAMEWORK_ARDUINO

#include "LinBusListener.h"
#include "esphome/core/log.h"

// Modern ESPHome UART include
#include "esphome/components/uart/uart_component.h"
using esphome::uart::UARTComponent;

namespace esphome {
namespace truma_inetbox {

static const char *const TAG = "truma_inetbox.LinBusListener";

#define QUEUE_WAIT_BLOCKING (TickType_t) portMAX_DELAY

/*
 * Modern Arduino HAL access:
 *
 * UARTComponent -> exposes:
 *   - write_array()
 *   - flush()
 *   - read_byte()
 *   - available()
 *   - get_uart()  -> returns HardwareSerial* for Arduino
 */

void LinBusListener::setup_framework() {
  auto *uart = static_cast<UARTComponent *>(this->parent_);

  // Access Arduino's HardwareSerial
  HardwareSerial *hw = uart->get_uart();

  if (hw == nullptr) {
    ESP_LOGE(TAG, "UARTComponent returned null HardwareSerial pointer!");
    return;
  }

  // -------------------------
  // Register Arduino callbacks
  // -------------------------

  // On receive callback
  hw->onReceive(this {
    this->onReceive_();
  }, false);

  // UART error callback
  hw->onReceiveError([this](hardwareSerial_error_t err) {
    this->clear_uart_buffer_();

    if (err == UART_BREAK_ERROR) {
      if (this->current_state_ != READ_STATE_SYNC) {
        this->current_state_ = READ_STATE_BREAK;
      }
    }
  });

  // -------------------------
  // Create event-processing task
  // -------------------------

  xTaskCreatePinnedToCore(
      LinBusListener::eventTask_,
      "lin_event_task",
      4096,
      this,
      2,
      &this->eventTaskHandle_,
      0);

  if (this->eventTaskHandle_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create LIN event task!");
  }
}

void LinBusListener::eventTask_(void *arg) {
  auto *inst = reinterpret_cast<LinBusListener *>(arg);
  for (;;) {
    inst->process_lin_msg_queue(QUEUE_WAIT_BLOCKING);
  }
}

}  // namespace truma_inetbox
}  // namespace esphome

#undef QUEUE_WAIT_BLOCKING

#endif  // USE_ESP32_FRAMEWORK_ARDUINO
