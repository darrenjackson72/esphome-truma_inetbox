#ifdef USE_ESP32_FRAMEWORK_ARDUINO

#include "LinBusListener.h"
#include "esphome/core/log.h"
#include "esphome/components/uart/uart_component.h"

namespace esphome {
namespace truma_inetbox {

static const char *const TAG = "truma_inetbox.LinBusListener";

#define QUEUE_WAIT_BLOCKING (TickType_t) portMAX_DELAY

/*
 * Modern ESPHome UART backend for Arduino:
 * - No hardware serial callbacks
 * - No direct ESP-IDF uart_intr_config()
 * - Polling-based LIN frame processing
 * - Compatible with ESPHome 2026 unified UART API
 */

void LinBusListener::setup_framework() {
  // No Arduino-specific setup needed.
  // The unified UARTComponent handles everything safely.
  ESP_LOGI(TAG, "Using polling-based LIN backend (Arduino safe)");
  
  // Create the event task that polls the UART
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
    // Poll UART for new LIN data
    inst->onReceive_();

    // Process queued LIN messages
    inst->process_lin_msg_queue(QUEUE_WAIT_BLOCKING);

    // Yield briefly to avoid starving other tasks
    vTaskDelay(1);
  }
}

}  // namespace truma_inetbox
}  // namespace esphome

#undef QUEUE_WAIT_BLOCKING

#endif  // USE_ESP32_FRAMEWORK_ARDUINO
