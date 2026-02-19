#ifdef USE_ESP32_FRAMEWORK_ARDUINO

#include "LinBusListener.h"
#include "esphome/core/log.h"
#include "esphome/components/uart/uart_component.h"

namespace esphome {
namespace truma_inetbox {

static const char *const TAG = "truma_inetbox.LinBusListener";

// Poll frequently; keep this tiny to avoid RX overrun
#define POLL_DELAY_TICKS  (pdMS_TO_TICKS(1))

void LinBusListener::setup_framework() {
  auto *uart = static_cast<uart::UARTComponent *>(this->parent_);
  if (!uart) {
    ESP_LOGE(TAG, "UART parent is null!");
    return;
  }

  // --- Tune the UART driver via ESPHome’s supported API ---
  uart->set_rx_buffer_size(8192);   // match YAML (or higher if needed)
  uart->set_rx_full_threshold(1);   // trigger handoff ASAP
  uart->set_rx_timeout(8);          // small timeout (bytes don’t sit in HW FIFO)
  // (These setters exist on UARTComponent in current ESPHome.)  // NOLINT
  // ----------------------------------------------------------

  ESP_LOGI(TAG, "LIN (Arduino): polling RX with tuned buffer/threshold/timeout");

  // Run our lightweight poller on the second core with a bit more priority
  xTaskCreatePinnedToCore(
      LinBusListener::eventTask_,
      "lin_event_task",
      4096,
      this,
      4,             // ↑ a touch higher priority than before
      &this->eventTaskHandle_,
      1);            // pin to APP CPU to avoid Wi‑Fi/Ethernet work

  if (this->eventTaskHandle_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create LIN event task!");
  }
}

void LinBusListener::eventTask_(void *arg) {
  auto *inst = reinterpret_cast<LinBusListener *>(arg);
  for (;;) {
    // 1) Drain the UART into the state machine
    inst->onReceive_();

    // 2) Process queued LIN messages WITHOUT blocking
    inst->process_lin_msg_queue(0);

    // 3) Yield briefly (keeps RX draining often)
    vTaskDelay(POLL_DELAY_TICKS);
  }
}

}  // namespace truma_inetbox
}  // namespace esphome

#undef POLL_DELAY_TICKS
#endif  // USE_ESP32_FRAMEWORK_ARDUINO
