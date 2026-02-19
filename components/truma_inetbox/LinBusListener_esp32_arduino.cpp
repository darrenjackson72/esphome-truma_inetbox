#ifdef USE_ESP32_FRAMEWORK_ARDUINO
#include "LinBusListener.h"
#include "esphome/core/log.h"
#include "driver/uart.h"
#include "soc/uart_struct.h"
#include "soc/uart_reg.h"

// --- Updated for ESPHome 2025+ unified UART API ---
#include "esphome/components/uart/uart_component.h"
using esphome::uart::UARTComponent;
// ---------------------------------------------------

namespace esphome {
namespace truma_inetbox {

static const char *const TAG = "truma_inetbox.LinBusListener";

#define QUEUE_WAIT_BLOCKING (TickType_t) portMAX_DELAY

void LinBusListener::setup_framework() {
  // parent_ is your UART component, cast it to the new unified UART type
  auto uartComp = static_cast<UARTComponent *>(this->parent_);

  // Modern ESPHome UART API:
  int uart_num     = uartComp->get_hw_serial_number();
  auto *hw_serial  = uartComp->get_hw_serial();

  // Configure UART interrupt handling for low-latency LIN reception
  uart_intr_config_t uart_intr;
  uart_intr.intr_enable_mask =
      UART_RXFIFO_FULL_INT_ENA_M | UART_RXFIFO_TOUT_INT_ENA_M;
  uart_intr.rxfifo_full_thresh       = 1;
  uart_intr.rx_timeout_thresh        = 10;
  uart_intr.txfifo_empty_intr_thresh = 10;
  uart_intr_config(uart_num, &uart_intr);

  // Hook UART receive callbacks (Arduino HAL)
  hw_serial->onReceive(this { this->onReceive_(); }, false);
  hw_serial->onReceiveError([this](hardwareSerial_error_t val) {
    this->clear_uart_buffer_();
    if (val == UART_BREAK_ERROR) {
      if (this->current_state_ != READ_STATE_SYNC) {
        this->current_state_ = READ_STATE_BREAK;
      }
      return;
    }
  });

  // Create LIN message processing task
  xTaskCreatePinnedToCore(LinBusListener::eventTask_,
                          "lin_event_task",
                          4096,
                          this,
                          2,
                          &this->eventTaskHandle_,
                          0);

  if (this->eventTaskHandle_ == NULL) {
    ESP_LOGE(TAG, " -- LIN message Task not created!");
  }
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

#endif  // USE_ESP32_FRAMEWORK_ARDUINO
