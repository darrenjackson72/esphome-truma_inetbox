#pragma once

#include "esphome/core/component.h"
#include "TrumaiNetBoxApp.h"

namespace esphome {
namespace truma_inetbox {

template<typename... Ts> class HeaterRoomTempAction : public Action<Ts...>, public Parented<TrumaiNetBoxApp> {
 public:
  TEMPLATABLE_VALUE(uint8_t, temperature)
  TEMPLATABLE_VALUE(uint8_t, heating_mode)   // was HeatingMode

  void play(Ts... x) override {
    this->parent_->get_heater()->action_heater_room(
        this->temperature_.value_or(x..., 0),
        static_cast<HeatingMode>(this->heating_mode_.value_or(x..., (uint8_t) HeatingMode::HEATING_MODE_OFF)));
  }
};

template<typename... Ts> class HeaterWaterTempAction : public Action<Ts...>, public Parented<TrumaiNetBoxApp> {
 public:
  TEMPLATABLE_VALUE(uint8_t, temperature)

  void play(Ts... x) override {
    this->parent_->get_heater()->action_heater_water(this->temperature_.value_or(x..., 0));
  }
};

template<typename... Ts> class HeaterWaterTempEnumAction : public Action<Ts...>, public Parented<TrumaiNetBoxApp> {
 public:
  TEMPLATABLE_VALUE(uint16_t, temperature)   // was TargetTemp

  void play(Ts... x) override {
    this->parent_->get_heater()->action_heater_water(
        static_cast<TargetTemp>(this->temperature_.value_or(x..., (uint16_t) TargetTemp::TARGET_TEMP_OFF)));
  }
};

template<typename... Ts> class HeaterElecPowerLevelAction : public Action<Ts...>, public Parented<TrumaiNetBoxApp> {
 public:
  TEMPLATABLE_VALUE(uint16_t, watt)

  void play(Ts... x) override {
    this->parent_->get_heater()->action_heater_electric_power_level(this->watt_.value_or(x..., 0));
  }
};

template<typename... Ts> class HeaterEnergyMixAction : public Action<Ts...>, public Parented<TrumaiNetBoxApp> {
 public:
  TEMPLATABLE_VALUE(uint8_t, energy_mix)     // was EnergyMix
  TEMPLATABLE_VALUE(uint16_t, watt)          // was ElectricPowerLevel

  void play(Ts... x) override {
    this->parent_->get_heater()->action_heater_energy_mix(
        static_cast<EnergyMix>(this->energy_mix_.value_or(x..., (uint8_t) EnergyMix::ENERGY_MIX_GAS)),
        static_cast<ElectricPowerLevel>(this->watt_.value_or(x..., (uint16_t) ElectricPowerLevel::ELECTRIC_POWER_LEVEL_0)));
  }
};

template<typename... Ts> class AirconManualTempAction : public Action<Ts...>, public Parented<TrumaiNetBoxApp> {
 public:
  TEMPLATABLE_VALUE(uint8_t, temperature)

  void play(Ts... x) override {
    this->parent_->get_aircon_manual()->action_set_temp(this->temperature_.value_or(x..., 0));
  }
};

template<typename... Ts> class TimerDisableAction : public Action<Ts...>, public Parented<TrumaiNetBoxApp> {
 public:
  void play(Ts... x) override { this->parent_->get_timer()->action_timer_disable(); }
};

template<typename... Ts> class TimerActivateAction : public Action<Ts...>, public Parented<TrumaiNetBoxApp> {
 public:
  TEMPLATABLE_VALUE(uint16_t, start)
  TEMPLATABLE_VALUE(uint16_t, stop)
  TEMPLATABLE_VALUE(uint8_t, room_temperature)
  TEMPLATABLE_VALUE(uint8_t, heating_mode)   // was HeatingMode
  TEMPLATABLE_VALUE(uint8_t, water_temperature)
  TEMPLATABLE_VALUE(uint8_t, energy_mix)     // was EnergyMix
  TEMPLATABLE_VALUE(uint16_t, watt)          // was ElectricPowerLevel

  void play(Ts... x) override {
    this->parent_->get_timer()->action_timer_activate(
        this->start_.value(x...),
        this->stop_.value(x...),
        this->room_temperature_.value(x...),
        static_cast<HeatingMode>(this->heating_mode_.value_or(x..., (uint8_t) HeatingMode::HEATING_MODE_OFF)),
        this->water_temperature_.value_or(x..., 0),
        static_cast<EnergyMix>(this->energy_mix_.value_or(x..., (uint8_t) EnergyMix::ENERGY_MIX_NONE)),
        static_cast<ElectricPowerLevel>(this->watt_.value_or(x..., (uint16_t) ElectricPowerLevel::ELECTRIC_POWER_LEVEL_0)));
  }
};

#ifdef USE_TIME
template<typename... Ts> class WriteTimeAction : public Action<Ts...>, public Parented<TrumaiNetBoxApp> {
 public:
  void play(Ts... x) override { this->parent_->get_clock()->action_write_time(); }
};
#endif  // USE_TIME

class TrumaiNetBoxAppHeaterMessageTrigger : public Trigger<const StatusFrameHeater *> {
 public:
  explicit TrumaiNetBoxAppHeaterMessageTrigger(TrumaiNetBoxApp *parent) {
    parent->get_heater()->add_on_message_callback([this](const StatusFrameHeater *message) { this->trigger(message); });
  }
};

}  // namespace truma_inetbox
}  // namespace esphome
