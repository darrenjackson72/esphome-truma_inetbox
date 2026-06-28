#pragma once

#include "esphome/core/component.h"
#include "TrumaiNetBoxApp.h"

namespace esphome {
namespace truma_inetbox {

template<typename... Ts> class HeaterRoomTempAction : public Action<Ts...>, public Parented<TrumaiNetBoxApp> {
 public:
  TEMPLATABLE_VALUE(uint8_t, temperature)
  TEMPLATABLE_VALUE(uint16_t, heating_mode)

  void play(const Ts &...x) override {
    this->parent_->get_heater()->action_heater_room(this->temperature_.value_or(x..., 0),
                                                    (HeatingMode) this->heating_mode_.value_or(x..., (uint16_t) HeatingMode::HEATING_MODE_OFF));
  }
};

template<typename... Ts> class HeaterWaterTempAction : public Action<Ts...>, public Parented<TrumaiNetBoxApp> {
 public:
  TEMPLATABLE_VALUE(uint8_t, temperature)

  void play(const Ts &...x) override {
    this->parent_->get_heater()->action_heater_water(this->temperature_.value_or(x..., 0));
  }
};

template<typename... Ts> class HeaterWaterTempEnumAction : public Action<Ts...>, public Parented<TrumaiNetBoxApp> {
 public:
  TEMPLATABLE_VALUE(uint16_t, temperature)

  void play(const Ts &...x) override {
    this->parent_->get_heater()->action_heater_water((TargetTemp) this->temperature_.value_or(x..., (uint16_t) TargetTemp::TARGET_TEMP_OFF));
  }
};

template<typename... Ts> class HeaterElecPowerLevelAction : public Action<Ts...>, public Parented<TrumaiNetBoxApp> {
 public:
  TEMPLATABLE_VALUE(uint16_t, watt)

  void play(const Ts &...x) override {
    this->parent_->get_heater()->action_heater_electric_power_level(this->watt_.value_or(x..., 0));
  }
};

template<typename... Ts> class HeaterEnergyMixAction : public Action<Ts...>, public Parented<TrumaiNetBoxApp> {
 public:
  TEMPLATABLE_VALUE(uint8_t, energy_mix)
  TEMPLATABLE_VALUE(uint16_t, watt)

  void play(const Ts &...x) override {
    this->parent_->get_heater()->action_heater_energy_mix(
        (EnergyMix) this->energy_mix_.value_or(x..., (uint8_t) EnergyMix::ENERGY_MIX_GAS),
        (ElectricPowerLevel) this->watt_.value_or(x..., (uint16_t) ElectricPowerLevel::ELECTRIC_POWER_LEVEL_0));
  }
};

template<typename... Ts> class AirconManualTempAction : public Action<Ts...>, public Parented<TrumaiNetBoxApp> {
 public:
  TEMPLATABLE_VALUE(uint8_t, temperature)

  void play(const Ts &...x) override {
    this->parent_->get_aircon_manual()->action_set_temp(this->temperature_.value_or(x..., 0));
  }
};

template<typename... Ts> class AirconManualModeAction : public Action<Ts...>, public Parented<TrumaiNetBoxApp> {
 public:
  TEMPLATABLE_VALUE(uint8_t, mode)

  void play(const Ts &...x) override {
    this->parent_->get_aircon_manual()->action_set_mode((AirconMode) this->mode_.value_or(x..., (uint8_t) AirconMode::AIRCON_MODE_OFF));
  }
};

template<typename... Ts> class AirconManualVentModeAction : public Action<Ts...>, public Parented<TrumaiNetBoxApp> {
 public:
  TEMPLATABLE_VALUE(uint8_t, vent_mode)

  void play(const Ts &...x) override {
    this->parent_->get_aircon_manual()->action_set_vent_mode(
        (AirconVentMode) this->vent_mode_.value_or(x..., (uint8_t) AirconVentMode::AIRCON_VENT_LOW));
  }
};

template<typename... Ts> class AirconManualAction : public Action<Ts...>, public Parented<TrumaiNetBoxApp> {
 public:
  TEMPLATABLE_VALUE(uint8_t, temperature)
  TEMPLATABLE_VALUE(uint8_t, mode)
  TEMPLATABLE_VALUE(uint8_t, vent_mode)

  void play(const Ts &...x) override {
    this->parent_->get_aircon_manual()->action_aircon_manual(
        this->temperature_.value_or(x..., 22),
        (AirconMode) this->mode_.value_or(x..., (uint8_t) AirconMode::AIRCON_MODE_OFF),
        (AirconVentMode) this->vent_mode_.value_or(x..., (uint8_t) AirconVentMode::AIRCON_VENT_LOW));
  }
};

template<typename... Ts> class TimerDisableAction : public Action<Ts...>, public Parented<TrumaiNetBoxApp> {
 public:
  void play(const Ts &...x) override { this->parent_->get_timer()->action_timer_disable(); }
};

template<typename... Ts> class TimerActivateAction : public Action<Ts...>, public Parented<TrumaiNetBoxApp> {
 public:
  TEMPLATABLE_VALUE(uint16_t, start)
  TEMPLATABLE_VALUE(uint16_t, stop)
  TEMPLATABLE_VALUE(uint8_t, room_temperature)
  TEMPLATABLE_VALUE(uint16_t, heating_mode)
  TEMPLATABLE_VALUE(uint8_t, water_temperature)
  TEMPLATABLE_VALUE(uint8_t, energy_mix)
  TEMPLATABLE_VALUE(uint16_t, watt)

  void play(const Ts &...x) override {
    this->parent_->get_timer()->action_timer_activate(
        this->start_.value(x...), this->stop_.value(x...), this->room_temperature_.value(x...),
        (HeatingMode) this->heating_mode_.value_or(x..., (uint16_t) HeatingMode::HEATING_MODE_OFF),
        this->water_temperature_.value_or(x..., 0),
        (EnergyMix) this->energy_mix_.value_or(x..., (uint8_t) EnergyMix::ENERGY_MIX_NONE),
        (ElectricPowerLevel) this->watt_.value_or(x..., (uint16_t) ElectricPowerLevel::ELECTRIC_POWER_LEVEL_0));
  }
};

#ifdef USE_TIME
template<typename... Ts> class WriteTimeAction : public Action<Ts...>, public Parented<TrumaiNetBoxApp> {
 public:
  void play(const Ts &...x) override { this->parent_->get_clock()->action_write_time(); }
};
#endif  // USE_TIME

class TrumaiNetBoxAppHeaterMessageTrigger : public Trigger<const StatusFrameHeater *> {
 public:
  explicit TrumaiNetBoxAppHeaterMessageTrigger(TrumaiNetBoxApp *parent) {
    parent->get_heater()->add_on_message_callback([this](const StatusFrameHeater *message) { this->trigger(message); });
  }
};

class TrumaiNetBoxAppAirconManualMessageTrigger : public Trigger<const StatusFrameAirconManual *> {
 public:
  explicit TrumaiNetBoxAppAirconManualMessageTrigger(TrumaiNetBoxApp *parent) {
    parent->get_aircon_manual()->add_on_message_callback(
        [this](const StatusFrameAirconManual *message) { this->trigger(message); });
  }
};

}  // namespace truma_inetbox
}  // namespace esphome
