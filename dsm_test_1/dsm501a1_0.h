#pragma once

#include "esphome/core/component.h"
#include "esphome/core/esphal.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

#define TAG "dsm501a"

namespace esphome {

struct DSM501a1_0SensorStore {
  volatile uint32_t last_interrupt{0};
  volatile uint32_t on_time{0};
  volatile bool last_level{false};
  ISRInternalGPIOPin *pin;

  static void gpio_intr(DSM501a1_0SensorStore *arg);
};

class DSM501a1_0Sensor : public sensor::Sensor, public PollingComponent {
public:
  DSM501a1_0Sensor() : PollingComponent (60000) {}
  void set_pin(GPIOPin *pin) { pin_ = pin; }
  Sensor *lowratio_sensor = new Sensor();
  Sensor *conc_sensor = new Sensor();

  void setup() override;
  float get_setup_priority() const override;
  void dump_config() override;
  void update() override;

protected:
  GPIOPin *pin_;

  DSM501a1_0SensorStore store_{};
  uint32_t last_update_;
};

void DSM501a1_0Sensor::setup() {
  ESP_LOGCONFIG(TAG, "Setting up DSM501a 1.0 um Sensor '%s'...", this->get_name().c_str());
  GPIOPin *aux = new GPIOPin (39, INPUT);
  this->set_pin(aux);
  this->pin_->setup();
  this->store_.pin = this->pin_->to_isr();
  this->store_.last_level = this->pin_->digital_read();
  this->last_update_ = micros();
  this->store_.last_interrupt = micros();

  this->pin_->attach_interrupt(DSM501a1_0SensorStore::gpio_intr, &this->store_, CHANGE);
}
void DSM501a1_0Sensor::dump_config() {
  LOG_SENSOR("", "DSM501a 1.0 um Sensor", this);
  LOG_PIN("  Pin: ", this->pin_);
  LOG_UPDATE_INTERVAL(this);
}
void DSM501a1_0Sensor::update() {
  this->pin_->detach_interrupt();
  const uint32_t now = micros();
  const bool level = this->store_.last_level;
  const uint32_t last_interrupt = this->store_.last_interrupt;
  uint32_t on_time = this->store_.on_time;

  const float total_time = float(now - this->last_update_);

  float value = (on_time / total_time) * 100.0f;
  // value -= 3.7;
  // if (value < 0) value = 0;
  ESP_LOGD(TAG, "'%s' Got lowratio = %.1f%%", this->get_name().c_str(), value);
  this->lowratio_sensor->publish_state(value);

  value = 1.1 * pow(value, 3) - 3.8 * pow(value, 2) + 520 * value + 0.62;
  ESP_LOGD(TAG, "'%s' Got PM 1.0 um = %.2f", this->get_name().c_str(), value);
  this->conc_sensor->publish_state(value);

  this->store_.on_time = 0;
  this->last_update_ = now;
  this->pin_->attach_interrupt(DSM501a1_0SensorStore::gpio_intr, &this->store_, CHANGE);
}

float DSM501a1_0Sensor::get_setup_priority() const { return setup_priority::DATA; }

void ICACHE_RAM_ATTR DSM501a1_0SensorStore::gpio_intr(DSM501a1_0SensorStore *arg) {
  const bool new_level = arg->pin->digital_read();
  if (new_level == arg->last_level)
    return;

  if (new_level) {
    arg->last_interrupt = micros();
    arg->last_level = new_level;
  } else {
    const uint32_t now = micros();
    arg->last_level = new_level;
    arg->on_time += now - arg->last_interrupt;
  }
}

}  // namespace esphome
