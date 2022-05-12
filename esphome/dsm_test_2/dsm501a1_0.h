#pragma once

#include "esphome/core/component.h"
#include "esphome/core/esphal.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/log.h"

#define TAG "dsm501a"

namespace esphome {

class DSM501a1_0SensorStore {
public:
  void setup(GPIOPin *pin) {
    pin->setup();
    this->pin_ = pin->to_isr();
    this->last_fall_ = micros();
    pin->attach_interrupt(&DSM501a1_0SensorStore::gpio_intr, this, CHANGE);
  }
  static void gpio_intr(DSM501a1_0SensorStore *arg);
  uint32_t get_pulse_width_us() const { return this->last_width_; }
  float get_pulse_width_s() const { return this->last_width_ / 1e6f; }
  float get_total_width_s() const { return this->total_width_ / 1e6f; }
  uint32_t get_last_fall() const { return last_fall_; }
  volatile uint32_t total_width_{0};

protected:
  ISRInternalGPIOPin *pin_;
  volatile uint32_t last_fall_{0};
  volatile uint32_t last_width_{0};
};

class DSM501a1_0Sensor : public sensor::Sensor, public PollingComponent {
public:
  DSM501a1_0Sensor() : PollingComponent (5000) {}
  Sensor *lowratio_sensor = new Sensor();
  Sensor *conc_sensor = new Sensor();
  void set_pin(GPIOPin *pin) { pin_ = pin; }
  void setup() override {
    GPIOPin *aux = new GPIOPin (39, INPUT);
    this->set_pin(aux);
    this->store_.setup(this->pin_);
  }
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }
  void update() override;

protected:
  DSM501a1_0SensorStore store_;
  GPIOPin *pin_;
};

void ICACHE_RAM_ATTR DSM501a1_0SensorStore::gpio_intr(DSM501a1_0SensorStore *arg) {
  const bool new_level = arg->pin_->digital_read();
  const uint32_t now = micros();
  if (!new_level) {
    arg->last_fall_ = now;
  } else {
    arg->last_width_ = (now - arg->last_fall_);
    arg->total_width_ += arg->last_width_;
  }
}

void DSM501a1_0Sensor::dump_config() {
  LOG_SENSOR("", "DSM501a 1.0 um", this)
  LOG_UPDATE_INTERVAL(this)
  LOG_PIN("  Pin: ", this->pin_);
}
void DSM501a1_0Sensor::update() {
  float width = this->store_.get_total_width_s();
  float lowratio = (width / 5.0) * 10;
  ESP_LOGCONFIG(TAG, "'%s' - Got lowratio %.3f", this->name_.c_str(), lowratio);
  this->lowratio_sensor->publish_state(lowratio);
  float concentration = 615.55 * lowratio;
  this->conc_sensor->publish_state(concentration);
  this->store_.total_width_ = 0;
}

}  // namespace esphome
