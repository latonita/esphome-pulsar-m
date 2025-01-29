#pragma once
#include <map>
#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

namespace esphome {
namespace pulsar_m {

struct InternalDataState {
  struct Readings {
    uint32_t total{0};
    uint32_t t1{0};
    uint32_t t2{0};
    uint32_t t3{0};
    uint32_t t4{0};
  } energy;
  char timeStr[9]{0};   // "23:59:99"
  char dateStr[11]{0};  // "30/08/2023"
  uint32_t serialNumber{0};
  uint32_t properReads{0};
  uint32_t readErrors{0};
  bool meterFound{false};
  bool initialized{false};
  bool failure{false};
  uint8_t got{0};
  uint32_t lastGoodRead_ms{0};
};

class PulsarMSensor : public sensor::Sensor {
 public:
  void set_channel(uint8_t channel) { this->channel_ = channel; }
  uint8_t get_channel() const { return this->channel_; }

 protected:
  uint8_t channel_{0};
};

class PulsarMComponent : public PollingComponent, public uart::UARTDevice {
 public:
  PulsarMComponent() = default;

  void set_datetime_text_sensor(text_sensor::TextSensor *datetime) { this->datetime_ = datetime; }
  void set_address_text_sensor(text_sensor::TextSensor *address) { this->address_ = address; }
  void set_serial_nr_text_sensor(text_sensor::TextSensor *serial_nr) { this->serial_nr_ = serial_nr; }
  void set_state_text_sensor(text_sensor::TextSensor *state) { this->reading_state_ = state; }

  void set_flow_control_pin(GPIOPin *flow_control_pin) { this->flow_control_pin_ = flow_control_pin; }
  void set_receive_timeout(uint32_t receive_timeout) { this->receive_timeout_ = receive_timeout; }
  void set_requested_meter_address(uint32_t address) { this->requested_meter_address_ = address; }

  float get_setup_priority() const override;

  void dump_config() override;
  void setup() override;

  void loop() override;
  void update() override;

  void register_sensor(PulsarMSensor *sensor);

 protected:
  std::map<uint8_t, PulsarMSensor *> sensors_;
  uint32_t channel_mask_{0};

  text_sensor::TextSensor *datetime_{nullptr};
  text_sensor::TextSensor *address_{nullptr};
  text_sensor::TextSensor *serial_nr_{nullptr};
  text_sensor::TextSensor *reading_state_{nullptr};

  GPIOPin *flow_control_pin_{nullptr};

  uint32_t receive_timeout_{0};
  uint32_t requested_meter_address_{0};
  uint32_t meter_address_bcd_{0};

  InternalDataState data_{};

  enum class State : uint8_t {
    NOT_INITIALIZED,
    IDLE,
    FIND_PULSAR,
    GET_DATE_TIME,
    GET_CHANNELS_DATA,
    GET_METER_INFO,
    PUBLISH_INFO,
  } fsm_state_{State::NOT_INITIALIZED};

  void send_command(uint8_t *buffer, size_t len);
  bool receive_proper_response(uint16_t expectedSize, uint16_t frameId);

  bool find_pulsar();
  bool get_date_time();
  bool get_channels_data();
  bool get_meter_info();

  uint16_t _request_id{0};
  uint16_t generate_request_id() { return ++this->_request_id; }

  const char *error_code_to_string(uint8_t);

  uint32_t uint32_to_bcd4(uint32_t value);
  bool bcd4_to_uint32(uint32_t bcd, uint32_t *value);

  uint16_t crc_16_ibm(const uint8_t *buffer, size_t len);
};

}  // namespace pulsar_m
}  // namespace esphome