#pragma once
#include <map>
#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "pulsar_m_uart.h"
namespace esphome {
namespace pulsar_m {

static const size_t MAX_IN_BUF_SIZE = 256;
static const size_t MAX_OUT_BUF_SIZE = 256;

enum class FunctionCode : uint8_t {
  Error = 0x00,
  Channel = 0x01,
  DateTime = 0x04,
  SetDateTime = 0x05,
  ChannelWeight = 0x07,
  Parameter = 0x0A,
};

enum class ErrorCodes : uint8_t {
  NoFunction = 0x01,
  BitMaskError = 0x02,
  WrongLength = 0x03,
  NoParameter = 0x04,
  WriteBlocked = 0x05,
  OutOfRange = 0x06,
  NoArchiveType = 0x07,
  MaxArchiveValues = 0x08,
  MAX_ERROR_CODE = 0x09
};

enum class Parameters : uint16_t {
  DeviceId = 0x0000,
  NetworkAddress = 0x0001,
  FirmwareVersion = 0x0002,
};

class PulsarMSensor : public sensor::Sensor {
 public:
  void set_channel(uint8_t channel) { this->channel_ = channel; }
  uint8_t get_channel() const { return this->channel_; }

 protected:
  uint8_t channel_{0};
};

using SensorMap = std::map<uint8_t, PulsarMSensor *>;
using ValuesMap = std::map<uint8_t, optional<double>>;
using FrameStopFunction = std::function<bool(uint8_t *buf, size_t size)>;
using ReadFunction = std::function<size_t()>;  // returns number of bytes read

class PulsarMComponent : public PollingComponent, public uart::UARTDevice {
 public:
  PulsarMComponent() : tag_(generateTag()){};

  float get_setup_priority() const override;

  void setup() override;
  void dump_config() override;

  void register_sensor(PulsarMSensor *sensor);

  void loop() override;
  void update() override;

  void set_flow_control_pin(GPIOPin *flow_control_pin) { this->flow_control_pin_ = flow_control_pin; }
  void set_receive_timeout(uint32_t receive_timeout) { this->receive_timeout_ms_ = receive_timeout; }
  void set_meter_address(uint32_t address);

  void set_device_time(uint32_t timestamp);  

#ifdef USE_TEXT_SENSOR
  SUB_TEXT_SENSOR(datetime)
  SUB_TEXT_SENSOR(address)
#endif

 protected:
  uint32_t meter_address_{0};
  uint32_t meter_address_bcd_{0};
  uint32_t receive_timeout_ms_{500};

  SensorMap sensors_;
  uint32_t channel_mask_{0};
  ValuesMap values_;
  
  char datetime_str_[20] = "Not set";
  uint32_t time_to_set_{0};
  uint32_t time_to_set_requested_at_ms_{0};

  GPIOPin *flow_control_pin_{nullptr};
  std::unique_ptr<PulsarMUart> iuart_;

  enum class State : uint8_t {
    NOT_INITIALIZED,
    IDLE,
    TRY_LOCK_BUS,
    WAIT,
    WAITING_FOR_RESPONSE,
    OPEN_SESSION,
    FIND_PULSAR,
    READ_PULSAR_ADDRESS,
    SET_DATE_TIME,
    READ_SET_DATE_TIME,
    REQ_DATE_TIME,
    READ_DATE_TIME,
    REQ_CHANNELS_DATA,
    READ_CHANNELS_DATA,
    PUBLISH_INFO,
  } state_{State::NOT_INITIALIZED};
  State last_reported_state_{State::NOT_INITIALIZED};
  struct {
    uint32_t start_time{0};
    uint32_t delay_ms{0};
    State next_state{State::IDLE};
  } wait_;

  bool is_idling() const { return this->state_ == State::WAIT || this->state_ == State::IDLE; };

  void set_next_state_(State next_state) { state_ = next_state; };
  void set_next_state_delayed_(uint32_t ms, State next_state);

  void read_reply_and_go_next_state_(ReadFunction read_fn, State next_state, uint8_t retries, bool mission_critical,
                                     bool check_crc);
  struct {
    ReadFunction read_fn;
    State next_state;
    bool mission_critical;
    bool check_crc;
    uint8_t tries_max;
    uint8_t tries_counter;
    uint32_t err_crc;
    uint32_t err_invalid_frames;
  } reading_state_{nullptr, State::IDLE, false, false, 0, 0, 0, 0};

  size_t received_frame_size_{0};

  uint32_t last_rx_time_{0};
  struct {
    uint8_t in[MAX_IN_BUF_SIZE];
    size_t amount_in;
    uint8_t out[MAX_OUT_BUF_SIZE];
    size_t amount_out;
  } buffers_;

  void clear_rx_buffers_();

  void prepare_frame_(const uint8_t *data, size_t length, bool calc_crc = true);
  void send_frame_(const uint8_t *data, size_t length, bool calc_crc = true);
  void send_frame_prepared_();

  size_t receive_frame_(FrameStopFunction stop_fn);
  size_t receive_frame_discovery_();
  size_t receive_frame_data_(FunctionCode fn_code, uint16_t frame_id);

  inline void update_last_rx_time_() { this->last_rx_time_ = millis(); }
  bool check_wait_timeout_() { return millis() - wait_.start_time >= wait_.delay_ms; }
  bool check_rx_timeout_() { return millis() - this->last_rx_time_ >= receive_timeout_ms_; }

  void report_failure(bool failure);
  void abort_mission_();

  const char *state_to_string(State state);
  void log_state_(State *next_state = nullptr);

  struct Stats {
    uint32_t connections_tried_{0};
    uint32_t crc_errors_{0};
    uint32_t crc_errors_recovered_{0};
    uint32_t invalid_frames_{0};
    uint8_t failures_{0};

    float crc_errors_per_session() const { return (float) crc_errors_ / connections_tried_; }
  } stats_;
  void stats_dump();

  uint8_t failures_before_reboot_{0};

  uint16_t frame_id_{0};
  uint16_t generate_frame_id() { return ++this->frame_id_; }

  static const char *error_code_to_string(uint8_t);
  static uint32_t uint32_to_bcd4(uint32_t value);
  static bool bcd4_to_uint32(uint32_t bcd, uint32_t *value);
  static uint16_t crc_16_ibm(const uint8_t *buffer, size_t len);

  bool try_lock_uart_session_();
  void unlock_uart_session_();

 private:
  static uint8_t next_obj_id_;
  std::string tag_;

  static std::string generateTag();
};

}  // namespace pulsar_m
}  // namespace esphome