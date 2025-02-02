//#include <bitset>
#include <cstdint>
#include "esphome/core/application.h"
#include "esphome/core/log.h"
#include "esphome/core/time.h"
#include "object_locker.h"
#include "pulsar_m.h"

namespace esphome {
namespace pulsar_m {

static const char *TAG0 = "pulsar_m_";

#define TAG (this->tag_.c_str())

static constexpr uint8_t BOOT_WAIT_S = 10;  // avoid communications until properly booted

#pragma pack(1)
// Протокол Пульсар-М

// Frame max total len 255

// ADDRESS : 4 bytes  : BCD MSB
// FN      : 1 byte   : UINT8
// LEN     : 1 byte   : UINT8
// PAYLOAD : 0..245 b : BLOB
// ID      : 2 bytes  : UINT16
// CRC16   : 2 bytes  : UINT16

struct FrameHeader {
  uint32_t address_bcd;
  FunctionCode fn;
  uint8_t len;
};

struct FrameFooter {
  uint16_t id;
  uint16_t crc;
};

struct FrameDiscoveryReq {
  FrameHeader header;
  uint8_t zero8{0};
  FrameFooter footer{0, 0};
  constexpr FrameDiscoveryReq() : header{.address_bcd = 0xF00F0FF0, .fn = FunctionCode::Error} {};
};

struct FrameDiscoveryResp {
  uint32_t bcd;
  uint32_t address;
  uint16_t crc;
};

struct FrameDataReq {
  FrameHeader header;
  uint32_t channel_mask{0};
  FrameFooter footer;

  FrameDataReq() = delete;
  constexpr FrameDataReq(uint32_t address_bcd, FunctionCode fn_code, uint16_t id)
      : header{.address_bcd = address_bcd, .fn = fn_code, .len = sizeof(FrameDataReq)}, footer{.id = id} {}
};

struct FrameDateTimeReq {
  FrameHeader header;
  FrameFooter footer;

  FrameDateTimeReq() = delete;
  constexpr FrameDateTimeReq(uint32_t address_bcd, uint16_t id)
      : header{.address_bcd = address_bcd, .fn = FunctionCode::DateTime, .len = sizeof(FrameDateTimeReq)},
        footer{.id = id} {}
};

/*
Все поля uint8_t
YEAR – год, отсчитывается от 2000, допустимые значения [0…99].
MONTH – месяц, допустимые значения [1…12].
DAY – день месяца, допустимые значения [1…31].
HOUR – час, допустимые значения [0…23].
MINUTE – минута, допустимые значения [0…59].
SECOND – секунда, допустимые значения [0…59]

0xFF - значение неизвестно
*/
struct FrameDateTimeResp {
  FrameHeader header;
  uint8_t year{0xff};
  uint8_t month{0xff};
  uint8_t day{0xff};
  uint8_t hour{0xff};
  uint8_t minute{0xff};
  uint8_t second{0xff};
  FrameFooter footer;
  constexpr FrameDateTimeResp() : header{.fn = FunctionCode::DateTime, .len = sizeof(FrameDateTimeResp)}, footer{} {}
};

struct FrameParamReq {
  FrameHeader header;
  uint16_t param_num;
  FrameFooter footer;

  FrameParamReq() = delete;
  constexpr FrameParamReq(uint32_t address_bcd, uint16_t param, uint16_t id)
      : header{.address_bcd = address_bcd, .fn = FunctionCode::Parameter, .len = sizeof(FrameParamReq)},
        param_num{param},
        footer{.id = id} {}
};

#pragma pack(0)

static_assert(sizeof(FrameDateTimeReq) == 10, "Wrong structure size: FrameDateTimeReq");
static_assert(sizeof(FrameDateTimeResp) == 16, "Wrong structure size: FrameDateTimeResp");

// there shall be double64_t and float32_t
static_assert(sizeof(double) == 8, "double size is not 8 bytes.");
static_assert(sizeof(float) == 4, "float size is not 4 bytes.");

static constexpr size_t FRAME_OVERHEAD = sizeof(FrameHeader) + sizeof(FrameFooter);

float PulsarMComponent::get_setup_priority() const { return setup_priority::AFTER_WIFI; }

void PulsarMComponent::setup() {
  ESP_LOGD(TAG, "setup");
#ifdef USE_ESP32_FRAMEWORK_ARDUINO
  iuart_ = make_unique<PulsarMUart>(*static_cast<uart::ESP32ArduinoUARTComponent *>(this->parent_));
#endif

#ifdef USE_ESP_IDF
  iuart_ = make_unique<PulsarMUart>(*static_cast<uart::IDFUARTComponent *>(this->parent_));
#endif

#if USE_ESP8266
  iuart_ = make_unique<PulsarMUart>(*static_cast<uart::ESP8266UartComponent *>(this->parent_));
#endif
  if (this->flow_control_pin_ != nullptr) {
    this->flow_control_pin_->setup();
  }

  this->set_timeout(BOOT_WAIT_S * 1000, [this]() {
    ESP_LOGD(TAG, "Boot timeout, component is ready to use");
    this->clear_rx_buffers_();
    this->set_next_state_(State::IDLE);
  });
}

void PulsarMComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Pulsar-M: %p", this);
  LOG_UPDATE_INTERVAL(this);
  LOG_PIN("  Flow Control Pin: ", this->flow_control_pin_);
  ESP_LOGCONFIG(TAG, "  Receive timeout: %.1fs", this->receive_timeout_ms_ / 1e3f);
  ESP_LOGCONFIG(TAG, "  Update interval: %.1fs", this->update_interval_ / 1e3f);
  ESP_LOGCONFIG(TAG, "  Meter address: %u", this->meter_address_);
#ifdef USE_SENSOR
  for (const auto &it : this->sensors_) {
    LOG_SENSOR("  ", "  Sensor", it.second);
  }
#endif
}

void PulsarMComponent::register_sensor(PulsarMSensor *sensor) {
  ESP_LOGV(TAG, "register_sensor() channel %d", sensor->get_channel());
  this->sensors_[sensor->get_channel()] = sensor;
  this->channel_mask_ |= 1 << (sensor->get_channel() - 1);
  this->values_[sensor->get_channel()] = nullopt;
}

void PulsarMComponent::abort_mission_() {
  this->unlock_uart_session_();
  this->set_next_state_(State::IDLE);
  this->report_failure(true);
}

void PulsarMComponent::report_failure(bool failure) {
  if (!failure) {
    this->stats_.failures_ = 0;
    return;
  }

  this->stats_.failures_++;
  if (this->failures_before_reboot_ > 0 && this->stats_.failures_ > this->failures_before_reboot_) {
    ESP_LOGE(TAG, "Too many failures in a row. Let's try rebooting device.");
    delay(100);
    App.safe_reboot();
  }
}

void PulsarMComponent::loop() {
  if (!this->is_ready() || this->state_ == State::NOT_INITIALIZED)
    return;

  switch (this->state_) {
    case State::IDLE: {
      this->update_last_rx_time_();
    } break;

    case State::TRY_LOCK_BUS: {
      this->log_state_();
      if (this->try_lock_uart_session_()) {
        this->stats_.connections_tried_++;
        this->clear_rx_buffers_();
        if (this->meter_address_ == 0) {
          this->set_next_state_(State::FIND_PULSAR);
        } else if (this->time_to_set_ != 0) {
          this->set_next_state_(State::SET_DATE_TIME);
        } else {
          this->set_next_state_(State::REQ_DATE_TIME);
        }
      } else {
        ESP_LOGV(TAG, "UART Bus is busy, waiting ...");
        this->set_next_state_delayed_(1000, State::TRY_LOCK_BUS);
      }
    } break;

    case State::WAIT: {
      if (this->check_wait_timeout_()) {
        this->set_next_state_(this->wait_.next_state);
        this->update_last_rx_time_();
      }
    } break;

    case State::WAITING_FOR_RESPONSE: {
      this->log_state_(&reading_state_.next_state);
      received_frame_size_ = reading_state_.read_fn();

      bool crc_is_ok = true;
      if (reading_state_.check_crc && received_frame_size_ > 0) {
        crc_is_ok = (0x0000 == this->crc_16_ibm(this->buffers_.in, received_frame_size_));
      }

      // special case for the unexpected frame with error. it has proper CRC, it
      // has fn=0, len=11 special case if we are in FindPulsar mode - double check

      // happy path
      if (this->received_frame_size_ > 0 && crc_is_ok) {
        this->set_next_state_(reading_state_.next_state);
        this->update_last_rx_time_();
        this->stats_.crc_errors_ += reading_state_.err_crc;
        this->stats_.crc_errors_recovered_ += reading_state_.err_crc;
        this->stats_.invalid_frames_ += reading_state_.err_invalid_frames;
        return;
      }

      // half-happy path
      // if not timed out yet, wait for data to come a little more
      if (crc_is_ok && !this->check_rx_timeout_()) {
        return;
      }
      if (this->received_frame_size_ == 0) {
        this->reading_state_.err_invalid_frames++;
        ESP_LOGW(TAG, "RX timeout");
      } else if (!crc_is_ok) {
        this->reading_state_.err_crc++;
        ESP_LOGW(TAG, "Frame received, but CRC failed.");
      } else {
        this->reading_state_.err_invalid_frames++;
        ESP_LOGW(TAG, "Frame corrupted.");
      }

      // if we are here, we have a timeout and no data
      // it means we have a failure
      // - either no reply from the meter at all
      // - or corrupted data and id doesn't trigger stop function
      if (this->buffers_.amount_in > 0) {
        this->stats_.crc_errors_++;
        ESP_LOGVV(TAG, "RX: %s", format_hex_pretty(this->buffers_.in, this->buffers_.amount_in).c_str());
      }
      this->clear_rx_buffers_();

      if (reading_state_.mission_critical) {
        this->stats_.crc_errors_ += reading_state_.err_crc;
        this->stats_.invalid_frames_ += reading_state_.err_invalid_frames;
        this->abort_mission_();
        return;
      }

      if (reading_state_.tries_counter < reading_state_.tries_max) {
        reading_state_.tries_counter++;
        ESP_LOGW(TAG, "Retrying [%d/%d]...", reading_state_.tries_counter, reading_state_.tries_max);
        this->send_frame_prepared_();
        this->update_last_rx_time_();
        return;
      }
      received_frame_size_ = 0;
      // failure, advancing to next state with no data received (frame_size = 0)
      this->stats_.crc_errors_ += reading_state_.err_crc;
      this->stats_.invalid_frames_ += reading_state_.err_invalid_frames;
      this->set_next_state_(reading_state_.next_state);
    } break;

    case State::FIND_PULSAR: {
      this->log_state_();
      FrameDiscoveryReq req;
      this->send_frame_((uint8_t *) &req, sizeof(FrameDiscoveryReq));

      auto read_fn = [this]() { return this->receive_frame_discovery_(); };
      this->read_reply_and_go_next_state_(read_fn, State::READ_PULSAR_ADDRESS, 0, true, false);

    } break;

    case State::READ_PULSAR_ADDRESS: {
      this->log_state_();
      if (this->received_frame_size_) {
        this->update_last_rx_time_();
        this->set_next_state_(State::REQ_DATE_TIME);
      }
    } break;

    case State::SET_DATE_TIME: {
      if (this->time_to_set_ == 0) {
        this->set_next_state_(State::REQ_DATE_TIME);
        break;
      }

      this->log_state_();

      FrameDateTimeResp req;
      req.header.address_bcd = this->meter_address_bcd_;
      req.header.fn = FunctionCode::SetDateTime;
      req.header.len = sizeof(FrameDateTimeResp);
      auto id = req.footer.id = this->generate_frame_id();

      uint32_t ms_since_asked = millis() - this->time_to_set_requested_at_ms_;
      auto time = ESPTime::from_epoch_local(this->time_to_set_ + ms_since_asked / 1000);  // Convert to local time

      this->time_to_set_ = 0;
      this->time_to_set_requested_at_ms_ = 0;

      req.year = time.year - 2000;
      req.month = time.month;
      req.day = time.day_of_month;
      req.hour = time.hour;
      req.minute = time.minute;
      req.second = time.second;
      ESP_LOGD(TAG, "Adjusting requested time by +%d seconds", ms_since_asked / 1000);
      ESP_LOGD(TAG, "Setting time to: %04d-%02d-%02d %02d:%02d:%02d", time.year, time.month, time.day_of_month,
               time.hour, time.minute, time.second);

      this->send_frame_((uint8_t *) &req, sizeof(FrameDateTimeResp));

      auto read_fn = [this, id]() { return this->receive_frame_data_(FunctionCode::SetDateTime, id); };

      this->read_reply_and_go_next_state_(read_fn, State::READ_SET_DATE_TIME, 0, false, true);

    } break;

    case State::READ_SET_DATE_TIME: {
      this->log_state_();
      this->set_next_state_(State::REQ_DATE_TIME);

      if (this->received_frame_size_) {
        uint8_t result = this->buffers_.in[sizeof(FrameHeader)];
        if (result == 1) {
          ESP_LOGD(TAG, "Date/time set successfully");
        } else {
          ESP_LOGW(TAG, "Date/time set failed");
        }
      }

    } break;

    case State::REQ_DATE_TIME: {
      this->log_state_();

      FrameDateTimeReq req(this->meter_address_bcd_, this->generate_frame_id());
      auto id = req.footer.id;

      this->send_frame_((uint8_t *) &req, sizeof(FrameDateTimeReq));

      ESP_LOGD(TAG, "Requesting date/time");
      auto read_fn = [this, id]() { return this->receive_frame_data_(FunctionCode::DateTime, id); };

      this->read_reply_and_go_next_state_(read_fn, State::READ_DATE_TIME, 0, false, true);

    } break;

    case State::READ_DATE_TIME: {
      this->log_state_();
      this->set_next_state_(State::REQ_CHANNELS_DATA);
      if (this->received_frame_size_ == 0) {
        ESP_LOGW(TAG, "No date/time received");
        break;
      }

      FrameDateTimeResp &res = *(FrameDateTimeResp *) this->buffers_.in;

      if (res.year == 0xFF || res.month == 0xFF || res.day == 0xFF || res.hour == 0xFF || res.minute == 0xFF ||
          res.second == 0xFF) {
        ESP_LOGD(TAG, "Date/time not set");
        snprintf(this->datetime_str_, sizeof(this->datetime_str_), "Not set\0");
      } else {
        uint16_t year = 2000 + res.year;
        snprintf(this->datetime_str_, sizeof(this->datetime_str_), "%04u-%02u-%02u %02u:%02u:%02u", year, res.month,
                 res.day, res.hour, res.minute, res.second);

        ESP_LOGD(TAG, "Got datetime: %s", this->datetime_str_);
      }

    } break;

    case State::REQ_CHANNELS_DATA: {
      uint8_t num = this->sensors_.size();
      if (num == 0) {
        ESP_LOGD(TAG, "No sensors registered. Next");
        this->set_next_state_(State::PUBLISH_INFO);
        break;
      }
      // print out channel mask in hex and number of channels
      ESP_LOGV(TAG, "Number of channels: %d (mask: 0x%08X)", num, this->channel_mask_);
      ESP_LOGD(TAG, "Requesting channel readings from meter N %u", this->meter_address_);

      FrameDataReq req(this->meter_address_bcd_, FunctionCode::Channel, this->generate_frame_id());
      req.channel_mask = this->channel_mask_;

      this->send_frame_((uint8_t *) &req, sizeof(FrameDataReq));
      auto id = req.footer.id;
      auto read_fn = [this, id]() { return this->receive_frame_data_(FunctionCode::Channel, id); };
      this->read_reply_and_go_next_state_(read_fn, State::READ_CHANNELS_DATA, 0, false, true);

    } break;

    case State::READ_CHANNELS_DATA: {
      this->log_state_();
      this->set_next_state_(State::PUBLISH_INFO);
      //      this->param_nr_ = 0;

      if (this->received_frame_size_) {
        uint8_t num = this->sensors_.size();  // one sensor per channel
        size_t expected16 = FRAME_OVERHEAD + num * sizeof(uint16_t);
        size_t expected32 = FRAME_OVERHEAD + num * sizeof(float);
        size_t expected64 = FRAME_OVERHEAD + num * sizeof(double);

        if (this->received_frame_size_ == expected16) {
          // this have to be uint16_t
          uint16_t *pv = (uint16_t *) (this->buffers_.in + sizeof(FrameHeader));
          for (auto &itv : this->values_) {
            itv.second = *pv;
            pv++;
          }
        } else if (this->received_frame_size_ == expected32 && this->is_integer_) {
          uint32_t *pv = (uint32_t *) (this->buffers_.in + sizeof(FrameHeader));
          for (auto &itv : this->values_) {
            itv.second = (double) *pv;
            pv++;
          }
        } else if (this->received_frame_size_ == expected32 && !this->is_integer_) {
          float *pv = (float *) (this->buffers_.in + sizeof(FrameHeader));
          for (auto &itv : this->values_) {
            itv.second = *pv;
            pv++;
          }
        } else if (this->received_frame_size_ == expected64 && this->is_integer_) {
          uint64_t *pv = (uint64_t *) (this->buffers_.in + sizeof(FrameHeader));
          for (auto &itv : this->values_) {
            itv.second = (double) *pv;
            pv++;
          }
        } else if (this->received_frame_size_ == expected64 && !this->is_integer_) {
          double *pv = (double *) (this->buffers_.in + sizeof(FrameHeader));
          for (auto &itv : this->values_) {
            itv.second = *pv;
            pv++;
          }
        } else {
          ESP_LOGW(TAG, "Wrong frame size %d, expected either %d, %d, or %d for 16, 32, or 64 bit values",
                   received_frame_size_, expected16, expected32, expected64);

          // clean values ? need to decide later

          // for (auto &[channel, value] : this->values_) {
          //   value = std::nullopt;
          // }

          // this->abort_mission_();

          break;
        }
      }
    } break;

      // case State::REQ_PARAM: {
      //   this->log_state_();

      //   if (++this->param_nr_ >= 8) {
      //     this->param_nr_ = 0;
      //     this->set_next_state_(State::PUBLISH_INFO);
      //     break;
      //   }

      //   FrameParamReq req(this->meter_address_bcd_, this->param_nr_, this->generate_frame_id());
      //   this->send_frame_((uint8_t *) &req, sizeof(FrameParamReq));
      //   auto id = req.footer.id;
      //   auto read_fn = [this, id]() { return this->receive_frame_data_(FunctionCode::Parameter, id); };
      //   this->read_reply_and_go_next_state_(read_fn, State::READ_PARAM, 0, false, true);

      // } break;

      // case State::READ_PARAM: {
      //   this->log_state_();
      //   this->set_next_state_(State::REQ_PARAM);

      //   if (this->received_frame_size_ > FRAME_OVERHEAD) {
      //     auto len = received_frame_size_ - FRAME_OVERHEAD;

      //     ESP_LOGD(TAG, "Param %d: %s", this->param_nr_,
      //              format_hex_pretty(this->buffers_.in + sizeof(FrameHeader), len).c_str());
      //   }
      // } break;

    case State::PUBLISH_INFO: {
      this->unlock_uart_session_();
      this->log_state_();
      ESP_LOGD(TAG, "Publishing data");
      this->update_last_rx_time_();
      this->set_next_state_(State::IDLE);

      if (this->values_.empty()) {
        ESP_LOGV(TAG, "No values to publish");
        break;
      }

      for (auto &its : this->values_) {
        auto channel = its.first;
        auto value = its.second;
        if (value.has_value()) {
          this->sensors_[channel]->publish_state(*value);
        } else {
          ESP_LOGW(TAG, "Channel %d: no value", channel);
        }
      }
#ifdef USE_TEXT_SENSOR
      if (this->datetime_text_sensor_ != nullptr) {
        this->datetime_text_sensor_->publish_state(this->datetime_str_);
      }
      if (this->address_text_sensor_ != nullptr) {
        this->address_text_sensor_->publish_state(to_string(this->meter_address_));
      }
#endif
      // this->stats_dump();
    } break;

    default:
      break;
  }
}

void PulsarMComponent::update() {
  if (!this->is_idling()) {
    ESP_LOGD(TAG, "Starting data collection impossible - component not ready");
    return;
  }
  ESP_LOGD(TAG, "Starting data collection");

  this->set_next_state_(State::TRY_LOCK_BUS);
}

void PulsarMComponent::set_meter_address(uint32_t address) {
  this->meter_address_ = address;
  this->meter_address_bcd_ = this->uint32_to_bcd4(address);
}

void PulsarMComponent::set_device_time(uint32_t timestamp) {
  ESP_LOGD(TAG, "set_device_time: %u", timestamp);
  this->time_to_set_ = timestamp;
  this->time_to_set_requested_at_ms_ = millis();

  auto time = ESPTime::from_epoch_local(timestamp);  // Convert to local time

  int year = time.year;
  int month = time.month;
  int day = time.day_of_month;
  int hour = time.hour;
  int minute = time.minute;
  int second = time.second;

  ESP_LOGD(TAG, "Time set queued: %04d-%02d-%02d %02d:%02d:%02d", year, month, day, hour, minute, second);
  // this->update();
}

void PulsarMComponent::set_next_state_delayed_(uint32_t ms, State next_state) {
  if (ms == 0) {
    set_next_state_(next_state);
  } else {
    ESP_LOGV(TAG, "Short delay for %u ms", ms);
    set_next_state_(State::WAIT);
    wait_.start_time = millis();
    wait_.delay_ms = ms;
    wait_.next_state = next_state;
  }
}

void PulsarMComponent::read_reply_and_go_next_state_(ReadFunction read_fn, State next_state, uint8_t retries,
                                                     bool mission_critical, bool check_crc) {
  reading_state_ = {};
  reading_state_.read_fn = read_fn;
  reading_state_.mission_critical = mission_critical;
  reading_state_.tries_max = retries;
  reading_state_.tries_counter = 0;
  reading_state_.check_crc = check_crc;
  reading_state_.next_state = next_state;
  received_frame_size_ = 0;

  set_next_state_(State::WAITING_FOR_RESPONSE);
}

void PulsarMComponent::prepare_frame_(const uint8_t *data, size_t length, bool calc_crc) {
  memcpy(this->buffers_.out, data, length);
  this->buffers_.amount_out = length;
  if (calc_crc && length > 2) {
    uint16_t crc = this->crc_16_ibm(data, length - 2);
    this->buffers_.out[length - 2] = crc & 0xFF;
    this->buffers_.out[length - 1] = crc >> 8;
  }
}

void PulsarMComponent::send_frame_(const uint8_t *data, size_t length, bool calc_crc) {
  this->prepare_frame_(data, length, calc_crc);
  this->send_frame_prepared_();
}

void PulsarMComponent::send_frame_prepared_() {
  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(true);

  this->write_array(this->buffers_.out, this->buffers_.amount_out);

  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(false);

  ESP_LOGV(TAG, "TX: %s", format_hex_pretty(this->buffers_.out, this->buffers_.amount_out).c_str());
}

size_t PulsarMComponent::receive_frame_discovery_() {
  auto stop_fn = [this](const uint8_t *data, size_t len) {
    if (len != sizeof(FrameDiscoveryResp)) {
      return false;
    }

    if (this->crc_16_ibm(data, len) != 0) {
      ESP_LOGD(TAG, "Discovery Stop_Fn CRC is wrong. RX: %s", format_hex_pretty(data, len).c_str());
      return false;
    }

    FrameDiscoveryResp *resp = (FrameDiscoveryResp *) data;
    uint32_t addr = 0;
    if (this->bcd4_to_uint32(resp->address, &addr)) {
      ESP_LOGV(TAG, "Got a reply from meter with address %u", addr);
      this->meter_address_bcd_ = resp->address;
      this->meter_address_ = addr;
      return true;
    }

    ESP_LOGE(TAG, "Got a erroneous reply from meter - wrong BCD/address");
    return false;
  };

  return this->receive_frame_(stop_fn);
}

size_t PulsarMComponent::receive_frame_data_(FunctionCode fn_code, uint16_t id) {
  auto stop_fn = [this, fn_code, id](const uint8_t *data, size_t len) {
    if (len < sizeof(FrameHeader) + sizeof(FrameFooter))
      return false;

    if (this->crc_16_ibm(data, len) != 0) {
      return false;
    }

    FrameHeader *header = (FrameHeader *) data;
    FrameFooter *footer = (FrameFooter *) (data + len - sizeof(FrameFooter));

    if (len == 11 && header->fn == FunctionCode::Error) {
      ESP_LOGW(TAG, "Error frame received: %s", error_code_to_string(data[sizeof(FrameHeader)]));
      return false;
    }

    if (header->fn != fn_code) {
      ESP_LOGW(TAG, "Unexpected function code %d, expected %d", header->fn, fn_code);
      return false;
    }

    if (footer->id != id) {
      ESP_LOGW(TAG, "Unexpected id %d, expected %d", footer->id, id);
      return false;
    }

    return true;
  };

  return this->receive_frame_(stop_fn);
}

size_t PulsarMComponent::receive_frame_(FrameStopFunction stop_fn) {
  const uint32_t read_time_limit_ms = 25;
  size_t ret_val;

  auto count = this->available();
  if (count <= 0)
    return 0;

  uint32_t read_start = millis();
  uint8_t *p;
  while (count-- > 0) {
    if (millis() - read_start > read_time_limit_ms) {
      return 0;
    }

    if (this->buffers_.amount_in < MAX_IN_BUF_SIZE) {
      p = &this->buffers_.in[this->buffers_.amount_in];
      if (!iuart_->read_one_byte(p)) {
        return 0;
      }
      this->buffers_.amount_in++;
    } else {
      memmove(this->buffers_.in, this->buffers_.in + 1, this->buffers_.amount_in - 1);
      p = &this->buffers_.in[this->buffers_.amount_in - 1];
      if (!iuart_->read_one_byte(p)) {
        return 0;
      }
    }

    if (stop_fn(this->buffers_.in, this->buffers_.amount_in)) {
      ESP_LOGV(TAG, "RX: %s", format_hex_pretty(this->buffers_.in, this->buffers_.amount_in).c_str());
      ret_val = this->buffers_.amount_in;
      this->buffers_.amount_in = 0;
      this->update_last_rx_time_();
      return ret_val;
    }

    yield();
    App.feed_wdt();
  }
  return 0;
}

void PulsarMComponent::clear_rx_buffers_() {
  int available = this->available();
  if (available > 0) {
    ESP_LOGVV(TAG, "Cleaning garbage from UART input buffer: %d bytes", available);
  }

  int len;
  while (available > 0) {
    len = std::min(available, (int) MAX_IN_BUF_SIZE);
    this->read_array(this->buffers_.in, len);
    available -= len;
  }
  memset(this->buffers_.in, 0, MAX_IN_BUF_SIZE);
  this->buffers_.amount_in = 0;
}

//----------------------------------------------------------------------------
// convert uint32 to 4-byte BCD, each byte is 2 decimal digits, 8 digits in
// total example: input = 04026591, output = 0x04 0x02 0x65 0x91
//----------------------------------------------------------------------------
uint32_t PulsarMComponent::uint32_to_bcd4(uint32_t value) {
  uint32_t result = 0;
  uint8_t *buffer = (uint8_t *) &result;
  for (int i = 3; i >= 0; --i) {
    buffer[i] = ((value / 10) % 10) << 4 | ((value % 10));
    value /= 100;
  }
  return result;
}

//----------------------------------------------------------------------------
// convert 4-byte BCD to uint32, each byte is 2 decimal digits, 8 digits in
// total example: input = 0x04 0x02 0x65 0x91, output = 04026591
//----------------------------------------------------------------------------
bool PulsarMComponent::bcd4_to_uint32(uint32_t bcd, uint32_t *value) {
  uint32_t result = 0;
  uint8_t *buffer = (uint8_t *) &bcd;
  for (int i = 0; i < 4; i++) {
    uint8_t high = (buffer[i] >> 4) & 0x0F;
    uint8_t low = buffer[i] & 0x0F;
    if (high > 9 || low > 9)
      return false;
    result = (result * 100) + (high * 10) + low;
  }
  *value = result;
  return true;
}

//----------------------------------------------------------------------------
// CRC-16-IBM
// Polynom: 0xA001
// Starting value: 0xFFFF
// Start from: LSB
// XOR in the end: no
//----------------------------------------------------------------------------
uint16_t PulsarMComponent::crc_16_ibm(uint8_t const *buffer, size_t len) {
  uint8_t i;
  uint16_t result = 0xFFFF;
  while (len--) {
    result ^= *buffer++;
    for (i = 0; i < 8; i++)
      result = (result & 1) ? (result >> 1) ^ 0xA001 : result >> 1;
  }
  return result;
}

/*
Стандартные коды ошибок:
0x01 – отсутствует запрашиваемый код функции.
0x02 – ошибка в битовой маске запроса.
0x03 – ошибочная длинна запроса.
0x04 – отсутствует параметр.
0x05 – запись заблокирована, требуется авторизация.
0x06 – записываемое значение (параметр) находится вне заданного диапазона.
0x07 – отсутствует запрашиваемый тип архива.
0x08 – превышение максимального количества архивных значений за один пакет.
*/
const char *PulsarMComponent::error_code_to_string(uint8_t error) {
  switch (error) {
    case 0x01:
      return "No function";
    case 0x02:
      return "Bit mask error";
    case 0x03:
      return "Wrong request length";
    case 0x04:
      return "Parameter missing";
    case 0x05:
      return "Write blocked, authorization required";
    case 0x06:
      return "Value out of range";
    case 0x07:
      return "Archive type missing";
    case 0x08:
      return "Max archive values exceeded";
    case 0x0B:
      return "Unknown error - selected channel not supported (?)";
    default:
      return "Unknown error";
  }
}

const char *PulsarMComponent::state_to_string(State state) {
  switch (state) {
    case State::IDLE:
      return "IDLE";
    case State::TRY_LOCK_BUS:
      return "TRY_LOCK_BUS";
    case State::WAIT:
      return "WAIT";
    case State::WAITING_FOR_RESPONSE:
      return "WAITING_FOR_RESPONSE";
    case State::FIND_PULSAR:
      return "FIND_PULSAR";
    case State::READ_PULSAR_ADDRESS:
      return "READ_PULSAR_ADDRESS";
    case State::SET_DATE_TIME:
      return "SET_DATE_TIME";
    case State::READ_SET_DATE_TIME:
      return "READ_SET_DATE_TIME";
    case State::REQ_DATE_TIME:
      return "REQ_DATE_TIME";
    case State::READ_DATE_TIME:
      return "READ_DATE_TIME";
    case State::REQ_CHANNELS_DATA:
      return "REQ_CHANNELS_DATA";
    case State::READ_CHANNELS_DATA:
      return "READ_CHANNELS_DATA";
    case State::PUBLISH_INFO:
      return "PUBLISH_INFO";
    default:
      return "UNKNOWN";
  }
}

void PulsarMComponent::log_state_(State *next_state) {
  if (this->state_ != this->last_reported_state_) {
    if (next_state == nullptr) {
      ESP_LOGV(TAG, "State::%s", this->state_to_string(this->state_));
    } else {
      ESP_LOGV(TAG, "State::%s -> %s", this->state_to_string(this->state_), this->state_to_string(*next_state));
    }
    this->last_reported_state_ = this->state_;
  }
}

void PulsarMComponent::stats_dump() {
  ESP_LOGD(TAG, "============================================");
  ESP_LOGD(TAG, "Data collection and publishing finished.");
  ESP_LOGD(TAG, "Total number of sessions ............. %u", this->stats_.connections_tried_);
  ESP_LOGD(TAG, "Total number of invalid frames ....... %u", this->stats_.invalid_frames_);
  ESP_LOGD(TAG, "Total number of CRC errors ........... %u", this->stats_.crc_errors_);
  ESP_LOGD(TAG, "Total number of CRC errors recovered . %u", this->stats_.crc_errors_recovered_);
  ESP_LOGD(TAG, "CRC errors per session ............... %f", this->stats_.crc_errors_per_session());
  ESP_LOGD(TAG, "Number of failures ................... %u", this->stats_.failures_);
  ESP_LOGD(TAG, "============================================");
}

bool PulsarMComponent::try_lock_uart_session_() {
  if (AnyObjectLocker::try_lock(this->parent_)) {
    ESP_LOGVV(TAG, "UART bus %p locked by %s", this->parent_, this->tag_.c_str());
    return true;
  }
  ESP_LOGVV(TAG, "UART bus %p is busy", this->parent_);
  return false;
}

void PulsarMComponent::unlock_uart_session_() {
  AnyObjectLocker::unlock(this->parent_);
  ESP_LOGVV(TAG, "UART bus %p released by %s", this->parent_, this->tag_.c_str());
}

uint8_t PulsarMComponent::next_obj_id_ = 0;

std::string PulsarMComponent::generateTag() { return str_sprintf("%s%03d", TAG0, ++next_obj_id_); }

}  // namespace pulsar_m
}  // namespace esphome
