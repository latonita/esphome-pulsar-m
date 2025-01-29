#include "esphome/core/application.h"
#include "esphome/core/log.h"
#include "pulsar_m.h"
#include <bitset>
#include <cstdint>

#define STATE_BOOTUP_WAIT "Waiting to boot up"
#define STATE_METER_NOT_FOUND "Meter not found"
#define STATE_METER_FOUND "Meter found"
#define STATE_OK "OK"
#define STATE_PARTIAL_OK "Read partial data"
#define STATE_DATA_FAIL "Unable to read data"

#define MASK_GOT_DATE_TIME 0b001
#define MASK_GOT_ACTIVE_POWER 0b010
#define MASK_GOT_ENERGY 0b100

#define MESSAGE_CRC_IEC 0x0F47

namespace esphome {
namespace pulsar_m {

static const char *TAG = "Pulsar_M";

static constexpr size_t rxBufferSize = 255;
static std::array<uint8_t, rxBufferSize> rxBuffer;

constexpr uint8_t bootupWaitUpdate = 10;  // avoid communications until properly booted

enum class ReadFunction : uint8_t {
  Find = 0x00,
  Channel = 0x01,
  DateTime = 0x04,
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
  ReadFunction fn;
  uint8_t len;
};

struct FrameFooter {
  uint16_t id;
  uint16_t crc;
};

struct FrameFindReq {
  FrameHeader header;
  uint8_t zero8{0};
  FrameFooter footer{0, 0};
  constexpr FrameFindReq() : header{.address_bcd = 0xF00F0FF0, .fn = ReadFunction::Find} {};
};

struct FrameFindResp {
  uint32_t bcd;
  uint32_t address;
  uint16_t crc;
};

struct FrameDataReq {
  FrameHeader header;
  uint32_t channel_mask{0};
  FrameFooter footer;

  FrameDataReq() = delete;
  constexpr FrameDataReq(uint32_t address_bcd, ReadFunction selected_function, uint16_t id)
      : header{.address_bcd = address_bcd, .fn = selected_function, .len = sizeof(FrameDataReq)}, footer{.id = id} {}
};

struct FrameDateTimeReq {
  FrameHeader header;
  FrameFooter footer;

  FrameDateTimeReq() = delete;
  constexpr FrameDateTimeReq(uint32_t address_bcd)
      : header{.address_bcd = address_bcd, .fn = ReadFunction::DateTime, .len = sizeof(FrameDateTimeReq)}, footer{} {}
};

struct FrameParamReq {
  FrameHeader header;
  uint16_t param;
  FrameFooter footer;

  FrameParamReq() = delete;
  constexpr FrameParamReq(uint32_t address_bcd, uint16_t param, uint16_t id)
      : header{.address_bcd = address_bcd, .fn = ReadFunction::Parameter, .len = sizeof(FrameParamReq)},
        param{param},
        footer{.id = id} {}
};

/*
 YEAR – год, отсчитывается от 2000, допустимые значения [0…99].
 MONTH – месяц, допустимые значения [1…12].
 DAY – день месяца, допустимые значения [1…31].
 HOUR – час, допустимые значения [0…23].
 MINUTE – минута, допустимые значения [0…59].
 SECOND – секунда, допустимые значения [0…59]

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
  constexpr FrameDateTimeResp() : header{.fn = ReadFunction::DateTime, .len = sizeof(FrameDateTimeResp)}, footer{} {}
};
#pragma pack(0)

static_assert(sizeof(FrameDateTimeReq) == 10, "Wrong structure size: FrameDateTimeReq");
static_assert(sizeof(FrameDateTimeResp) == 16, "Wrong structure size: FrameDateTimeResp");
// there shall be double64_t and float32_t
static_assert(sizeof(double) == 8, "double size is not 8 bytes.");
static_assert(sizeof(float) == 4, "float size is not 4 bytes.");

float PulsarMComponent::get_setup_priority() const { return setup_priority::AFTER_WIFI; }

void PulsarMComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Pulsar-M:");
  LOG_UPDATE_INTERVAL(this);
  ESP_LOGCONFIG(TAG, "  Meter address requested: %u", this->requested_meter_address_);
  ESP_LOGCONFIG(TAG, "  Receive timeout: %.1fs", this->receive_timeout_ / 1e3f);
  ESP_LOGCONFIG(TAG, "  Update interval: %.1fs", this->update_interval_ / 1e3f);
  LOG_PIN("  Flow Control Pin: ", this->flow_control_pin_);
  LOG_TEXT_SENSOR("  ", "Datetime", this->datetime_);
  LOG_TEXT_SENSOR("  ", "Address", this->address_);
  LOG_TEXT_SENSOR("  ", "Serial number", this->serial_nr_);
  ESP_LOGCONFIG(TAG, "Data errors %d, proper reads %d", this->data_.readErrors, this->data_.properReads);
}

void PulsarMComponent::setup() {
  if (this->reading_state_ != nullptr) {
    this->reading_state_->publish_state(STATE_BOOTUP_WAIT);
  }

  this->set_timeout(1000, [this]() { this->fsm_state_ = State::IDLE; });
}

void PulsarMComponent::loop() {
  if (!this->is_ready())
    return;

  switch (this->fsm_state_) {
    case State::NOT_INITIALIZED:
    case State::IDLE: {
      uint32_t now = millis();
      if (now - this->data_.lastGoodRead_ms > 5 * 60 * 1000) {
        ESP_LOGE(TAG, "Rebooting due to no good reads from the meter for 5 minutes...");
        delay(1000);
        App.reboot();
      }
    } break;

    case State::FIND_PULSAR: {
      this->fsm_state_ = State::GET_DATE_TIME;

      if (this->requested_meter_address_ != 0) {
        break;
      }

      flush();
      if (!this->find_pulsar()) {
        this->fsm_state_ = State::IDLE;
      }
    } break;

    case State::GET_METER_INFO: {
      flush();
      if (this->get_meter_info()) {
        //   ESP_LOGI(TAG, "Found meter with s/n %u, we will be working with it from now on.",
        //   this->data_.networkAddress); this->data_.meterFound = true; this->requested_meter_address_ =
        //   this->data_.networkAddress;

        //   if (this->network_address_ != nullptr) {
        //     this->network_address_->publish_state(to_string(this->data_.networkAddress));
        //   }
        //   if (this->serial_nr_ != nullptr) {
        //     this->serial_nr_->publish_state(to_string(this->data_.serialNumber));
        //   }
        //   if (this->reading_state_ != nullptr) {
        //     this->reading_state_->publish_state(STATE_METER_FOUND);
        //   }
        // } else {
        //   if (this->reading_state_ != nullptr) {
        //     this->reading_state_->publish_state(STATE_METER_NOT_FOUND);
        //   }
      }
      //   if (get_energy_by_tariff()) {
      //     if (this->tariff_ != nullptr) {
      //       char tariff_str[3];
      //       tariff_str[0] = 'T';
      //       tariff_str[1] = '0' + (this->data_.energy.currentTariff & 0b11);
      //       tariff_str[2] = 0;
      //       this->tariff_->publish_state(tariff_str);
      //     }

      //     if (this->energy_total_ != nullptr) {
      //       this->energy_total_->publish_state(this->data_.energy.total);
      //     }
      //     if (this->energy_t1_ != nullptr) {
      //       this->energy_t1_->publish_state(this->data_.energy.t1);
      //     }
      //     if (this->energy_t2_ != nullptr) {
      //       this->energy_t2_->publish_state(this->data_.energy.t2);
      //     }
      //     if (this->energy_t3_ != nullptr) {
      //       this->energy_t3_->publish_state(this->data_.energy.t3);
      //     }
      //     if (this->energy_t4_ != nullptr) {
      //       this->energy_t4_->publish_state(this->data_.energy.t4);
      //     }
      //     this->data_.got |= MASK_GOT_ENERGY;
      //   }
      if (this->meter_address_bcd_ == 0) {
        ESP_LOGD(TAG, "Can't find meter address, skipping further requests");
        this->fsm_state_ = State::IDLE;
      } else {
        this->fsm_state_ = State::GET_DATE_TIME;
      }
    } break;

      //    case State::GET_METER_INFO: {
      // if (!this->data_.meterFound) {
      //   flush();
      //   if (get_meter_info()) {
      //     ESP_LOGI(TAG, "Found meter with s/n %u, we will be working with it from now on.",
      //     this->data_.networkAddress); this->data_.meterFound = true; requested_meter_address_ =
      //     this->data_.networkAddress;

      //     if (this->network_address_ != nullptr) {
      //       this->network_address_->publish_state(to_string(this->data_.networkAddress));
      //     }
      //     if (this->serial_nr_ != nullptr) {
      //       this->serial_nr_->publish_state(to_string(this->data_.serialNumber));
      //     }
      //     if (this->reading_state_ != nullptr) {
      //       this->reading_state_->publish_state(STATE_METER_FOUND);
      //     }
      //   } else {
      //     if (this->reading_state_ != nullptr) {
      //       this->reading_state_->publish_state(STATE_METER_NOT_FOUND);
      //     }
      //   }
      // }
      // this->fsm_state_ = this->data_.meterFound ? State::GET_DATE_TIME : State::PUBLISH_INFO;
      //    } break;
    case State::GET_DATE_TIME: {
      flush();
      this->get_date_time();

      this->fsm_state_ = State::GET_CHANNELS_DATA;
    } break;

    case State::GET_CHANNELS_DATA: {
      flush();
      if (this->channel_mask_) {
        uint8_t num = std::bitset<32>(this->channel_mask_).count();
        // print out channel mask in hex and number of channels
        ESP_LOGD(TAG, "Number of channels: %d (mask: 0x%08X)", num, this->channel_mask_);

        this->get_channels_data();
      }
      this->fsm_state_ = State::PUBLISH_INFO;
    } break;

    case State::PUBLISH_INFO: {
      this->data_.lastGoodRead_ms = millis();

      if (this->data_.got == 0b111) {
        this->data_.failure = false;
        this->data_.initialized = true;
        if (this->reading_state_ != nullptr) {
          this->reading_state_->publish_state(STATE_OK);
        }
        this->data_.lastGoodRead_ms = millis();
      } else {
        ESP_LOGI(TAG, "Got no or partial data %o", this->data_.got);
        this->data_.failure = true;
        if (this->reading_state_ != nullptr) {
          this->reading_state_->publish_state((this->data_.got == 0) ? STATE_DATA_FAIL : STATE_PARTIAL_OK);
        }
      }
      ESP_LOGD(TAG, "Data errors %d, proper reads %d", this->data_.readErrors, this->data_.properReads);
      this->fsm_state_ = State::IDLE;
    } break;
    default:
      break;
  }
}

void PulsarMComponent::update() {
  if (this->is_ready() && this->fsm_state_ == State::IDLE) {
    ESP_LOGV(TAG, "Update: Initiating new data collection");
    this->data_.got = 0;
    this->fsm_state_ = State::FIND_PULSAR;
  } else {
    ESP_LOGV(TAG, "Update: Component not ready yet");
  }
}

void PulsarMComponent::register_sensor(PulsarMSensor *sensor) {
  ESP_LOGD(TAG, "register_sensor() channel %d", sensor->get_channel());
  this->sensors_[sensor->get_channel()] = sensor;
  this->channel_mask_ |= 1 << (sensor->get_channel() - 1);
}

bool PulsarMComponent::find_pulsar() {
  ESP_LOGD(TAG, "find_pulsar()");

  FrameFindReq req;
  send_command((uint8_t *) &req, sizeof(FrameFindReq));

  if (!receive_proper_response(sizeof(FrameFindResp), 0))
    return false;

  FrameFindResp &res = *(FrameFindResp *) rxBuffer.data();

  uint32_t addr = 0;
  if (this->bcd4_to_uint32(res.address, &addr)) {
    ESP_LOGD(TAG, "Got reply from meter with address %u", addr);
    this->meter_address_bcd_ = res.address;
    this->requested_meter_address_ = addr;

    if (this->address_ != nullptr) {
      this->address_->publish_state(to_string(addr));
    }
  } else {
    ESP_LOGW(TAG, "get_date_time() Got reply from meter with unknown address/wrong BCD");
    return false;
  }
  return true;
}

bool PulsarMComponent::get_meter_info() {
  ESP_LOGD(TAG, "get_meter_info()");

  // FrameParamReq req_addr(this->meter_address_bcd_, (uint16_t) Parameters::NetworkAddress,
  // this->generate_request_id()); send_command((uint8_t *) &req_addr, sizeof(FrameParamReq)); uint16_t expectedSize =
  // sizeof(FrameHeader) + sizeof(uint32_t) + sizeof(FrameFooter); if (!receive_proper_response(expectedSize,
  // req_addr.footer.id))
  //   return false;

  // FrameDateTimeResp &res = *(FrameDateTimeResp *) rxBuffer.data();

  // FrameParamReq req_fw(this->meter_address_bcd_, (uint16_t) Parameters::FirmwareVersion,
  // this->generate_request_id()); send_command((uint8_t *) &req_fw, sizeof(FrameParamReq)); expectedSize =
  // sizeof(FrameHeader) + sizeof(uint64_t) + sizeof(FrameFooter); if (!receive_proper_response(expectedSize,
  // req_fw.footer.id))
  //   return false;

  return true;
}

bool PulsarMComponent::get_date_time() {
  ESP_LOGD(TAG, "get_date_time()");

  FrameDateTimeReq req(this->meter_address_bcd_ > 0 ? this->meter_address_bcd_
                                                    : this->uint32_to_bcd4(this->requested_meter_address_));
  req.footer.id = this->generate_request_id();

  send_command((uint8_t *) &req, sizeof(FrameDateTimeReq));

  if (!receive_proper_response(sizeof(FrameDateTimeResp), req.footer.id))
    return false;

  FrameDateTimeResp &res = *(FrameDateTimeResp *) rxBuffer.data();

  char buffer[20];  // Buffer for "YYYY-MM-DD HH:MM:SS"
  if (res.year == 0xFF || res.month == 0xFF || res.day == 0xFF || res.hour == 0xFF || res.minute == 0xFF ||
      res.second == 0xFF) {
    ESP_LOGW(TAG, "get_date_time() Date/time not set");
    snprintf(buffer, sizeof(buffer), "Not set");
  } else {
    uint16_t year = 2000 + res.year;
    snprintf(buffer, sizeof(buffer), "%04u-%02u-%02u %02u:%02u:%02u", year, res.month, res.day, res.hour, res.minute,
             res.second);

    ESP_LOGD(TAG, "get_date_time() Got reply from meter: %s", buffer);
  }
  
  if (this->datetime_ != nullptr) {
    this->datetime_->publish_state(buffer);
  }

  return true;
}

bool PulsarMComponent::get_channels_data() {
  ESP_LOGV(TAG, "get_channels_data()");
  uint8_t channelsNum = this->sensors_.size();  // one sensor per channel
  uint16_t expectedSizeChannels = sizeof(FrameHeader) + sizeof(double) * channelsNum + sizeof(FrameFooter);

  // ---
  ESP_LOGD(TAG, "get_channels_data() Requesting channels data from meter with address %u",
           this->requested_meter_address_);
  FrameDataReq req_data(this->meter_address_bcd_, ReadFunction::Channel, this->generate_request_id());
  req_data.channel_mask = this->channel_mask_;
  send_command((uint8_t *) &req_data, sizeof(FrameDataReq));

  if (!receive_proper_response(expectedSizeChannels, req_data.footer.id))
    return false;

  uint8_t *rxBufferPtr = rxBuffer.data() + 6;

  double *valuePtr = (double *) rxBufferPtr;

  for (const auto &[channel, sensor] : this->sensors_) {
    ESP_LOGD(TAG, "Channel %d: value %lf", sensor->get_channel(), *valuePtr);
    sensor->publish_state(*valuePtr);
    valuePtr++;
  }

  return true;
}

void PulsarMComponent::send_command(uint8_t *buffer, size_t len) {
  if (len < 10 || len > 255) {
    ESP_LOGE(TAG, "send_command() wrong len %d", len);
    return;
  }
  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(true);

  uint16_t crc = this->crc_16_ibm(buffer, len - 2);
  FrameFooter *footer = (FrameFooter *) &buffer[len - sizeof(FrameFooter)];
  footer->crc = crc;

  ESP_LOGD(TAG, "TX: %s, len: %d", format_hex_pretty(buffer, len).c_str(), len);
  write_array((const uint8_t *) buffer, len);

  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(false);
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

bool PulsarMComponent::receive_proper_response(const uint16_t expectedSize, const uint16_t frameId) {
  auto stopWaiting = millis() + this->receive_timeout_;
  uint16_t bytesRead = 0;
  int currentByte = 0;
  ESP_LOGV(TAG, "Expecting %d bytes", expectedSize);
  while ((bytesRead < expectedSize) && (millis() < stopWaiting)) {
    while (available() > 0 && bytesRead < expectedSize) {
      currentByte = read();
      if (currentByte >= 0) {
        rxBuffer[bytesRead++] = (uint8_t) currentByte;
      }
      yield();
    }
    delay(5);
    yield();
    ESP_LOGV(TAG, "Got some bytesRead %d", bytesRead);
  }
  // v
  ESP_LOGD(TAG, "Bytes expected/read %d/%d", expectedSize, bytesRead);
  // vv
  if (bytesRead > 0) {
    ESP_LOGD(TAG, "RX: %s, len: %d", format_hex_pretty((const uint8_t *) rxBuffer.data(), bytesRead).c_str(),
             bytesRead);
  }

  if (crc_16_ibm(rxBuffer.data(), bytesRead) != 0x0000) {
    ESP_LOGE(TAG, "receiveProperResponse CRC failed");
    this->data_.readErrors++;
    return false;
  }

  // ERROR
  if (bytesRead == 11 && rxBuffer[4] == 0x00) {
    ESP_LOGE(TAG, "receiveProperResponse. Device returned error code %d - %s", rxBuffer[5],
             this->error_code_to_string(rxBuffer[5]));
    return false;
  }

  if (bytesRead != expectedSize) {
    ESP_LOGE(TAG, "receiveProperResponse wrong size");
    this->data_.readErrors++;
    return false;
  };

  if (frameId != 0) {
    FrameFooter *footer = (FrameFooter *) &rxBuffer[expectedSize - sizeof(FrameFooter)];
    if (footer->id != frameId) {
      ESP_LOGE(TAG, "receiveProperResponse wrong frame id");
      this->data_.readErrors++;
      return false;
    }
  }

  this->data_.properReads++;
  return true;
}

// //----------------------------------------------------------------------------
// // convert uint32 to 4-byte BCD, each byte is 2 decimal digits, 8 digits in total
// // example: input = 04026591, output = 0x04 0x02 0x65 0x91
// //----------------------------------------------------------------------------
// void PulsarMComponent::uint32_to_bcd4(uint8_t *buffer, uint32_t value) {
//   for (int i = 3; i >= 0; --i) {
//     buffer[i] = ((value / 10) % 10) << 4 | ((value % 10));
//     value /= 100;
//   }
// }
//----------------------------------------------------------------------------
// convert uint32 to 4-byte BCD, each byte is 2 decimal digits, 8 digits in total
// example: input = 04026591, output = 0x04 0x02 0x65 0x91
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
// convert 4-byte BCD to uint32, each byte is 2 decimal digits, 8 digits in total
// example: input = 0x04 0x02 0x65 0x91, output = 04026591
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
}  // namespace pulsar_m
}  // namespace esphome