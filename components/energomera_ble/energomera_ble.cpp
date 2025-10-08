#include "energomera_ble.h"

#include "esphome/core/log.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sstream>

#include <esp_gatt_defs.h>
#include <esp_heap_caps.h>

#define SET_STATE(st) \
  { \
    ESP_LOGV(TAG, "State change request:"); \
    this->set_state_(st); \
  }

namespace esphome {
namespace energomera_ble {

static const char *const TAG = "energomera_ble";

static const uint8_t ENERGOMERA_SERVICE_UUID_128[16] = {0xdf, 0x14, 0xd9, 0x62, 0xd8, 0x1c, 0xc3, 0x97,
                                                        0xe2, 0x45, 0xef, 0x8b, 0x00, 0x01, 0x1b, 0xb9};
static const uint8_t ENERGOMERA_VERSION_UUID_128[16] = {0xdf, 0x14, 0xd9, 0x62, 0xd8, 0x1c, 0xc3, 0x97,
                                                        0xe2, 0x45, 0xef, 0x8b, 0x01, 0x01, 0x1b, 0xb9};
static const uint8_t ENERGOMERA_TX_UUID_128[16] = {0xdf, 0x14, 0xd9, 0x62, 0xd8, 0x1c, 0xc3, 0x97,
                                                   0xe2, 0x45, 0xef, 0x8b, 0x05, 0x01, 0x1b, 0xb9};
static const uint8_t ENERGOMERA_RESPONSE_UUIDS_128[16][16] = {
    {0xdf, 0x14, 0xd9, 0x62, 0xd8, 0x1c, 0xc3, 0x97, 0xe2, 0x45, 0xef, 0x8b, 0x06, 0x01, 0x1b, 0xb9},
    {0xdf, 0x14, 0xd9, 0x62, 0xd8, 0x1c, 0xc3, 0x97, 0xe2, 0x45, 0xef, 0x8b, 0x07, 0x01, 0x1b, 0xb9},
    {0xdf, 0x14, 0xd9, 0x62, 0xd8, 0x1c, 0xc3, 0x97, 0xe2, 0x45, 0xef, 0x8b, 0x08, 0x01, 0x1b, 0xb9},
    {0xdf, 0x14, 0xd9, 0x62, 0xd8, 0x1c, 0xc3, 0x97, 0xe2, 0x45, 0xef, 0x8b, 0x09, 0x01, 0x1b, 0xb9},
    {0xdf, 0x14, 0xd9, 0x62, 0xd8, 0x1c, 0xc3, 0x97, 0xe2, 0x45, 0xef, 0x8b, 0x0a, 0x01, 0x1b, 0xb9},
    {0xdf, 0x14, 0xd9, 0x62, 0xd8, 0x1c, 0xc3, 0x97, 0xe2, 0x45, 0xef, 0x8b, 0x0b, 0x01, 0x1b, 0xb9},
    {0xdf, 0x14, 0xd9, 0x62, 0xd8, 0x1c, 0xc3, 0x97, 0xe2, 0x45, 0xef, 0x8b, 0x0c, 0x01, 0x1b, 0xb9},
    {0xdf, 0x14, 0xd9, 0x62, 0xd8, 0x1c, 0xc3, 0x97, 0xe2, 0x45, 0xef, 0x8b, 0x0d, 0x01, 0x1b, 0xb9},
    {0xdf, 0x14, 0xd9, 0x62, 0xd8, 0x1c, 0xc3, 0x97, 0xe2, 0x45, 0xef, 0x8b, 0x0d, 0x01, 0x1b, 0xb9},
    {0xdf, 0x14, 0xd9, 0x62, 0xd8, 0x1c, 0xc3, 0x97, 0xe2, 0x45, 0xef, 0x8b, 0x0e, 0x01, 0x1b, 0xb9},
    {0xdf, 0x14, 0xd9, 0x62, 0xd8, 0x1c, 0xc3, 0x97, 0xe2, 0x45, 0xef, 0x8b, 0x0f, 0x01, 0x1b, 0xb9},
    {0xdf, 0x14, 0xd9, 0x62, 0xd8, 0x1c, 0xc3, 0x97, 0xe2, 0x45, 0xef, 0x8b, 0x10, 0x01, 0x1b, 0xb9},
    {0xdf, 0x14, 0xd9, 0x62, 0xd8, 0x1c, 0xc3, 0x97, 0xe2, 0x45, 0xef, 0x8b, 0x11, 0x01, 0x1b, 0xb9},
    {0xdf, 0x14, 0xd9, 0x62, 0xd8, 0x1c, 0xc3, 0x97, 0xe2, 0x45, 0xef, 0x8b, 0x12, 0x01, 0x1b, 0xb9},
    {0xdf, 0x14, 0xd9, 0x62, 0xd8, 0x1c, 0xc3, 0x97, 0xe2, 0x45, 0xef, 0x8b, 0x13, 0x01, 0x1b, 0xb9},
    {0xdf, 0x14, 0xd9, 0x62, 0xd8, 0x1c, 0xc3, 0x97, 0xe2, 0x45, 0xef, 0x8b, 0x14, 0x01, 0x1b, 0xb9},
};
// CEREMOTE_SERVICE_UUID  "b91b0100-8bef-45e2-97c3-1cd862d914df"
#define CEREMOTE_SERVICE_UUID \
  { 0xdf, 0x14, 0xd9, 0x62, 0xd8, 0x1c, 0xc3, 0x97, 0xe2, 0x45, 0xef, 0x8b, 0x00, 0x01, 0x1b, 0xb9 }

//   // CEREMOTE_TX_UUID  "b91b0105-8bef-45e2-97c3-1cd862d914df"  0x21
// #define CEREMOTE_TX_UUID \
//   { 0xdf, 0x14, 0xd9, 0x62, 0xd8, 0x1c, 0xc3, 0x97, 0xe2, 0x45, 0xef, 0x8b, 0x05, 0x01, 0x1b, 0xb9 }
// // CEREMOTE_RX0_UUID  "b91b0101-8bef-45e2-97c3-1cd862d914df"  0x1e
// #define CEREMOTE_RX0_UUID \
//   { 0xdf, 0x14, 0xd9, 0x62, 0xd8, 0x1c, 0xc3, 0x97, 0xe2, 0x45, 0xef, 0x8b, 0x01, 0x01, 0x1b, 0xb9 }

static char empty_str[] = "";
static char format_hex_char(uint8_t v) { return v >= 10 ? 'A' + (v - 10) : '0' + v; }

static std::string format_frame_pretty(const uint8_t *data, size_t length) {
  if (length == 0)
    return "";
  std::string ret;
  ret.resize(3 * length - 1);
  std::ostringstream ss(ret);

  for (size_t i = 0; i < length; i++) {
    switch (data[i]) {
      case 0x00:
        ss << "<NUL>";
        break;
      case 0x01:
        ss << "<SOH>";
        break;
      case 0x02:
        ss << "<STX>";
        break;
      case 0x03:
        ss << "<ETX>";
        break;
      case 0x04:
        ss << "<EOT>";
        break;
      case 0x05:
        ss << "<ENQ>";
        break;
      case 0x06:
        ss << "<ACK>";
        break;
      case 0x0d:
        ss << "<CR>";
        break;
      case 0x0a:
        ss << "<LF>";
        break;
      case 0x15:
        ss << "<NAK>";
        break;
      case 0x20:
        ss << "<SP>";
        break;
      default:
        if (data[i] <= 0x20 || data[i] >= 0x7f) {
          ss << "<" << format_hex_char((data[i] & 0xF0) >> 4) << format_hex_char(data[i] & 0x0F) << ">";
        } else {
          ss << (char) data[i];
        }
        break;
    }
  }
  if (length > 4)
    ss << " (" << length << ")";
  return ss.str();
}

static inline bool uuid_equals_128(const uint8_t *lhs, const uint8_t *rhs) { return std::memcmp(lhs, rhs, 16) == 0; }

static uint8_t apply_even_parity(uint8_t value) {
  uint8_t bit_count = 0;
  for (uint8_t bit = 0; bit < 7; bit++) {
    bit_count += (value >> bit) & 0x01;
  }
  uint8_t with_msb = value | 0x80;
  if ((bit_count & 0x01) != 0) {
    return with_msb;
  }
  return with_msb & 0x7F;
}

void EnergomeraBleComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Energomera BLE component");
  if (this->parent_ == nullptr) {
    ESP_LOGE(TAG, "BLE client parent not configured");
    this->mark_failed();
    return;
  }

  this->tx_message_remaining_.reserve(128);
  this->response_buffer_.reserve(256);

  this->request_iter = this->sensors_.begin();
  this->sensor_iter = this->sensors_.begin();

  this->sync_address_from_parent_();

  if (this->address_set_) {
    esp_ble_gattc_cache_clean(this->parent_->get_remote_bda());
  }

  ESP_LOGI(TAG, "Energomera BLE setup complete.");
  this->set_timeout(10 * 1000, [this]() {
    ESP_LOGD(TAG, "Boot timeout, component is ready to use");
    SET_STATE(FsmState::IDLE);
  });
  this->set_timeout("energomera_ble_watchdog", 25 * 1000, [this]() { this->watchdog_(); });
}

const char *ble_client_state_to_string(esphome::esp32_ble_tracker::ClientState state) {
  switch (state) {
    case esphome::esp32_ble_tracker::ClientState::INIT:
      return "INIT";
    case esphome::esp32_ble_tracker::ClientState::DISCONNECTING:
      return "DISCONNECTING";
    case esphome::esp32_ble_tracker::ClientState::IDLE:
      return "IDLE";
    case esphome::esp32_ble_tracker::ClientState::DISCOVERED:
      return "DISCOVERED";
    case esphome::esp32_ble_tracker::ClientState::CONNECTING:
      return "CONNECTING";
    case esphome::esp32_ble_tracker::ClientState::CONNECTED:
      return "CONNECTED";
    case esphome::esp32_ble_tracker::ClientState::ESTABLISHED:
      return "ESTABLISHED";
    default:
      return "UNKNOWN";
  }
}
void EnergomeraBleComponent::watchdog_() {
  static uint8_t error_states = 0;
  static uint8_t ble_connectings = 0;

  if (this->state_ == FsmState::ERROR) {
    error_states++;
  }

  auto my_ble_state = this->node_state;
  auto parent_ble_state = this->parent_->state();
  if (my_ble_state == esphome::esp32_ble_tracker::ClientState::CONNECTING) {
    ble_connectings++;
  }

  ESP_LOGW(TAG, "Watchdog triggered. FSM state is %s, My BLE state is %s, Parent BLE state is %s",
           this->state_to_string_(this->state_), ble_client_state_to_string(my_ble_state),
           ble_client_state_to_string(parent_ble_state));

  if (error_states >= 10 || ble_connectings >= 10) {
    ESP_LOGE(TAG, "Too many errors or BLE connecting states. Disconnecting.");
    delay(100);
    ble_connectings = 0;
    error_states = 0;
    this->parent_->disconnect();
    SET_STATE(FsmState::IDLE);
    // ESP.restart();
    // return;
  }

  this->set_timeout("energomera_ble_watchdog", 25 * 1000, [this]() { this->watchdog_(); });
}

void EnergomeraBleComponent::remove_bonding() {
  if (this->parent_ == nullptr) {
    ESP_LOGE(TAG, "BLE client parent not configured");
    return;
  }
  if (!this->address_set_) {
    ESP_LOGE(TAG, "BLE address not set");
  } else {
    auto status = esp_ble_remove_bond_device(this->parent_->get_remote_bda());
    ESP_LOGI(TAG, "Energomera BLE remove bond device = %d", status);
    esp_ble_gattc_cache_clean(this->parent_->get_remote_bda());
  }
  this->characteristics_resolved_ = false;
  SET_STATE(FsmState::IDLE);
}

void EnergomeraBleComponent::register_sensor(EnergomeraBleSensorBase *sensor) {
  this->sensors_.insert({sensor->get_request(), sensor});
}

void EnergomeraBleComponent::loop() {
  if (!this->address_set_)
    this->sync_address_from_parent_();

  ValueRefsArray vals;  // values from brackets, refs to this->buffers_.in
  char *in_param_ptr =
      (char *) &this->response_buffer_.data()[1];  // ref to second byte, first is STX/SOH in R1 requests

  switch (this->state_) {
    case FsmState::IDLE: {
    } break;

    case FsmState::START: {
    } break;

    case FsmState::RESOLVING: {
      SET_STATE(FsmState::REQUESTING_FIRMWARE);
    } break;

    case FsmState::REQUESTING_FIRMWARE: {
      this->request_firmware_version_();
      SET_STATE(FsmState::WAITING_FIRMWARE);
    } break;

    case FsmState::WAITING_FIRMWARE: {
    } break;

    case FsmState::ENABLING_NOTIFICATION: {
    } break;

    case FsmState::WAITING_NOTIFICATION_ENABLE: {
    } break;

    case FsmState::PREPARING_COMMAND: {
      if (this->request_iter == this->sensors_.end()) {
        ESP_LOGI(TAG, "All requests sent");
        SET_STATE(FsmState::PUBLISH);
        this->parent_->disconnect();
        break;
      }
      this->response_buffer_.clear();
      auto req = this->request_iter->first;
      this->prepare_prog_frame_(req.c_str());
      ESP_LOGI(TAG, "Sending request %s (%u bytes payload)", req.c_str(),
               (unsigned) this->tx_message_remaining_.size());
      SET_STATE(FsmState::SENDING_COMMAND);

    } break;
    case FsmState::SENDING_COMMAND: {
      if (this->send_next_fragment_()) {
      } else {
        SET_STATE(FsmState::ERROR);
      }
    } break;

    case FsmState::WAITING_NOTIFICATION: {
    } break;

    case FsmState::READING_RESPONSE: {
      // reading packets
    } break;

    case FsmState::GOT_RESPONSE:
      // full packet received
      if (!this->response_buffer_.empty()) {
        uint8_t brackets_found = get_values_from_brackets_(in_param_ptr, vals);
        if (!brackets_found) {
          ESP_LOGE(TAG, "Invalid frame format: '%s'", in_param_ptr);
          return;
        }

        ESP_LOGD(TAG,
                 "Received name: '%s', values: %d, idx: 1(%s), 2(%s), 3(%s), 4(%s), 5(%s), 6(%s), 7(%s), 8(%s), 9(%s), "
                 "10(%s), 11(%s), 12(%s)",
                 in_param_ptr, brackets_found, vals[0], vals[1], vals[2], vals[3], vals[4], vals[5], vals[6], vals[7],
                 vals[8], vals[9], vals[10], vals[11]);

        if (in_param_ptr[0] == '\0') {
          if (vals[0][0] == 'E' && vals[0][1] == 'R' && vals[0][2] == 'R') {
            ESP_LOGE(TAG, "Request '%s' either not supported or malformed. Error code %s", in_param_ptr, vals[0]);
          } else {
            ESP_LOGE(TAG, "Request '%s' either not supported or malformed.", in_param_ptr);
          }
        } else {
          auto req = this->request_iter->first;

          if (this->request_iter->second->get_function() != in_param_ptr) {
            ESP_LOGE(TAG, "Returned data name mismatch. Skipping frame");
            return;
          }

          auto range = sensors_.equal_range(req);
          for (auto it = range.first; it != range.second; ++it) {
            if (!it->second->is_failed())
              set_sensor_value_(it->second, vals);
          }
        }
      }

      this->request_iter = this->sensors_.upper_bound(this->request_iter->first);
      SET_STATE(FsmState::PREPARING_COMMAND);
      this->response_buffer_.clear();

      break;

    case FsmState::PUBLISH: {
      // publish one by one, then go IDLE

      ESP_LOGV(TAG, "Publishing data");

      if (this->sensor_iter != this->sensors_.end()) {
        this->sensor_iter->second->publish();
        this->sensor_iter++;
      } else {
        SET_STATE(FsmState::IDLE);
      }

    } break;

    case FsmState::ERROR:
      // do nothing
      break;
  }
}

uint8_t EnergomeraBleComponent::get_values_from_brackets_(char *line, ValueRefsArray &vals) {
  // line = "VOLTA(100.1)VOLTA(200.1)VOLTA(300.1)VOLTA(400.1)"
  vals.fill(empty_str);

  uint8_t idx = 0;
  bool got_param_name{false};
  char *p = line;
  while (*p && idx < VAL_NUM) {
    if (*p == '(') {
      if (!got_param_name) {
        got_param_name = true;
        *p = '\0';  // null-terminate param name
      }
      char *start = p + 1;
      char *end = strchr(start, ')');
      if (end) {
        *end = '\0';  // null-terminate value
        if (idx < VAL_NUM) {
          vals[idx++] = start;
        }
        p = end;
      }
    }
    p++;
  }
  return idx;  // at least one bracket found
}

bool char2float(const char *str, float &value) {
  char *end;
  value = strtof(str, &end);
  return *end == '\0' || *end == '*' || *end == '#';
}

// Get N-th value from comma-separated string, 1-based index
// line = "20.08.24,0.45991"
// get_nth_value_from_csv_(line, 1) -> "20.08.24"
// get_nth_value_from_csv_(line, 2) -> "0.45991"
char *EnergomeraBleComponent::get_nth_value_from_csv_(char *line, uint8_t idx) {
  if (idx == 0) {
    return line;
  }
  char *ptr;
  ptr = strtok(line, ",");
  while (ptr != nullptr) {
    if (idx-- == 1)
      return ptr;
    ptr = strtok(nullptr, ",");
  }
  return nullptr;
}

bool EnergomeraBleComponent::set_sensor_value_(EnergomeraBleSensorBase *sensor, ValueRefsArray &vals) {
  auto type = sensor->get_type();
  bool ret = true;

  uint8_t idx = sensor->get_index() - 1;
  if (idx >= VAL_NUM) {
    ESP_LOGE(TAG, "Invalid sensor index %u", idx);
    return false;
  }
  char str_buffer[128] = {'\0'};
  strncpy(str_buffer, vals[idx], 128);

  char *str = str_buffer;
  uint8_t sub_idx = sensor->get_sub_index();
  if (sub_idx == 0) {
    ESP_LOGV(TAG, "Setting value for sensor '%s', idx = %d to '%s'", sensor->get_request().c_str(), idx + 1, str);
  } else {
    ESP_LOGV(TAG, "Extracting value for sensor '%s', idx = %d, sub_idx = %d from '%s'", sensor->get_request().c_str(),
             idx + 1, sub_idx, str);
    str = this->get_nth_value_from_csv_(str, sub_idx);
    if (str == nullptr) {
      ESP_LOGE(TAG,
               "Cannot extract sensor value by sub-index %d. Is data comma-separated? Also note that sub-index starts "
               "from 1",
               sub_idx);
      str_buffer[0] = '\0';
      str = str_buffer;
    }
    ESP_LOGV(TAG, "Setting value using sub-index = %d, extracted sensor value is '%s'", sub_idx, str);
  }

#ifdef USE_SENSOR
  if (type == SensorType::SENSOR) {
    float f = 0;
    // todo: for non-energomeras... value can be "100.0" or "100.0*kWh" or "100.0#A"
    ret = str && str[0] && char2float(str, f);
    if (ret) {
      static_cast<EnergomeraBleSensor *>(sensor)->set_value(f);
    } else {
      ESP_LOGE(TAG, "Cannot convert incoming data to a number. Consider using a text sensor. Invalid data: '%s'", str);
    }
  }
#endif
#ifdef USE_TEXT_SENSOR
  if (type == SensorType::TEXT_SENSOR) {
    static_cast<EnergomeraBleTextSensor *>(sensor)->set_value(str);
  }
#endif
  return ret;
}

void EnergomeraBleComponent::try_connect() {
  if (this->state_ != FsmState::IDLE) {
    ESP_LOGW(TAG, "Not in IDLE state, can't start data collection. Current state is %s",
             this->state_to_string_(this->state_));
    return;
  }
  ESP_LOGI(TAG, "Initiating data collection from Energomera BLE device");
  this->request_iter = this->sensors_.begin();
  this->sensor_iter = this->sensors_.begin();

  auto ret = esp_ble_gatt_set_local_mtu(200);
  if (ret) {
    ESP_LOGE(TAG, "set local  MTU failed, error code = %x", ret);
  }

  /* set the security iocap & auth_req & key size & init key response key parameters to the stack*/
  esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;  // bonding with peer device after authentication
  esp_ble_io_cap_t iocap = ESP_IO_CAP_IN;  // NONE;                    // set the IO capability to No output No input
  uint8_t key_size = 16;                   // the key size should be 7~16 bytes
  uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  uint8_t oob_support = ESP_BLE_OOB_DISABLE;
  esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_OOB_SUPPORT, &oob_support, sizeof(uint8_t));

  // Enable bonding
  uint8_t bonding = 1;
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &bonding, sizeof(uint8_t));

  this->parent_->connect();
}

void EnergomeraBleComponent::update() { this->try_connect(); }

void EnergomeraBleComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Energomera BLE Component");
  if (this->address_set_) {
    ESP_LOGCONFIG(TAG, "  target address: %02X:%02X:%02X:%02X:%02X:%02X", this->target_address_[0],
                  this->target_address_[1], this->target_address_[2], this->target_address_[3],
                  this->target_address_[4], this->target_address_[5]);
  } else if (this->parent_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  target address (parent): %s", this->parent_->address_str().c_str());
  } else {
    ESP_LOGCONFIG(TAG, "  target address: not set");
  }
  ESP_LOGCONFIG(TAG, "  version requested: %s", this->version_requested_ ? "yes" : "no");
  ESP_LOGCONFIG(TAG, "  version reported: %s", this->version_reported_ ? "yes" : "no");
}

void EnergomeraBleComponent::sync_address_from_parent_() {
  if (this->address_set_ || this->parent_ == nullptr)
    return;
  uint8_t *remote = this->parent_->get_remote_bda();
  if (remote == nullptr)
    return;
  bool nonzero = false;
  for (uint8_t i = 0; i < 6; i++) {
    if (remote[i] != 0) {
      nonzero = true;
      break;
    }
  }
  if (!nonzero)
    return;
  std::memcpy(this->target_address_.data(), remote, 6);
  this->address_set_ = true;
  ESP_LOGD(TAG, "Using parent BLE address: %02X:%02X:%02X:%02X:%02X:%02X", remote[0], remote[1], remote[2], remote[3],
           remote[4], remote[5]);
}

bool EnergomeraBleComponent::match_service_uuid_(const esp_bt_uuid_t &uuid) const {
  if (uuid.len == ESP_UUID_LEN_16)
    return uuid.uuid.uuid16 == 0x0100;
  if (uuid.len == ESP_UUID_LEN_32)
    return uuid.uuid.uuid32 == 0x00000100;  // defensive
  if (uuid.len == ESP_UUID_LEN_128)
    return std::memcmp(uuid.uuid.uuid128, ENERGOMERA_SERVICE_UUID_128, sizeof(ENERGOMERA_SERVICE_UUID_128)) == 0;
  return false;
}

void EnergomeraBleComponent::request_firmware_version_() {
  if (this->parent_ == nullptr)
    return;
  if (this->version_requested_)
    return;
  if (this->service_start_handle_ == 0 || this->service_end_handle_ == 0) {
    ESP_LOGW(TAG, "Service handles not resolved, cannot read firmware version yet");
    return;
  }

  if (this->version_char_handle_ == 0) {
    ESP_LOGW(TAG, "Firmware version characteristic (0x0101) not found in service");
    return;
  }

  esp_err_t err = esp_ble_gattc_read_char(this->parent_->get_gattc_if(), this->parent_->get_conn_id(),
                                          this->version_char_handle_, ESP_GATT_AUTH_REQ_NONE);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to request firmware version read: %d", err);
    SET_STATE(FsmState::ERROR);
    return;
  }
  this->version_requested_ = true;
  SET_STATE(FsmState::WAITING_FIRMWARE);
  ESP_LOGD(TAG, "Firmware version read requested (handle 0x%04X)", this->version_char_handle_);
}

bool EnergomeraBleComponent::resolve_characteristics_() {
  if (this->characteristics_resolved_)
    return true;

  if (this->parent_ == nullptr)
    return false;

  if (this->service_start_handle_ == 0 || this->service_end_handle_ == 0) {
    ESP_LOGD(TAG, "Service handles not ready for characteristic resolution");
    return false;
  }

  std::fill(this->response_char_handles_.begin(), this->response_char_handles_.end(), 0);
  this->version_char_handle_ = 0;
  this->tx_char_handle_ = 0;
  this->tx_cccd_handle_ = 0;

  uint16_t count = 0;
  esp_gatt_status_t status = esp_ble_gattc_get_attr_count(this->parent_->get_gattc_if(), this->parent_->get_conn_id(),
                                                          ESP_GATT_DB_CHARACTERISTIC, this->service_start_handle_,
                                                          this->service_end_handle_, ESP_GATT_INVALID_HANDLE, &count);
  if (status != ESP_GATT_OK) {
    ESP_LOGW(TAG, "esp_ble_gattc_get_attr_count failed: %d", status);
  }

  if (status == ESP_GATT_OK && count > 0) {
    auto *char_elems =
        (esp_gattc_char_elem_t *) heap_caps_malloc(sizeof(esp_gattc_char_elem_t) * count, MALLOC_CAP_8BIT);
    if (char_elems != nullptr) {
      status =
          esp_ble_gattc_get_all_char(this->parent_->get_gattc_if(), this->parent_->get_conn_id(),
                                     this->service_start_handle_, this->service_end_handle_, char_elems, &count, 0);
      if (status == ESP_GATT_OK) {
        for (uint16_t i = 0; i < count; i++) {
          auto &elem = char_elems[i];
          uint16_t handle = elem.char_handle;
          if (elem.uuid.len == ESP_UUID_LEN_16) {
            ESP_LOGD(TAG, "Characteristic handle=0x%04X uuid16=0x%04X properties=0x%02X", handle, elem.uuid.uuid.uuid16,
                     elem.properties);
            if (elem.uuid.uuid.uuid16 == 0x0101) {
              this->version_char_handle_ = handle;
            }
          } else if (elem.uuid.len == ESP_UUID_LEN_128) {
            ESP_LOGV(TAG,
                     "Characteristic handle=0x%04X "
                     "uuid128=%02X%02X%02X%02X-%02X%02X-%02X%02X-%02X%02X-%02X%02X%02X%02X%02X%02X properties=0x%02X",
                     handle, elem.uuid.uuid.uuid128[15], elem.uuid.uuid.uuid128[14], elem.uuid.uuid.uuid128[13],
                     elem.uuid.uuid.uuid128[12], elem.uuid.uuid.uuid128[11], elem.uuid.uuid.uuid128[10],
                     elem.uuid.uuid.uuid128[9], elem.uuid.uuid.uuid128[8], elem.uuid.uuid.uuid128[7],
                     elem.uuid.uuid.uuid128[6], elem.uuid.uuid.uuid128[5], elem.uuid.uuid.uuid128[4],
                     elem.uuid.uuid.uuid128[3], elem.uuid.uuid.uuid128[2], elem.uuid.uuid.uuid128[1],
                     elem.uuid.uuid.uuid128[0], elem.properties);
            if (uuid_equals_128(elem.uuid.uuid.uuid128, ENERGOMERA_VERSION_UUID_128)) {
              ESP_LOGV(TAG, "Found firmware version characteristic at handle 0x%04X", handle);
              this->version_char_handle_ = handle;
            } else if (uuid_equals_128(elem.uuid.uuid.uuid128, ENERGOMERA_TX_UUID_128)) {
              ESP_LOGV(TAG, "Found TX/NOTIFY characteristic at handle 0x%04X", handle);
              this->tx_char_handle_ = handle;
              this->tx_cccd_handle_ = handle + 1;

            } else {
              for (size_t idx = 0; idx < this->response_char_handles_.size(); idx++) {
                if (uuid_equals_128(elem.uuid.uuid.uuid128, ENERGOMERA_RESPONSE_UUIDS_128[idx])) {
                  ESP_LOGV(TAG, "Found response characteristic %zu at handle 0x%04X", idx, handle);
                  this->response_char_handles_[idx] = handle;
                  break;
                }
              }
            }
          }
        }
      } else {
        ESP_LOGW(TAG, "esp_ble_gattc_get_all_char failed: %d", status);
      }
      free(char_elems);
    } else {
      ESP_LOGW(TAG, "Failed to allocate memory for characteristic enumeration");
    }
  }
  ESP_LOGD(TAG, "Characteristic resolution done. Version handle=0x%04X, TX handle=0x%04X, CCCD handle=0x%04X",
           this->version_char_handle_, this->tx_char_handle_, this->tx_cccd_handle_);
  this->characteristics_resolved_ = (this->version_char_handle_ != 0 && this->tx_char_handle_ != 0);
  return this->characteristics_resolved_;
}

void EnergomeraBleComponent::enable_notifications_() {
  if (this->notifications_enabled_)
    return;
  // if (!this->link_encrypted_) {
  //   ESP_LOGD(TAG, "Link not encrypted yet; deferring notification enable");
  //   SET_STATE(FsmState::WAITING_NOTIFICATION_ENABLE);
  //   return;
  // }
  if (this->parent_ == nullptr || this->tx_char_handle_ == 0 || this->tx_cccd_handle_ == 0) {
    ESP_LOGW(TAG, "Cannot enable notifications: required handles missing");
    SET_STATE(FsmState::ERROR);
    return;
  }

  esp_err_t status = esp_ble_gattc_register_for_notify(this->parent_->get_gattc_if(), this->parent_->get_remote_bda(),
                                                       this->tx_char_handle_);
  if (status != ESP_OK) {
    ESP_LOGW(TAG, "esp_ble_gattc_register_for_notify failed: %d (continuing)", status);
  }

  ESP_LOGV(TAG, "Notification enable request sent (handle 0x%04X)", this->tx_cccd_handle_);
  SET_STATE(FsmState::WAITING_NOTIFICATION_ENABLE);
}

void EnergomeraBleComponent::prepare_prog_frame_(const std::string &request) {
  static uint8_t command_buffer[128];

  size_t len = snprintf((char *) command_buffer, sizeof(command_buffer), "/?!\x01R1\x02%s\x03", request.c_str());
  this->tx_message_remaining_.assign(command_buffer, command_buffer + len + 1);  // include null terminator

  int checksum = 0;
  for (size_t i = 0; i < tx_message_remaining_.size() - 5; ++i) {
    checksum += tx_message_remaining_[i + 4];
  }
  tx_message_remaining_[tx_message_remaining_.size() - 1] = static_cast<uint8_t>(checksum & 0x7F);

  for (auto &byte : this->tx_message_remaining_) {
    byte = apply_even_parity(byte & 0x7F);
  }
  this->tx_fragment_started_ = false;
  this->tx_sequence_counter_ = 0;
}

void EnergomeraBleComponent::prepare_request_(const std::string &request) { this->tx_message_remaining_.clear(); }

uint16_t EnergomeraBleComponent::get_max_payload_() const {
  if (this->mtu_ <= 4)
    return 0;
  return this->mtu_ - 4;
}

bool EnergomeraBleComponent::send_next_fragment_() {
  if (this->parent_ == nullptr || this->tx_char_handle_ == 0)
    return false;
  if (this->tx_message_remaining_.empty())
    return false;

  uint16_t max_payload = this->get_max_payload_();
  if (max_payload == 0) {
    ESP_LOGW(TAG, "MTU too small to carry command data");
    return false;
  }

  bool more_after_this = this->tx_message_remaining_.size() > max_payload;
  uint16_t chunk_len = more_after_this ? max_payload : this->tx_message_remaining_.size();

  std::vector<uint8_t> packet(chunk_len + 1);
  if (!this->tx_fragment_started_) {
    this->tx_fragment_started_ = true;
    this->tx_sequence_counter_ = 0;
    if (more_after_this) {
      packet[0] = 0x00;
    } else {
      packet[0] = 0x80;
      if (this->mtu_ > 23)
        packet[0] |= 0x40;
    }
  } else {
    this->tx_sequence_counter_ = (this->tx_sequence_counter_ + 1) & 0x7F;
    packet[0] = this->tx_sequence_counter_;
    if (!more_after_this)
      packet[0] |= 0x80;
  }

  std::copy_n(this->tx_message_remaining_.begin(), chunk_len, packet.begin() + 1);

  esp_err_t status =
      esp_ble_gattc_write_char(this->parent_->get_gattc_if(), this->parent_->get_conn_id(), this->tx_char_handle_,
                               packet.size(), packet.data(), ESP_GATT_WRITE_TYPE_NO_RSP, ESP_GATT_AUTH_REQ_NONE);
  if (status != ESP_OK) {
    ESP_LOGW(TAG, "send_next_fragment_() esp_ble_gattc_write_char failed: %d", status);
    return false;
  }

  ESP_LOGV(TAG, "TX: %s", format_hex_pretty(packet.data(), packet.size()).c_str());

  this->tx_message_remaining_.erase(this->tx_message_remaining_.begin(),
                                    this->tx_message_remaining_.begin() + chunk_len);

  if (more_after_this) {
    this->parent_->run_later([this]() { this->send_next_fragment_(); });
  } else {
    SET_STATE(FsmState::WAITING_NOTIFICATION);
  }

  return true;
}

void EnergomeraBleComponent::begin_response_reads_(uint8_t slot_count) {
  uint8_t slots = slot_count + 1;
  if (slots == 0)
    slots = 1;
  if (slots > this->response_char_handles_.size())
    slots = this->response_char_handles_.size();

  this->expected_response_slots_ = slots;
  this->current_response_slot_ = 0;
  this->response_buffer_.clear();

  SET_STATE(FsmState::READING_RESPONSE);
  this->issue_next_response_read_();
}

void EnergomeraBleComponent::issue_next_response_read_() {
  if (this->current_response_slot_ >= this->expected_response_slots_) {
    this->finalize_command_response_();
    return;
  }

  uint16_t handle = this->response_char_handles_[this->current_response_slot_];
  if (handle == 0) {
    ESP_LOGW(TAG, "Response characteristic index %u not resolved", this->current_response_slot_);
    this->current_response_slot_++;
    this->issue_next_response_read_();
    return;
  }

  esp_err_t status = esp_ble_gattc_read_char(this->parent_->get_gattc_if(), this->parent_->get_conn_id(), handle,
                                             ESP_GATT_AUTH_REQ_NONE);
  if (status != ESP_OK) {
    ESP_LOGW(TAG, "Failed to request response read (handle 0x%04X): %d", handle, status);
    SET_STATE(FsmState::ERROR);
  }
}

void EnergomeraBleComponent::handle_command_read_(const esp_ble_gattc_cb_param_t::gattc_read_char_evt_param &param) {
  if (param.status != ESP_GATT_OK) {
    ESP_LOGW(TAG, "Response read failed (handle 0x%04X): %d", param.handle, param.status);
    SET_STATE(FsmState::ERROR);
    return;
  }

  for (uint16_t i = 0; i < param.value_len; i++) {
    this->response_buffer_.push_back(param.value[i] & 0x7F);
  }

  this->current_response_slot_++;
  this->issue_next_response_read_();
}

void EnergomeraBleComponent::finalize_command_response_() {
  SET_STATE(FsmState::GOT_RESPONSE);

  if (this->response_buffer_.empty()) {
    ESP_LOGW(TAG, "No response payload received");
    return;
  }

  ESP_LOGV(TAG, "RX: %s", format_frame_pretty(this->response_buffer_.data(), this->response_buffer_.size()).c_str());
  ESP_LOGVV(TAG, "RX: %s", format_hex_pretty(this->response_buffer_.data(), this->response_buffer_.size()).c_str());
}

void EnergomeraBleComponent::handle_notification_(const esp_ble_gattc_cb_param_t::gattc_notify_evt_param &param) {
  if (!param.is_notify)
    return;
  if (!param.value || param.value_len == 0) {
    ESP_LOGW(TAG, "Notification with empty payload received");
    return;
  }

  uint8_t slot_count = param.value[0];
  ESP_LOGV(TAG, "Notification received (need to read %u chunks)", slot_count);
  this->begin_response_reads_(slot_count);
}

void EnergomeraBleComponent::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                                 esp_ble_gattc_cb_param_t *param) {
  const char *event_names[] = {
      "ESP_GATTC_REG_EVT",
      "ESP_GATTC_UNREG_EVT",
      "ESP_GATTC_OPEN_EVT",
      "ESP_GATTC_READ_CHAR_EVT",
      "ESP_GATTC_WRITE_CHAR_EVT",
      "ESP_GATTC_CLOSE_EVT",
      "ESP_GATTC_SEARCH_CMPL_EVT",
      "ESP_GATTC_SEARCH_RES_EVT",
      "ESP_GATTC_READ_DESCR_EVT",
      "ESP_GATTC_WRITE_DESCR_EVT",
      "ESP_GATTC_NOTIFY_EVT",
      "ESP_GATTC_PREP_WRITE_EVT",
      "ESP_GATTC_EXEC_EVT",
      "ESP_GATTC_ACL_EVT",
      "ESP_GATTC_CANCEL_OPEN_EVT",
      "ESP_GATTC_SRVC_CHG_EVT",
      "ESP_GATTC_UNKNOWN_EVT_16",
      "ESP_GATTC_ENC_CMPL_CB_EVT",
      "ESP_GATTC_CFG_MTU_EVT",
      "ESP_GATTC_ADV_DATA_EVT",
      "ESP_GATTC_MULT_ADV_ENB_EVT",
      "ESP_GATTC_MULT_ADV_UPD_EVT",
      "ESP_GATTC_MULT_ADV_DATA_EVT",
      "ESP_GATTC_MULT_ADV_DIS_EVT",
      "ESP_GATTC_CONGEST_EVT",
      "ESP_GATTC_BTH_SCAN_ENB_EVT",
      "ESP_GATTC_BTH_SCAN_CFG_EVT",
      "ESP_GATTC_BTH_SCAN_RD_EVT",
      "ESP_GATTC_BTH_SCAN_THR_EVT",
      "ESP_GATTC_BTH_SCAN_PARAM_EVT",
      "ESP_GATTC_BTH_SCAN_DIS_EVT",
      "ESP_GATTC_SCAN_FLT_CFG_EVT",
      "ESP_GATTC_SCAN_FLT_PARAM_EVT",
      "ESP_GATTC_SCAN_FLT_STATUS_EVT",
      "ESP_GATTC_ADV_VSC_EVT",
      "ESP_GATTC_UNKNOWN_EVT_35",
      "ESP_GATTC_UNKNOWN_EVT_36",
      "ESP_GATTC_UNKNOWN_EVT_37",
      "ESP_GATTC_REG_FOR_NOTIFY_EVT",
      "ESP_GATTC_UNREG_FOR_NOTIFY_EVT",
      "ESP_GATTC_CONNECT_EVT",
      "ESP_GATTC_DISCONNECT_EVT",
      "ESP_GATTC_READ_MULTIPLE_EVT",
      "ESP_GATTC_QUEUE_FULL_EVT",
      "ESP_GATTC_SET_ASSOC_EVT",
      "ESP_GATTC_GET_ADDR_LIST_EVT",
      "ESP_GATTC_DIS_SRVC_CMPL_EVT",
      "ESP_GATTC_READ_MULTI_VAR_EVT",

  };

  const char *event_name = (event < sizeof(event_names) / sizeof(event_names[0])) ? event_names[event] : "UNKNOWN";
  ESP_LOGVV(TAG, "GATTC Event %d (%s)", event, event_name);

  switch (event) {
    case ESP_GATTC_CONNECT_EVT: {
      if (!this->parent_->check_addr(param->connect.remote_bda))
        break;
      ESP_LOGV(TAG, "ESP_GATTC_CONNECT_EVT GATT client connected");
      this->link_encrypted_ = false;
      this->service_start_handle_ = 0;
      this->service_end_handle_ = 0;
      this->version_char_handle_ = 0;
      this->tx_char_handle_ = 0;
      this->tx_cccd_handle_ = 0;
      this->version_requested_ = false;
      this->version_reported_ = false;
      this->service_search_requested_ = false;
      // this->characteristics_resolved_ = false;
      this->notifications_enabled_ = false;
      this->tx_fragment_started_ = false;
      this->tx_sequence_counter_ = 0;
      this->mtu_ = 23;
      this->tx_message_remaining_.clear();
      this->response_buffer_.clear();
      this->expected_response_slots_ = 0;
      this->current_response_slot_ = 0;
      std::fill(this->response_char_handles_.begin(), this->response_char_handles_.end(), 0);
      this->sync_address_from_parent_();

      break;
    }

    case ESP_GATTC_OPEN_EVT: {
      if (!this->parent_->check_addr(param->open.remote_bda))
        break;
      if (param->open.status != ESP_GATT_OK) {
        ESP_LOGW(TAG, "Failed to open GATT connection: status=%d", param->open.status);
        SET_STATE(FsmState::ERROR);
      } else {
        ESP_LOGV(TAG, "GATT connection open (conn_id=%d)", param->open.conn_id);
        esp_ble_gattc_send_mtu_req(this->parent_->get_gattc_if(), this->parent_->get_conn_id());
      }
      break;
    }
    case ESP_GATTC_CFG_MTU_EVT: {
      if (param->cfg_mtu.status == ESP_GATT_OK) {
        this->mtu_ = param->cfg_mtu.mtu;
        ESP_LOGV(TAG, "MTU updated to %d", param->cfg_mtu.mtu);
      } else {
        ESP_LOGW(TAG, "MTU update failed: %d", param->cfg_mtu.status);
      }
      break;
    }
    case ESP_GATTC_CLOSE_EVT: {
      if (param->search_cmpl.conn_id != this->parent_->get_conn_id())
        break;
      ESP_LOGI(TAG, "GATT connection closed (conn_id=%d)", param->close.conn_id);
      SET_STATE(FsmState::DISCONNECTED);

    } break;

    case ESP_GATTC_SEARCH_RES_EVT: {
      if (param->search_res.conn_id != this->parent_->get_conn_id())
        break;

      if (this->characteristics_resolved_) {
        // already resolved, ignore further services
        break;
      }

      if (this->match_service_uuid_(param->search_res.srvc_id.uuid)) {
        this->service_start_handle_ = param->search_res.start_handle;
        this->service_end_handle_ = param->search_res.end_handle;

        this->service_search_requested_ = false;
        ESP_LOGI(TAG, "Energomera service discovered: 0x%04X-0x%04X", this->service_start_handle_,
                 this->service_end_handle_);
      } else {
        ESP_LOGV(TAG, "Service discovered: uuid 0x%04X (len %d) handles 0x%04X-0x%04X",
                 (param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16)   ? param->search_res.srvc_id.uuid.uuid.uuid16
                 : (param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_32) ? param->search_res.srvc_id.uuid.uuid.uuid32
                                                                           : 0,
                 (param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_128) ? "(128)" : "",
                 param->search_res.srvc_id.uuid.len, param->search_res.start_handle, param->search_res.end_handle);
      }

      break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT: {
      if (param->search_cmpl.conn_id != this->parent_->get_conn_id())
        break;
      ESP_LOGV(TAG, "ESP_GATTC_SEARCH_CMPL_EVT: connected=%s, paired=%s, encrypted=%s",
               this->parent_->connected() ? "YES" : "NO", this->parent_->is_paired() ? "YES" : "NO",
               this->link_encrypted_ ? "YES" : "NO");

      if (!this->link_encrypted_) {
        ESP_LOGI(TAG, "Service discovery complete, but waiting for authentication before accessing services");
        break;
      }

      if (!this->resolve_characteristics_()) {
        SET_STATE(FsmState::ERROR);
        break;
      }

      SET_STATE(FsmState::REQUESTING_FIRMWARE);  // TODO: do we need it? maybe we can skip it
      this->set_timeout("request_fw", 100, [this]() { this->request_firmware_version_(); });
    } break;

    case ESP_GATTC_DIS_SRVC_CMPL_EVT: {
      if (param->dis_srvc_cmpl.conn_id != this->parent_->get_conn_id())
        break;
      ESP_LOGI(TAG, "Targeted service search completed (status=%d)", param->dis_srvc_cmpl.status);
      if (!this->characteristics_resolved_) {
        if (!this->link_encrypted_) {
          ESP_LOGI(TAG, "Targeted search complete, but waiting for authentication before accessing characteristics");
          break;
        } else {
          esp_bt_uuid_t filter_service_uuid = {
              .len = ESP_UUID_LEN_128,
              .uuid =
                  {
                      .uuid128 = CEREMOTE_SERVICE_UUID,
                  },
          };
          esp_ble_gattc_search_service(this->parent_->get_gattc_if(), this->parent_->get_conn_id(),
                                       &filter_service_uuid);
        }
      }
      break;
    }

    case ESP_GATTC_READ_CHAR_EVT: {
      if (param->read.conn_id != this->parent_->get_conn_id())
        break;

      if (param->read.handle == this->version_char_handle_) {
        this->version_requested_ = false;
        if (param->read.status != ESP_GATT_OK || param->read.value_len == 0) {
          ESP_LOGW(TAG, "Firmware version read failed: %d", param->read.status);
          SET_STATE(FsmState::ERROR);
          break;
        }

        ESP_LOGV(TAG, "Firmware data: %s", format_hex_pretty(param->read.value, param->read.value_len).c_str());

        this->version_reported_ = true;

        SET_STATE(FsmState::ENABLING_NOTIFICATION);
        this->set_timeout("enable_notify", 100, [this]() { this->enable_notifications_(); });
        break;
      }

      if (this->state_ == FsmState::READING_RESPONSE)
        this->handle_command_read_(param->read);
      break;
    }

    case ESP_GATTC_WRITE_CHAR_EVT: {
      if (param->write.conn_id != this->parent_->get_conn_id())
        break;
      ESP_LOGVV(TAG, "ESP_GATTC_WRITE_CHAR_EVT received (handle = 0x%04X, status=%d)", param->write.handle,
                param->write.status);
    } break;

    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
      if (param->reg_for_notify.status != ESP_GATT_OK) {
        ESP_LOGW(TAG, "Failed to register for notifications: %d", param->reg_for_notify.status);
        SET_STATE(FsmState::ERROR);
        break;
      }
      this->notifications_enabled_ = true;
      ESP_LOGD(TAG, "Registered for notifications on handle 0x%04X", param->reg_for_notify.handle);

      this->node_state = esphome::esp32_ble_tracker::ClientState::ESTABLISHED;

      uint16_t notify_en = 0x0001;
      auto err = esp_ble_gattc_write_char_descr(this->parent_->get_gattc_if(), this->parent_->get_conn_id(),
                                                this->tx_cccd_handle_, sizeof(notify_en), (uint8_t *) &notify_en,
                                                ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);

      if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to enable notifications on handle 0x%04X: %d", this->tx_cccd_handle_, err);
        SET_STATE(FsmState::ERROR);
        return;
      }

    } break;

    case ESP_GATTC_WRITE_DESCR_EVT: {
      if (param->write.conn_id != this->parent_->get_conn_id())
        break;
      ESP_LOGVV(TAG, "ESP_GATTC_WRITE_DESCR_EVT received (handle = 0x%04X, status=%d)", param->write.handle,
                param->write.status);

      if (this->state_ == FsmState::WAITING_NOTIFICATION_ENABLE) {
        if (param->write.status == ESP_GATT_OK) {
          ESP_LOGI(TAG, "Notifications enabled");
          SET_STATE(FsmState::PREPARING_COMMAND);
        } else {
          ESP_LOGW(TAG, "Failed to enable notifications: %d", param->write.status);
          SET_STATE(FsmState::ERROR);
        }
      }

      break;
    }

    case ESP_GATTC_NOTIFY_EVT: {
      if (param->notify.conn_id != this->parent_->get_conn_id())
        break;

      ESP_LOGV(TAG, "Notification received (handle = 0x%04X, )", param->notify.handle, param->notify.value_len);
      this->expected_response_slots_ = param->notify.value[0];

      if (this->state_ == FsmState::WAITING_NOTIFICATION) {
        ESP_LOGV(TAG, "Notification expected and received  ");
        this->handle_notification_(param->notify);
        break;
      }
      break;
    }

    case ESP_GATTC_DISCONNECT_EVT: {
      if (!this->parent_->check_addr(param->disconnect.remote_bda))
        break;
      ESP_LOGVV(TAG, "GATT client disconnected: reason=0x%02X", param->disconnect.reason);
      this->link_encrypted_ = false;
      this->version_requested_ = false;
      this->version_char_handle_ = 0;
      this->tx_char_handle_ = 0;
      this->tx_cccd_handle_ = 0;
      this->characteristics_resolved_ = false;
      this->notifications_enabled_ = false;
      this->tx_fragment_started_ = false;
      this->tx_sequence_counter_ = 0;
      this->mtu_ = 23;
      this->tx_message_remaining_.clear();
      this->response_buffer_.clear();
      this->expected_response_slots_ = 0;
      this->current_response_slot_ = 0;
      this->cancel_timeout("enable_notify");
      this->cancel_timeout("request_fw");
      std::fill(this->response_char_handles_.begin(), this->response_char_handles_.end(), 0);
      if (this->state_ != FsmState::PUBLISH) {
        SET_STATE(FsmState::IDLE);
      }
      break;
    }
    default:
      break;
  }
}

void EnergomeraBleComponent::gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  const char *event_names[] = {
      "ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT",
      "ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT",     /*!< When scan response data set complete, the event comes */
      "ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT",        /*!< When scan parameters set complete, the event comes */
      "ESP_GAP_BLE_SCAN_RESULT_EVT",                    /*!< When one scan result ready, the event comes each time */
      "ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT",      /*!< When raw advertising data set complete, the event comes */
      "ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT", /*!< When raw scan response data set complete, the event comes
                                                         */
      "ESP_GAP_BLE_ADV_START_COMPLETE_EVT",             /*!< When start advertising complete, the event comes */
      "ESP_GAP_BLE_SCAN_START_COMPLETE_EVT",            /*!< When start scan complete, the event comes */
      "ESP_GAP_BLE_AUTH_CMPL_EVT",                      /*!< Authentication complete indication. */
      "ESP_GAP_BLE_KEY_EVT",                            /*!< BLE  key event for peer device keys */
      "ESP_GAP_BLE_SEC_REQ_EVT",                        /*!< BLE  security request */
      "ESP_GAP_BLE_PASSKEY_NOTIF_EVT",                  /*!< passkey notification event */
      "ESP_GAP_BLE_PASSKEY_REQ_EVT",                    /*!< passkey request event */
      "ESP_GAP_BLE_OOB_REQ_EVT",                        /*!< OOB request event */
      "ESP_GAP_BLE_LOCAL_IR_EVT", /*!< BLE local IR (identity Root 128-bit random static value used to generate Long
                                     Term Key) event */
      "ESP_GAP_BLE_LOCAL_ER_EVT", /*!< BLE local ER (Encryption Root value used to generate identity resolving key)
                                     event */
      "ESP_GAP_BLE_NC_REQ_EVT",   /*!< Numeric Comparison request event */
      "ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT",          /*!< When stop adv complete, the event comes */
      "ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT",         /*!< When stop scan complete, the event comes */
      "ESP_GAP_BLE_SET_STATIC_RAND_ADDR_EVT",       /*!< When set the static rand address complete, the event comes */
      "ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT",         /*!< When update connection parameters complete, the event comes */
      "ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT",    /*!< When set pkt length complete, the event comes */
      "ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT", /*!< When  Enable/disable privacy on the local device complete, the
                                                       event comes */
      "ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT",   /*!< When remove the bond device complete, the event comes */
      "ESP_GAP_BLE_CLEAR_BOND_DEV_COMPLETE_EVT",    /*!< When clear the bond device clear complete, the event comes */
      "ESP_GAP_BLE_GET_BOND_DEV_COMPLETE_EVT",      /*!< When get the bond device list complete, the event comes */
      "ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT",         /*!< When read the rssi complete, the event comes */
      "ESP_GAP_BLE_UPDATE_WHITELIST_COMPLETE_EVT",  /*!< When add or remove whitelist complete, the event comes */

  };

  const char *event_name = (event < sizeof(event_names) / sizeof(event_names[0])) ? event_names[event] : "UNKNOWN";
  ESP_LOGVV(TAG, "GAP Event %d (%s)", event, event_name);

  switch (event) {
    case ESP_GAP_BLE_KEY_EVT: {
      // printf("BLE Key event: type=%d, size=%d\n", param->ble_security.ble_key.key_type,
      //        param->ble_security.ble_key.key_size);

      // if (param->ble_security.ble_key.key_type == ESP_LE_KEY_LTK) {
      //   printf("LTK: ");
      //   for (int i = 0; i < param->ble_security.ble_key.key_size; i++)
      //     printf("%02X ", param->ble_security.ble_key.key[i]);
      //   printf("\n");
      // }
    } break;
    case ESP_GAP_BLE_PASSKEY_NOTIF_EVT:
      if (!this->parent_->check_addr(param->ble_security.key_notif.bd_addr)) {
        ESP_LOGW(TAG, "Passkey notification for wrong device - ignoring");
        break;
      }
      ESP_LOGE(TAG, "*** Passkey notification: %06u ***", param->ble_security.key_notif.passkey);
      break;

    case ESP_GAP_BLE_PASSKEY_REQ_EVT:
      if (!this->parent_->check_addr(param->ble_security.ble_req.bd_addr)) {
        ESP_LOGW(TAG, "Passkey request for wrong device - ignoring");
        break;
      }
      this->pin_code_was_requested_ = true;
      ESP_LOGE(TAG, "*** Passkey request - supplying PIN %06u ***", this->passkey_);
      esp_ble_passkey_reply(param->ble_security.ble_req.bd_addr, true, this->passkey_);
      break;

    case ESP_GAP_BLE_SEC_REQ_EVT: {
      if (!this->parent_->check_addr(param->ble_security.ble_req.bd_addr)) {
        ESP_LOGW(TAG, "Security request for wrong device - ignoring");
        break;
      }
      auto auth_cmpl = param->ble_security.auth_cmpl;
      ESP_LOGI(TAG, "ESP_GAP_BLE_SEC_REQ_EVT success: %d, fail reason: %d, auth mode: %d", auth_cmpl.success,
               auth_cmpl.fail_reason, auth_cmpl.auth_mode);
      ESP_LOGW(TAG, "*** Security request received, responding... ***");
      esp_err_t sec_rsp = esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
      ESP_LOGI(TAG, "Security response result: %d", sec_rsp);
      break;
    }

    case ESP_GAP_BLE_NC_REQ_EVT:
      if (!this->parent_->check_addr(param->ble_security.ble_req.bd_addr)) {
        ESP_LOGW(TAG, "Numeric comparison for wrong device - ignoring");
        break;
      }
      ESP_LOGE(TAG, "*** Numeric comparison request: %06u ***", param->ble_security.key_notif.passkey);
      esp_ble_confirm_reply(param->ble_security.ble_req.bd_addr, true);
      break;

    case ESP_GAP_BLE_OOB_REQ_EVT: {
      if (!this->parent_->check_addr(param->ble_security.ble_req.bd_addr)) {
        ESP_LOGW(TAG, "OOB request for wrong device - ignoring");
        break;
      }
      ESP_LOGE(TAG, "*** OOB data request - rejecting ***");
      esp_ble_oob_req_reply(param->ble_security.ble_req.bd_addr, nullptr, 16);
      break;
    }

    case ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT:
      ESP_LOGE(TAG, "*** Bond removal completed ***");
      break;

    case ESP_GAP_BLE_CLEAR_BOND_DEV_COMPLETE_EVT:
      ESP_LOGE(TAG, "*** Bond clearing completed ***");
      break;

    case ESP_GAP_BLE_AUTH_CMPL_EVT:
      if (!this->parent_->check_addr(param->ble_security.auth_cmpl.bd_addr)) {
        break;
      }

      if (param->ble_security.auth_cmpl.success) {
        ESP_LOGVV(TAG, "*** Pairing completed successfully. Pin code was requested before ? %s ***",
                  this->pin_code_was_requested_ ? "YES" : "NO");
        this->link_encrypted_ = true;

      } else {
        ESP_LOGE(TAG, "*** Pairing FAILED, reason=%d ***", param->ble_security.auth_cmpl.fail_reason);
        this->link_encrypted_ = false;
      }
      break;
    default:
      break;
  }
}

const char *EnergomeraBleComponent::state_to_string_(FsmState state) const {
  switch (state) {
    case FsmState::NOT_INITIALIZED:
      return "NOT_INITIALIZED";
    case FsmState::IDLE:
      return "IDLE";
    case FsmState::START:
      return "START";
    case FsmState::RESOLVING:
      return "RESOLVING";
    case FsmState::REQUESTING_FIRMWARE:
      return "REQUESTING_FIRMWARE";
    case FsmState::WAITING_FIRMWARE:
      return "WAITING_FIRMWARE";
    case FsmState::ENABLING_NOTIFICATION:
      return "ENABLING_NOTIFICATION";
    case FsmState::WAITING_NOTIFICATION_ENABLE:
      return "WAITING_NOTIFICATION_ENABLE";
    case FsmState::PREPARING_COMMAND:
      return "PREPARING_COMMAND";
    case FsmState::SENDING_COMMAND:
      return "SENDING_COMMAND";
    case FsmState::WAITING_NOTIFICATION:
      return "WAITING_NOTIFICATION";
    case FsmState::READING_RESPONSE:
      return "READING_RESPONSE";
    case FsmState::GOT_RESPONSE:
      return "GOT_RESPONSE";
    case FsmState::PUBLISH:
      return "PUBLISH";
    case FsmState::ERROR:
      return "ERROR";
    case FsmState::DISCONNECTED:
      return "DISCONNECTED";
    default:
      return "UNKNOWN";
  }
}

void EnergomeraBleComponent::set_state_(FsmState state) {
  ESP_LOGV(TAG, "State change: %s -> %s", state_to_string_(this->state_), state_to_string_(state));

  this->state_ = state;
}

}  // namespace energomera_ble
}  // namespace esphome
