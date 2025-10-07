#pragma once

#include "esphome/core/component.h"
#include "esphome/components/ble_client/ble_client.h"

#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif

#ifdef USE_TIME
#include "esphome/components/time/real_time_clock.h"
#endif

#include <esp_gattc_api.h>

#include <array>
#include <string>
#include <vector>
#include <map>

#include "energomera_ble_sensor.h"

namespace esphome {
namespace energomera_ble {

const uint8_t VAL_NUM = 12;
using ValueRefsArray = std::array<char *, VAL_NUM>;
using SensorMap = std::multimap<std::string, EnergomeraBleSensorBase *>;

class EnergomeraBleComponent : public PollingComponent, public ble_client::BLEClientNode {
 public:
  EnergomeraBleComponent(){};

  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_BLUETOOTH; };

  void set_passkey(uint32_t passkey) { this->passkey_ = passkey % 1000000U; };

  void set_meter_address(const std::string &address) { (void) address; }
  void set_receive_timeout_ms(uint32_t timeout) { (void) timeout; }
  void set_delay_between_requests_ms(uint32_t delay) { (void) delay; }
  void set_reboot_after_failure(uint16_t value) { (void) value; }

  void register_sensor(EnergomeraBleSensorBase *sensor);

  void try_connect();
  void remove_bonding();

#ifdef USE_BINARY_SENSOR
  void set_indicator(binary_sensor::BinarySensor *indicator) { (void) indicator; }
#endif
#ifdef USE_TIME
  void set_time_source(time::RealTimeClock *time) { (void) time; }
#endif

 protected:
  void watchdog_();
  bool set_sensor_value_(EnergomeraBleSensorBase *sensor, ValueRefsArray &vals);

  char *get_nth_value_from_csv_(char *line, uint8_t idx);

  void request_firmware_version_();
  void sync_address_from_parent_();
  bool resolve_characteristics_();

  enum class FsmState : uint8_t {
    NOT_INITIALIZED = 0,
    IDLE,
    START,
    RESOLVING,
    REQUESTING_FIRMWARE,
    WAITING_FIRMWARE,
    ENABLING_NOTIFICATION,
    WAITING_NOTIFICATION_ENABLE,
    PREPARING_COMMAND,
    SENDING_COMMAND,
    WAITING_NOTIFICATION,
    READING_RESPONSE,
    GOT_RESPONSE,
    PUBLISH,
    ERROR,
    DISCONNECTED
  };

  const char *state_to_string_(FsmState state) const;

  void set_state_(FsmState state);

  void enable_notifications_();

  void prepare_request_(const std::string &request);
  void prepare_prog_frame_(const std::string &request);

  bool send_next_fragment_();
  void begin_response_reads_(uint8_t slots);
  void issue_next_response_read_();
  void handle_command_read_(const esp_ble_gattc_cb_param_t::gattc_read_char_evt_param &param);
  void finalize_command_response_();
  void handle_notification_(const esp_ble_gattc_cb_param_t::gattc_notify_evt_param &param);
  uint16_t get_max_payload_() const;

  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param) override;
  void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) override;

  bool match_service_uuid_(const esp_bt_uuid_t &uuid) const;

  // Cached handles
  uint16_t service_start_handle_{0};
  uint16_t service_end_handle_{0};
  uint16_t version_char_handle_{0};
  uint16_t tx_char_handle_{0};
  uint16_t tx_cccd_handle_{0};
  std::array<uint16_t, 15> response_char_handles_{};

  std::array<uint8_t, 6> target_address_{};
  bool address_set_{false};
  bool version_requested_{false};
  bool version_reported_{false};
  bool service_search_requested_{false};
  bool characteristics_resolved_{false};
  bool notifications_enabled_{false};
  uint8_t tx_sequence_counter_{0};
  bool tx_fragment_started_{false};
  uint16_t mtu_{23};

  std::vector<uint8_t> tx_message_remaining_;
  std::vector<uint8_t> response_buffer_;

  uint8_t expected_response_slots_{0};
  uint8_t current_response_slot_{0};
  FsmState state_{FsmState::NOT_INITIALIZED};
  bool link_encrypted_{false};
  bool pin_code_was_requested_{false};
  //  std::string pin_code_;

  uint32_t passkey_{0};

  bool services_logged_{false};

  SensorMap sensors_{};
  SensorMap::iterator request_iter{nullptr};
  SensorMap::iterator sensor_iter{nullptr};

  uint8_t get_values_from_brackets_(char *line, ValueRefsArray &vals);
};

}  // namespace energomera_ble
}  // namespace esphome
