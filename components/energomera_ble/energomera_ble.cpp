#include "energomera_ble.h"
#include "esphome/core/log.h"

namespace esphome {
namespace energomera_ble {

static const char *const TAG = "energomera_ble";

void EnergomeraBleComponent::setup() {
  ESP_LOGI(TAG, "Energomera BLE setup");
  this->set_state_(FsmState::START);
}

void EnergomeraBleComponent::loop() {
  // Watchdog FSM
  this->watchdog_();

  // --- Автоматическое восстановление соединения ---
  static uint32_t last_reconnect_attempt = 0;
  if (this->state_ == FsmState::DISCONNECTED) {
    uint32_t now = millis();
    if (now - last_reconnect_attempt > 10000) {  // каждые 10 сек
      ESP_LOGW(TAG, "BLE disconnected, attempting reconnect...");
      this->try_connect();
      last_reconnect_attempt = now;
    }
    return; // пока не переподключились — ничего не делаем
  }

  // --- Keep-alive каждые 30 сек ---
  static uint32_t last_keepalive = 0;
  if (this->state_ == FsmState::IDLE) {
    uint32_t now = millis();
    if (now - last_keepalive > 30000) {
      ESP_LOGD(TAG, "Sending keep-alive request");
      this->prepare_request_("VER");  // безопасный короткий запрос
      this->send_next_fragment_();
      last_keepalive = now;
    }
  }
}

void EnergomeraBleComponent::update() {
  if (this->state_ == FsmState::IDLE) {
    ESP_LOGD(TAG, "Polling sensors...");
    for (auto &it : this->sensors_) {
      ValueRefsArray vals{};
      if (this->set_sensor_value_(it.second, vals)) {
        it.second->publish_state(atof(vals[0]));
      }
    }
  }
}

void EnergomeraBleComponent::try_connect() {
  if (this->address_set_) {
    ESP_LOGI(TAG, "Trying to connect to Energomera meter...");
    this->connect(); // ✅ Исправлено под ESPHome 2025+
  } else {
    ESP_LOGW(TAG, "No BLE address set, cannot connect");
  }
}

void EnergomeraBleComponent::remove_bonding() {
  ESP_LOGW(TAG, "Bonding removal disabled for stable reconnects");
}

void EnergomeraBleComponent::gattc_event_handler(esp_gattc_cb_event_t event,
                                                 esp_gatt_if_t gattc_if,
                                                 esp_ble_gattc_cb_param_t *param) {
  switch (event) {
    case ESP_GATTC_CONNECT_EVT: {
      ESP_LOGI(TAG, "Connected to Energomera, configuring connection...");

      // ✅ Исправлено для ESP-IDF 5.4
      esp_ble_gatt_set_local_mtu(247);

      esp_bd_addr_t peer_addr;
      memcpy(peer_addr, param->connect.remote_bda, sizeof(esp_bd_addr_t));

      esp_ble_gap_set_prefer_conn_params(peer_addr, 24, 40, 0, 600);

      this->state_ = FsmState::START;
      this->characteristics_resolved_ = false;
      this->notifications_enabled_ = false;
      this->version_requested_ = false;
      this->version_reported_ = false;
      this->service_search_requested_ = false;

      ESP_LOGI(TAG, "BLE connection established successfully");
      break;
    }

    case ESP_GATTC_DISCONNECT_EVT:
      ESP_LOGW(TAG, "Energomera BLE disconnected");
      this->state_ = FsmState::DISCONNECTED;
      break;

    case ESP_GATTC_NOTIFY_EVT:
      this->handle_notification_(param->notify);
      break;

    case ESP_GATTC_READ_CHAR_EVT:
      this->handle_command_read_(param->read);
      break;

    default:
      break;
  }
}

void EnergomeraBleComponent::gap_event_handler(esp_gap_ble_cb_event_t event,
                                               esp_ble_gap_cb_param_t *param) {
  switch (event) {
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
      if (param->ble_security.auth_cmpl.success) {
        ESP_LOGI(TAG, "BLE pairing successful");
        this->link_encrypted_ = true;
      } else {
        ESP_LOGW(TAG, "BLE pairing failed, will retry");
        this->link_encrypted_ = false;
      }
      break;
    default:
      break;
  }
}

void EnergomeraBleComponent::register_sensor(EnergomeraBleSensorBase *sensor) {
  this->sensors_.insert({sensor->get_sensor_code(), sensor});
}

void EnergomeraBleComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Energomera BLE Component:");
  ESP_LOGCONFIG(TAG, "  Number of sensors: %u", (unsigned) this->sensors_.size());
  ESP_LOGCONFIG(TAG, "  Passkey: %06u", this->passkey_);
  ESP_LOGCONFIG(TAG, "  State: %s", this->state_to_string_(this->state_));
}

void EnergomeraBleComponent::watchdog_() {
  if (this->state_ == FsmState::ERROR) {
    ESP_LOGW(TAG, "Watchdog restarting BLE state machine...");
    this->state_ = FsmState::DISCONNECTED;
  }
}

const char *EnergomeraBleComponent::state_to_string_(FsmState state) const {
  switch (state) {
    case FsmState::NOT_INITIALIZED: return "NOT_INITIALIZED";
    case FsmState::IDLE: return "IDLE";
    case FsmState::START: return "START";
    case FsmState::RESOLVING: return "RESOLVING";
    case FsmState::REQUESTING_FIRMWARE: return "REQUESTING_FIRMWARE";
    case FsmState::WAITING_FIRMWARE: return "WAITING_FIRMWARE";
    case FsmState::ENABLING_NOTIFICATION: return "ENABLING_NOTIFICATION";
    case FsmState::WAITING_NOTIFICATION_ENABLE: return "WAITING_NOTIFICATION_ENABLE";
    case FsmState::PREPARING_COMMAND: return "PREPARING_COMMAND";
    case FsmState::SENDING_COMMAND: return "SENDING_COMMAND";
    case FsmState::WAITING_NOTIFICATION: return "WAITING_NOTIFICATION";
    case FsmState::READING_RESPONSE: return "READING_RESPONSE";
    case FsmState::GOT_RESPONSE: return "GOT_RESPONSE";
    case FsmState::PUBLISH: return "PUBLISH";
    case FsmState::ERROR: return "ERROR";
    case FsmState::DISCONNECTED: return "DISCONNECTED";
    default: return "UNKNOWN";
  }
}

}  // namespace energomera_ble
}  // namespace esphome
