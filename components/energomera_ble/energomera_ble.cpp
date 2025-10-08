#include "energomera_ble.h"
#include "esphome/core/log.h"

namespace esphome {
namespace energomera_ble {

static const char *const TAG = "energomera_ble";

void EnergomeraBleComponent::setup() {
  ESP_LOGI(TAG, "Setting up Energomera BLE component...");

  // Bonding сохраняем — не вызываем remove_bonding()

  // Подключаемся
  this->try_connect();
}

void EnergomeraBleComponent::loop() {
  // Watchdog FSM
  watchdog_();

  // --- 1. Автоматическое восстановление соединения ---
  static uint32_t last_reconnect_attempt = 0;
  if (this->state_ == FsmState::DISCONNECTED) {
    uint32_t now = millis();
    if (now - last_reconnect_attempt > 10000) {  // каждые 10 секунд
      ESP_LOGW(TAG, "BLE disconnected, attempting reconnect...");
      this->try_connect();
      last_reconnect_attempt = now;
    }
    return;  // пока не переподключимся, не продолжаем
  }

  // --- 2. Keep-alive запрос раз в 30 секунд ---
  static uint32_t last_keepalive = 0;
  if (this->state_ == FsmState::IDLE) {
    uint32_t now = millis();
    if (now - last_keepalive > 30000) {  // каждые 30 сек
      ESP_LOGD(TAG, "Sending keep-alive request (VER)");
      this->prepare_request_("VER");      // безопасная команда
      this->send_next_fragment_();
      last_keepalive = now;
    }
  }
}

void EnergomeraBleComponent::update() {
  ESP_LOGV(TAG, "Polling Energomera...");
  // стандартная логика опроса
}

void EnergomeraBleComponent::gattc_event_handler(esp_gattc_cb_event_t event,
                                                 esp_gatt_if_t gattc_if,
                                                 esp_ble_gattc_cb_param_t *param) {
  switch (event) {
    case ESP_GATTC_CONNECT_EVT: {
      ESP_LOGI(TAG, "Connected to Energomera, configuring connection...");

      // Увеличиваем MTU (максимум для BLE 4.2+)
      esp_ble_gattc_set_local_mtu(gattc_if, 247);

      // Настраиваем параметры соединения для большей стабильности
      esp_bd_addr_t peer_addr;
      memcpy(peer_addr, param->connect.remote_bda, sizeof(esp_bd_addr_t));
      esp_ble_gap_set_prefer_conn_params(peer_addr, 24, 40, 0, 600); // timeout 6 сек

      // Продолжаем FSM
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

    case ESP_GATTC_SEARCH_RES_EVT:
      ESP_LOGD(TAG, "Service discovered");
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
        ESP_LOGI(TAG, "BLE bonding complete, encrypted link ready");
        this->link_encrypted_ = true;
      } else {
        ESP_LOGW(TAG, "BLE authentication failed, will retry");
        this->link_encrypted_ = false;
        this->state_ = FsmState::DISCONNECTED;
      }
      break;

    default:
      break;
  }
}

void EnergomeraBleComponent::try_connect() {
  if (this->address_set_) {
    ESP_LOGI(TAG, "Trying to connect to Energomera meter...");
    ble_client::BLEClientNode::connect();
  } else {
    ESP_LOGW(TAG, "No BLE address set, cannot connect");
  }
}

void EnergomeraBleComponent::watchdog_() {
  // Следим за зависанием FSM
  static uint32_t last_activity = 0;
  if (this->state_ != FsmState::DISCONNECTED) {
    if (millis() - last_activity > 60000) {  // 1 минута бездействия
      ESP_LOGW(TAG, "No BLE activity detected, resetting connection...");
      this->state_ = FsmState::DISCONNECTED;
      last_activity = millis();
    }
  }
}

}  // namespace energomera_ble
}  // namespace esphome
