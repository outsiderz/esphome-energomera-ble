#pragma once

#include "esphome.h"
#include "esp32_ble_client.h"
#include "energomera_ble_sensor.h"

namespace esphome {
namespace energomera_ble {

class EnergomeraBleComponent : public Component, public ESP32BLEClientNode {
 public:
  std::map<std::string, EnergomeraBleSensorBase*> sensors;

  void setup() override {
    ESP_LOGD("energomera", "Setup Energomera BLE Component");
    this->try_connect();
  }

  void loop() override {
    // Если не подключены к BLE — пытаемся переподключиться
    if (!this->connected()) {
      ESP_LOGW("energomera", "BLE not connected, trying to reconnect...");
      this->try_connect();
      return;
    }

    // Если FSM в состоянии ERROR — сброс bonding и переподключение
    if (state_ == FsmState::ERROR) {
      ESP_LOGW("energomera", "FSM in ERROR, removing bonding and reconnecting...");
      this->remove_bonding();
      this->set_state_(FsmState::IDLE);
      this->try_connect();
      return;
    }

    // Обновление всех сенсоров
    for (auto &sensor_pair : sensors) {
      if (sensor_pair.second != nullptr) {
        sensor_pair.second->update_state();
      }
    }

    // Публикация данных, если FSM в состоянии PUBLISH
    if (state_ == FsmState::PUBLISH) {
      ESP_LOGV("energomera_ble", "Publishing data");
      // здесь должен быть вызов publish() у сенсоров
      for (auto &sensor_pair : sensors) {
        sensor_pair.second->publish();
      }
      this->set_state_(FsmState::IDLE);
    }
  }

  void register_sensor(EnergomeraBleSensorBase *sensor) {
    if (sensor) {
      sensors[sensor->get_name()] = sensor;
    }
  }

  void try_connect() {
    if (!this->connected()) {
      ESP_LOGI("energomera_ble", "Initiating BLE connection...");
      this->connect();
    }
  }

  void remove_bonding() {
    ESP_LOGI("energomera_ble", "Removing BLE bonding...");
    this->remove_bond();
  }

 protected:
  enum class FsmState : uint8_t { IDLE = 0, PUBLISH, ERROR };
  FsmState state_{FsmState::IDLE};

  void set_state_(FsmState state) {
    state_ = state;
    ESP_LOGV("energomera_ble", "FSM state changed to %d", static_cast<int>(state));
  }
};

}  // namespace energomera_ble
}  // namespace esphome
