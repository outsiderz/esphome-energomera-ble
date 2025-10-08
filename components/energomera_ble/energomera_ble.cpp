#include "energomera_ble.h"
#include "esphome/core/log.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace energomera_ble {

static const char *TAG = "energomera_ble";

// ------------------------ Setup / Loop / Update ------------------------

void EnergomeraBleComponent::setup() {
    ESP_LOGI(TAG, "Setting up Energomera BLE component");
    if (this->address_set_) {
        this->try_connect();
    }
}

void EnergomeraBleComponent::loop() {
    // Можно добавить watchdog или проверку соединения
    if (!this->is_connected() && this->address_set_) {
        ESP_LOGW(TAG, "BLE disconnected, reconnecting...");
        this->try_connect();
    }
}

void EnergomeraBleComponent::update() {
    // Публикуем последние значения сенсоров
    for (auto &sensor : this->sensors_) {
        if (sensor != nullptr) {
            sensor->publish_state(this->last_value_);  // last_value_ нужно заполнять из уведомлений
        }
    }
}

void EnergomeraBleComponent::dump_config() {
    ESP_LOGCONFIG(TAG, "Energomera BLE component configured");
}

// ------------------------ Sensor Registration ------------------------

void EnergomeraBleComponent::register_sensor(sensor::Sensor *sensor) {
    if (sensor != nullptr) {
        this->sensors_.push_back(sensor);
    }
}

// ------------------------ BLE Connection ------------------------

void EnergomeraBleComponent::try_connect() {
    if (this->ble_client_ != nullptr && this->address_set_) {
        ESP_LOGI(TAG, "Attempting BLE reconnect...");
        this->ble_client_->connect();
    }
}

bool EnergomeraBleComponent::is_connected() const {
    return this->ble_client_ != nullptr && this->ble_client_->connected();
}

// ------------------------ Handling Notifications ------------------------

void EnergomeraBleComponent::handle_notification_(const esp_ble_gattc_cb_param_t::gattc_notify_evt_param &param) {
    // Пример: получаем данные от счетчика
    if (param.value_len > 0) {
        this->last_value_ = atof(reinterpret_cast<const char *>(param.value));  // просто пример
        ESP_LOGD(TAG, "Notification received: %f", this->last_value_);
        this->update();
    }
}

// ------------------------ Target Address ------------------------

void EnergomeraBleComponent::set_meter_address(const std::string &address) {
    if (address.length() == 17) {  // XX:XX:XX:XX:XX:XX
        for (int i = 0; i < 6; i++) {
            this->target_address_[i] = std::stoi(address.substr(i * 3, 2), nullptr, 16);
        }
        this->address_set_ = true;
    } else {
        ESP_LOGE(TAG, "Invalid BLE address: %s", address.c_str());
    }
}

}  // namespace energomera_ble
}  // namespace esphome
