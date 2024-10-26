#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_flash.h"
#include <time.h>
#include "string.h"

#include "utils.h"
//#include "zigbee.h"
char strftime_buf[64];
void get_rtc_time()
{
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%a %H:%M:%S", &timeinfo);
}

void setup_NVS()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
        ESP_LOGI("nv","error 1");
    }
    ESP_ERROR_CHECK(err);
    nvs_handle_t my_handle;
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    ESP_LOGI(__func__, "Opening Non-Volatile Storage (NVS) handle... %s", (err != ESP_OK) ? T_STATUS_FAILED : T_STATUS_DONE);
    if (err == ESP_OK)
    {

        // Read
        int32_t restart_counter = 0; // value will default to 0, if not set yet in NVS
        err = nvs_get_i32(my_handle, "restart_counter", &restart_counter);
        ESP_LOGI(__func__, "Reading restart counter from NVS ... %s", (err != ESP_OK) ? T_STATUS_FAILED : T_STATUS_DONE);
        switch (err)
        {
        case ESP_OK:
            ESP_LOGI(__func__, "Restart counter = %" PRIu32, restart_counter);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGI(__func__, "The value is not initialized yet!");
            break;
        default:
            ESP_LOGI(__func__, "Error (%s) reading!", esp_err_to_name(err));
        }

        // Write
        restart_counter++;
        err = nvs_set_i32(my_handle, "restart_counter", restart_counter);
        ESP_LOGI(__func__, "Updating restart counter in NVS ... %s", (err != ESP_OK) ? T_STATUS_FAILED : T_STATUS_DONE);

        err = nvs_commit(my_handle);
        ESP_LOGI(__func__, "Committing updates in NVS ... %s", (err != ESP_OK) ? T_STATUS_FAILED : T_STATUS_DONE);

        nvs_close(my_handle);
    }
}

int16_t read_NVS(const char *nvs_key)
{
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);

    int16_t value = 0;
    err = nvs_get_i16(my_handle, nvs_key, &value);
    switch (err)
    {
    case ESP_OK:
        break;
    case ESP_ERR_NVS_NOT_FOUND:
        ESP_LOGE(__func__, "The value is not initialized yet! read");
        uint16_t value = 1;

        err = nvs_set_i16(my_handle, nvs_key, value);
        break;
    default:
        ESP_LOGE(__func__, "Error (%s) reading!", esp_err_to_name(err));
    }

    nvs_close(my_handle);
    if (err != ESP_OK)
    {
        return false;
    }
    return value;
}

bool write_NVS(const char *nvs_key, int value)
{
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    err = nvs_set_i32(my_handle, nvs_key, value);
    ESP_LOGI(__func__, "Write value... %s", (err != ESP_OK) ? T_STATUS_FAILED : T_STATUS_DONE);

    // Commit written value.
    // After setting any values, nvs_commit() must be called to ensure changes are written
    // to flash storage. Implementations may write to storage at other times,
    // but this is not guaranteed.
    err = nvs_commit(my_handle);
    ESP_LOGI(__func__, "Commit updates... %s", (err != ESP_OK) ? T_STATUS_FAILED : T_STATUS_DONE);

    // Close
    nvs_close(my_handle);

    if (err != ESP_OK)
    {
        return false;
    }
    return true;
}