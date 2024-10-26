#include "esp_zigbee_core.h"
#include "sht30.h"
#include "utils.h"

#define MAX_CHILDREN                    10          
#define INSTALLCODE_POLICY_ENABLE       false
#define HA_ENDPOINT 1
#define HA_ENDPOINT_TWO 2
#define ESP_ZB_PRIMARY_CHANNEL_MASK     (1l << 13) 

#define LED_PIN1 10
#define LED_PIN2 11
#define LED_PIN3 12

#define MANUFACTURER_NAME               "Okoshko Production"
#define MODEL_NAME                      "Smuta"
#define FIRMWARE_VERSION                "1"

#define SCL_PIN 4
#define SDA_PIN 5
#define I2C_PORT 0
#define I2C_SPEED 10000
sht30_t sht30;

#define ESP_ZB_ZR_CONFIG()                                \
    {                                                     \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ROUTER,         \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE, \
        .nwk_cfg.zczr_cfg = {                             \
            .max_children = 10,                 \
        },                                                \
    }

#define ESP_ZB_DEFAULT_RADIO_CONFIG()                           \
    {                                                           \
        .radio_mode = ZB_RADIO_MODE_NATIVE,                        \
    }

#define ESP_ZB_DEFAULT_HOST_CONFIG()                            \
    {                                                           \
        .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,      \
    }

#define TEMPERATURE_SENSOR_CONFIG()                                       \
    {                                                                     \
        .measured_value = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_UNKNOWN, \
        .min_value = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MIN_VALUE_INVALID,  \
        .max_value = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MAX_VALUE_INVALID,  \
    }
#define HUM_SENSOR_CONFIG()                                       \
    {                                                                     \
        .measured_value = ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_DEFAULT, \
        .min_value = ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MIN_VALUE_MIN_VALUE,  \
        .max_value = ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MIN_VALUE_MAX_VALUE,  \
    }

#define ARRAY_LENTH(arr) (sizeof(arr) / sizeof(arr[0]))