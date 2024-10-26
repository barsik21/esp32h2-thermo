#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "zigbee.h"
#include "iot_button.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "rom/gpio.h"
#include <time.h>
#include <sys/time.h>


static char manufacturer[16], model[16], firmware_version[16];
static const char *TAG = "ESP_THERMO";
bool connected = false;
bool red_led_zigbee = false;
bool green_led_zigbee = false;
bool switch1 = false;
bool time_updated = false;

#define deftemp 0
uint16_t val = 0x0;
uint16_t undefined_value = 0x8000;
// id дополнительных атрибутов
const uint16_t cust_attr_high_temp = 0x0221;
const uint16_t cust_attr_low_temp = 0x0222;
const uint16_t cust_attr_control_temp = 0x0220;
const uint16_t cust_attr_report_delay_temp = 0x0201;
int16_t temperature_high = 0;
int16_t temperature_low = 0;

int16_t temperature = 2000;
int16_t humidity = 5000;

void led_blink(int delay)
{
    gpio_set_level(LED_PIN3, true);
    vTaskDelay(pdMS_TO_TICKS(delay));
    gpio_set_level(LED_PIN3, false);
}

void read_server_time()
{
    esp_zb_zcl_read_attr_cmd_t read_req;
    read_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    uint16_t attributes[] = {ESP_ZB_ZCL_ATTR_TIME_LOCAL_TIME_ID};

    read_req.attr_number = sizeof(attributes) / sizeof(uint16_t);
    read_req.attr_field = attributes;

    read_req.clusterID = ESP_ZB_ZCL_CLUSTER_ID_TIME;

    read_req.zcl_basic_cmd.dst_endpoint = 1;               // Coordinator
    read_req.zcl_basic_cmd.src_endpoint = HA_ENDPOINT; // Device main endpoint
    read_req.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000;
    esp_zb_zcl_read_attr_cmd_req(&read_req);
}

esp_err_t writeAttribute(uint8_t endpoint, uint16_t clusterID, uint16_t attributeID, void *value)
{
    esp_zb_zcl_status_t status;
    status = esp_zb_zcl_set_attribute_val(endpoint, clusterID, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, attributeID, value, false);

    if (status != ESP_ZB_ZCL_STATUS_SUCCESS)
    {
        ESP_LOGE(TAG, "Setting attribute %04x:%04x failed(0x%02x)!", clusterID, attributeID, status);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t reportAttribute(uint8_t endpoint, uint16_t clusterID, uint16_t attributeID)
{
    esp_zb_zcl_status_t status;
    esp_zb_zcl_report_attr_cmd_t cmd = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = 0x0000,
            .dst_endpoint = endpoint,
            .src_endpoint = endpoint,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = clusterID,
        .attributeID = attributeID,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
    };
    status = esp_zb_zcl_report_attr_cmd_req(&cmd);

    if (status != ESP_ZB_ZCL_STATUS_SUCCESS)
    {
        ESP_LOGE(TAG, "Updating attribute %04x:%04x failed(0x%02x)!", clusterID, attributeID, status);
        return ESP_FAIL;
    }

    return ESP_OK;
}

///////////////////////////////// BUTTONS ///////////////////////////////////////////////////////////
static void button_single_click_cb(void *arg, void *usr_data)
{
    temperature = sht30_read_temperature_celsius(&sht30) * 100;
    humidity = sht30_read_humidity(&sht30) * 100;

    ESP_LOGI(TAG, "Temperature is: %i", temperature);
    ESP_LOGI(TAG, "Humidity is: %i", humidity);
    writeAttribute(HA_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &temperature);
    reportAttribute(HA_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID);

    // writeAttribute(HA_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, &humidity);
    // reportAttribute(HA_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID);
}

static void button_long_press_cb(void *arg, void *usr_data)
{
    ESP_LOGI("Button boot", "Long press, leave & reset");
    nvs_flash_erase_partition("nvs");
    esp_zb_factory_reset();
}

static void button_double_press_cb(void *arg, void *usr_data)
{
    ESP_LOGI("Button boot", "double press");
}

void register_button()
{
    // create gpio button
    button_config_t gpio_btn_cfg = {
        .type = BUTTON_TYPE_GPIO,
        .long_press_time = CONFIG_BUTTON_LONG_PRESS_TIME_MS,
        .short_press_time = CONFIG_BUTTON_SHORT_PRESS_TIME_MS,
        .gpio_button_config = {
            .gpio_num = 9,
            .active_level = 0,
        },
    };

    button_handle_t gpio_btn = iot_button_create(&gpio_btn_cfg);
    if (NULL == gpio_btn)
    {
        ESP_LOGE("Button boot", "Button create failed");
    }

    iot_button_register_cb(gpio_btn, BUTTON_SINGLE_CLICK, button_single_click_cb, NULL);
    iot_button_register_cb(gpio_btn, BUTTON_LONG_PRESS_START, button_long_press_cb, NULL);
    iot_button_register_cb(gpio_btn, BUTTON_DOUBLE_CLICK, button_double_press_cb, NULL);
}
///////////////////////////////// BUTTONS ///////////////////////////////////////////////////////////

void readData(void *parameter)
{
    vTaskDelay(1000 / 10);

    while (1)
    {
        sht30_fetch_data(&sht30);
        if(connected){
            esp_zb_zcl_on_off_cmd_t onoffControl;
            onoffControl.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
            onoffControl.zcl_basic_cmd.src_endpoint = HA_ENDPOINT_TWO;

            if(temperature_high*100 <= temperature){
                onoffControl.on_off_cmd_id = ESP_ZB_ZCL_CMD_ON_OFF_OFF_ID;
                ESP_LOGW("Control on/off", "Report high on");
                esp_zb_zcl_on_off_cmd_req(&onoffControl);
            }else if(temperature_low*100 >= temperature){
                onoffControl.on_off_cmd_id = ESP_ZB_ZCL_CMD_ON_OFF_ON_ID;
                ESP_LOGW("Control on/off", "Report low off");
                esp_zb_zcl_on_off_cmd_req(&onoffControl);
            }
            
            //ESP_LOGW("Control on/off", "Report");
        }
        vTaskDelay(5000 / 10);
    }
}

void temp_report_main(void *arg)
{
    
    while (1)
    {

        temperature = sht30_read_temperature_celsius(&sht30) * 100;
        humidity = sht30_read_humidity(&sht30) * 100;
        if (connected)
        {
            ESP_LOGI(TAG, "temp report loop iteration");
            writeAttribute(HA_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &temperature);
            reportAttribute(HA_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID);

            // writeAttribute(HA_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, &humidity);
            // reportAttribute(HA_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID);

            vTaskDelay((60000 * 5) / portTICK_PERIOD_MS);
        }
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

void led_task(void *arg)
{
    ESP_LOGI(TAG, "led task");

    while (1)
    {
        if (connected)
        {
            green_led_zigbee = true;
            red_led_zigbee = !connected;
            vTaskDelay(300 / portTICK_PERIOD_MS);
        }
        else
        {
            green_led_zigbee = false;
            red_led_zigbee = !red_led_zigbee;
            vTaskDelay(300 / portTICK_PERIOD_MS);
        }
        gpio_set_level(LED_PIN1, green_led_zigbee);
        gpio_set_level(LED_PIN2, red_led_zigbee);
    }
}

static void set_zcl_string(char *buffer, char *value)
{
    buffer[0] = (char)strlen(value);
    memcpy(buffer + 1, value, buffer[0]);
}

void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_zdo_signal_leave_params_t *leave_params = NULL;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    switch (sig_type)
    {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Zigbee stack initialized");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        ESP_LOGI(TAG, "signal: %d, status: %d", sig_type, err_status);
        if (err_status == ESP_OK)
        {
            ESP_LOGI(TAG, "Start network steering");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        }
        else
        {
            ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %d)", err_status);
        }
        break;

    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK)
        {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel());
            connected = true;
            read_server_time();
        }
        else
        {
            ESP_LOGI(TAG, "Network steering was not successful (status: %d)", err_status);
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
            connected = false;
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_LEAVE:
        leave_params = (esp_zb_zdo_signal_leave_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        if (leave_params->leave_type == ESP_ZB_NWK_LEAVE_TYPE_RESET)
        {
            ESP_LOGI("reset esp", "Reset device");
            connected = false;
            // esp_zb_factory_reset();
        }
        break;
    default:
        break;
    }
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);

    switch (message->info.cluster)
    {
    case ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY:
        uint8_t count = 0;
        count = *(uint8_t *)message->attribute.data.value;
        ESP_LOGI("Identify", "iden %i", *(uint8_t *)message->attribute.data.value);
        for(int i=0;i<count;i++) {
            led_blink(15); 
            vTaskDelay(300 / portTICK_PERIOD_MS);
        }
        break;
    case ESP_ZB_ZCL_CLUSTER_ID_ON_OFF:
        if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL)
        {
            switch1 = message->attribute.data.value ? *(bool *)message->attribute.data.value : switch1;
            ESP_LOGI(TAG, "Light sets to %s", switch1 ? "On" : "Off");
            gpio_set_level(LED_PIN3, switch1);
        }
        break;
    case ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT:
        switch (message->attribute.id)
        {
        case cust_attr_high_temp:
            temperature_high = *(int16_t *)message->attribute.data.value;
            ESP_LOGI(TAG, "high temp set: %i", temperature_high);
            write_NVS("high_temp", temperature_high);
            break;
        case cust_attr_low_temp:
            temperature_low = *(int16_t *)message->attribute.data.value;
            ESP_LOGI(TAG, "low temp set: %i", temperature_low);
            write_NVS("low_temp", temperature_low);
            break;
        default:
            break;
        }
        break;
    default:
        ESP_LOGI(TAG, "Received message: endpoint(0x%x), cluster(0x%i), attribute(0x%x), data size(%d)", message->info.dst_endpoint, message->info.cluster, message->attribute.id, message->attribute.data.size);
        break;
    }

    return ret;
}

static esp_err_t zb_attribute_reporting_handler(const esp_zb_zcl_report_attr_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->status);
    ESP_LOGI(TAG, "Received report from address(0x%x) src endpoint(%d) to dst endpoint(%d) cluster(0x%x)", message->src_address.u.short_addr, message->src_endpoint, message->dst_endpoint, message->cluster);

    ESP_LOGI(TAG, "Received report information: attribute(0x%x), type(0x%x), value(%d)\n", message->attribute.id, message->attribute.data.type, message->attribute.data.value ? *(uint8_t *)message->attribute.data.value : 0);
    return ESP_OK;
}

static void esp_app_zb_attribute_handler(uint16_t cluster_id, const esp_zb_zcl_attribute_t *attribute)
{
    ESP_LOGI(TAG, "attrib");
}

static esp_err_t zb_read_attr_resp_handler(const esp_zb_zcl_cmd_read_attr_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);

    ESP_LOGI(TAG, "Read attribute response: from address(0x%x) src endpoint(%d) to dst endpoint(%d) cluster(0x%x)",
             message->info.src_address.u.short_addr, message->info.src_endpoint,
             message->info.dst_endpoint, message->info.cluster);

    esp_zb_zcl_read_attr_resp_variable_t *variable = message->variables;
    while (variable)
    {
        ESP_LOGI(TAG, "Read attribute response: status(%d), cluster(0x%x), attribute(0x%x), type(0x%x), value(%d)",
                 variable->status, message->info.cluster,
                 variable->attribute.id, variable->attribute.data.type,
                 variable->attribute.data.value ? *(uint8_t *)variable->attribute.data.value : 0);
        if (variable->status == ESP_ZB_ZCL_STATUS_SUCCESS)
        {
            esp_app_zb_attribute_handler(message->info.cluster, &variable->attribute);
        }

        if (message->info.dst_endpoint == HA_ENDPOINT)
        {
            switch (message->info.cluster)
            {
            case ESP_ZB_ZCL_CLUSTER_ID_TIME:
                ESP_LOGW(__func__, "Server time recieved %lu", *(uint32_t *)variable->attribute.data.value);
                struct timeval tv;
                tv.tv_sec = *(uint32_t *)variable->attribute.data.value + 946684800;
                settimeofday(&tv, NULL);
                time_updated = true;

                uint32_t boot_time = *(uint32_t *)variable->attribute.data.value;
                ESP_LOGI(__func__, "Write new boot time %lu", boot_time);
                esp_zb_zcl_status_t state_boot_time = esp_zb_zcl_set_attribute_val(HA_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_TIME, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_TIME_LAST_SET_TIME_ID, &boot_time, false);
                if (state_boot_time != ESP_ZB_ZCL_STATUS_SUCCESS)
                {
                    ESP_LOGE(__func__, "Setting boot time attribute failed!");
                }

                break;
            default:
                ESP_LOGI(__func__, "Message data: cluster(0x%x), attribute(0x%x)  ", message->info.cluster, variable->attribute.id);
            }
        }

        variable = variable->next;
    }

    return ESP_OK;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id)
    {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        break;
    case ESP_ZB_CORE_CMD_DEFAULT_RESP_CB_ID:
        esp_zb_zcl_cmd_default_resp_message_t *resp = (esp_zb_zcl_cmd_default_resp_message_t *)(message);
        ESP_LOGI(TAG, "Received default response cluster: %x", resp->info.cluster);
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        break;
    case ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID:
        ret = zb_read_attr_resp_handler((esp_zb_zcl_cmd_read_attr_resp_message_t *)message);
        break;
    case ESP_ZB_CORE_REPORT_ATTR_CB_ID:
        ret = zb_attribute_reporting_handler((esp_zb_zcl_report_attr_message_t *)message);
        break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback, message: %s", callback_id, (char *)message);
        break;
    }
    return ret;
}

static int16_t zb_temperature_to_s16(float temp)
{
    return (int16_t)(temp * 100);
}

static float zb_s16_to_temperature(int16_t value)
{
    return 1.0 * value / 100;
}

static void esp_zb_task(void *pvParameters)
{
    /* initialize Zigbee stack with Zigbee end-device config */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZR_CONFIG();
    esp_zb_init(&zb_nwk_cfg);
    esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_t *esp_zb_cluster_list_two = esp_zb_zcl_cluster_list_create();
    //------------------------------------------ Attribute ------------------------------------------------
    esp_zb_basic_cluster_cfg_t basic_cluster_cfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = 0x04,
    };

    set_zcl_string(manufacturer, MANUFACTURER_NAME);
    set_zcl_string(model, MODEL_NAME);
    set_zcl_string(firmware_version, FIRMWARE_VERSION);

    esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_basic_cluster_create(&basic_cluster_cfg);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, manufacturer);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, model);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_SW_BUILD_ID, firmware_version);
    //------------------------------------------ Identify ------------------------------------------------
    uint8_t identyfi_id;
    identyfi_id = 0;
    esp_zb_attribute_list_t *esp_zb_identify_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY);
    esp_zb_identify_cluster_add_attr(esp_zb_identify_cluster, ESP_ZB_ZCL_CMD_IDENTIFY_IDENTIFY_ID, &identyfi_id);
    //------------------------------------------ Identify ------------------------------------------------

    esp_zb_cluster_list_add_basic_cluster(esp_zb_cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(esp_zb_cluster_list, esp_zb_identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_attribute_list_t *esp_zb_server_time_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_TIME);
    esp_zb_attribute_list_t *esp_zb_client_time_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_TIME);
    esp_zb_time_cluster_add_attr(esp_zb_client_time_cluster, ESP_ZB_ZCL_ATTR_TIME_LAST_SET_TIME_ID, &undefined_value);

    esp_zb_cluster_list_add_time_cluster(esp_zb_cluster_list, esp_zb_server_time_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    esp_zb_cluster_list_add_time_cluster(esp_zb_cluster_list, esp_zb_client_time_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    //***********************ON/OFF CLUSTER***************************
    esp_zb_on_off_cluster_cfg_t on_off_cfg;
    on_off_cfg.on_off = ESP_ZB_ZCL_ON_OFF_ON_OFF_DEFAULT_VALUE;
    esp_zb_attribute_list_t *esp_zb_on_off_cluster = esp_zb_on_off_cluster_create(&on_off_cfg);
    esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list_two, esp_zb_on_off_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    //***********************ON/OFF CLUSTER***************************

    //------------------------------------------ Temp ------------------------------------------------
    esp_zb_temperature_meas_cluster_cfg_t tempCluster = {
        .max_value = ESP_ZB_ZCL_TEMP_MEASUREMENT_MAX_MEASURED_VALUE_DEFAULT,
        .min_value = ESP_ZB_ZCL_TEMP_MEASUREMENT_MIN_MEASURED_VALUE_DEFAULT,
        .measured_value = ESP_ZB_ZCL_TEMP_MEASUREMENT_MEASURED_VALUE_DEFAULT,
    };
    esp_zb_attribute_list_t *esp_zb_temp_cluster = esp_zb_temperature_meas_cluster_create(&tempCluster);
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(esp_zb_temp_cluster, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, cust_attr_high_temp, ESP_ZB_ZCL_ATTR_TYPE_S16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &val));
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(esp_zb_temp_cluster, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, cust_attr_low_temp, ESP_ZB_ZCL_ATTR_TYPE_S16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &val));
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(esp_zb_temp_cluster, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, cust_attr_report_delay_temp, ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &val));
    esp_zb_cluster_list_add_temperature_meas_cluster(esp_zb_cluster_list, esp_zb_temp_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    //------------------------------------------ Temp ------------------------------------------------

    //------------------------------------------ Endpoint ------------------------------------------------
    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t en1 = {
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .endpoint = HA_ENDPOINT,
        .app_device_id = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID,
        .app_device_version = 0};
    esp_zb_endpoint_config_t en2 = {
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .endpoint = HA_ENDPOINT_TWO,
        .app_device_id = ESP_ZB_HA_ON_OFF_SWITCH_DEVICE_ID,
        .app_device_version = 0};
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, en1);
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list_two, en2);
    esp_zb_device_register(esp_zb_ep_list);

    //------------------------------------------ Callback ------------------------------------------------
    esp_zb_core_action_handler_register(zb_action_handler);

    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    esp_zb_set_secondary_network_channel_set(0x07FFF800);
    
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}

void app_main(void)
{

    gpio_pad_select_gpio(LED_PIN1);
    gpio_pad_select_gpio(LED_PIN2);
    gpio_pad_select_gpio(LED_PIN3);
    gpio_set_direction(LED_PIN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_PIN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_PIN3, GPIO_MODE_OUTPUT);

    setup_NVS();
    temperature_high = read_NVS("high_temp");
    temperature_low = read_NVS("low_temp");
    register_button();
    sht30_init(&sht30, I2C_PORT, SCL_PIN, SDA_PIN, SHT30_ADDRESS_B, I2C_SPEED, MAX_WAIT_TIME);
    sht30_heater_control(&sht30, Heater_Disable);
    sht30_periodic(&sht30, Repeatability_High, MPS_05);
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    temperature = sht30_read_temperature_celsius(&sht30) * 100;
    humidity = sht30_read_humidity(&sht30) * 100;
    ESP_LOGI(TAG, "Temperature is %i celsius", temperature);
    ESP_LOGI(TAG, "Humidity is %i", humidity);

    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
    xTaskCreate(readData, "ReadData", 4096, NULL, 5, NULL);
    xTaskCreate(temp_report_main, "Tempreture_report", 4096, NULL, 5, NULL);
    xTaskCreate(led_task, "Led Task", 4096, NULL, 5, NULL);
}
