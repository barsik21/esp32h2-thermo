#include "sht30.h"

static const char *TAG_SHT30 = "SHT30";

i2c_master_dev_handle_t dev_handle;


sht30_status_t sht30_init(sht30_t *sht30, uint8_t _i2c_port, uint8_t _scl_io_num, uint8_t _sda_io_num,
                        uint8_t _device_address, uint16_t _scl_speed_hz, uint32_t _scl_wait_us) 
{
    esp_err_t status;

    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = _i2c_port,
        .scl_io_num = _scl_io_num,
        .sda_io_num = _sda_io_num,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    status = i2c_new_master_bus(&i2c_mst_config, &bus_handle);
    if (status != ESP_OK) {
        ESP_LOGI(TAG_SHT30, "Error while inicializating new i2c master bus.");
        return init_error;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = _device_address,
        .scl_speed_hz = _scl_speed_hz,
        //.scl_wait_us = _scl_wait_us,
    };

    status = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
    if (status != ESP_OK) {
        ESP_LOGI(TAG_SHT30, "Error while adding i2c device on bus.");
        return init_error;
    }

    return ok;
}

sht30_status_t sht30_write(sht30_t *sht30, uint8_t *command)
{
    esp_err_t err = i2c_master_transmit(dev_handle, command, CommandLength, -1);

    if (err != ESP_OK) {
        return error;
    }

    return ok;
}

sht30_status_t sht30_read(sht30_t *sht30, uint8_t *dataRec, size_t len)
{
    esp_err_t err = i2c_master_receive(dev_handle, dataRec, len, -1);

    if (err == ESP_ERR_INVALID_STATE) {
        return data_not_ready;
    }
    else if (err != ESP_OK) {
        return error;
    }

    return ok;
}

sht30_status_t sht30_single_shot(sht30_t *sht30, sht30_repeatability_t repeatability, sht30_clock_stretching_t clock)
{
    sht30_status_t status;
    uint8_t command[2];
    uint8_t returnData[6];

    switch(clock) {
        case ClockStretching_Enable:
            command[0] = SingleShot_CS;
            switch(repeatability){
                case Repeatability_High:
                    command[1] = SingleShot_RH_CS;
                    break;
                case Repeatability_Medium:
                    command[1] = SingleShot_RM_CS;
                    break;
                case Repeatability_Low:
                    command[1] = SingleShot_RL_CS;
                    break;
            }
            break;
        case ClockStretching_Disable:
            command[0] = SingleShot_DCS;
            switch(repeatability){
                case Repeatability_High:
                    command[1] = SingleShot_RH_DCS;
                    break;
                case Repeatability_Medium:
                    command[1] = SingleShot_RM_DCS;
                    break;
                case Repeatability_Low:
                    command[1] = SingleShot_RL_DCS;
                    break;
            }
            break;
    }

    status = sht30_write(sht30, command);

    if (status != ok) {
        ESP_LOGI(TAG_SHT30, "Error writing command.\n");
        return error;
    }

    if (clock == ClockStretching_Disable) {
        vTaskDelay( 100 / portTICK_PERIOD_MS);
    }

    status = sht30_read(sht30, returnData, 6);

    if (status == data_not_ready) {
        ESP_LOGI(TAG_SHT30, "Data not ready.\n");
        return data_not_ready;
    }
    else if (status != ok) {
        ESP_LOGI(TAG_SHT30, "Error reading data.\n");
        return error;
    }

    uint8_t crc_temp = sht30_calculate_crc(sht30, returnData);

    if (crc_temp != returnData[2]) {
        ESP_LOGI(TAG_SHT30, "Temperature data not valid.\n");
        return data_not_valid;
    }

    uint8_t crc_hum = sht30_calculate_crc(sht30, &returnData[3]);

    if (crc_hum != returnData[5]) {
        ESP_LOGI(TAG_SHT30, "Humidity data not valid.\n");
        return data_not_valid;
    }

    sht30->temperature = (returnData[0] << 8) | returnData[1];
    sht30->humidity = (returnData[3] << 8) | returnData[4];

    return ok;
}

sht30_status_t sht30_periodic(sht30_t *sht30, sht30_repeatability_t repeatability, sht30_measurements_per_seconds_t mps)
{
    uint8_t command[2];

    switch(mps) {
        case MPS_05:
            command[0] = Periodic_05;
            switch(repeatability){
                case Repeatability_High:
                    command[1] = Periodic_RH_05;
                    break;
                case Repeatability_Medium:
                    command[1] = Periodic_RM_05;
                    break;
                case Repeatability_Low:
                    command[1] = Periodic_RL_05;
                    break;
            }
            break;
        case MPS_1:
            command[0] = Periodic_1;
            switch(repeatability){
                case Repeatability_High:
                    command[1] = Periodic_RH_1;
                    break;
                case Repeatability_Medium:
                    command[1] = Periodic_RM_1;
                    break;
                case Repeatability_Low:
                    command[1] = Periodic_RL_1;
                    break;
            }
            break;
        case MPS_2:
            command[0] = Periodic_2;
            switch(repeatability){
                case Repeatability_High:
                    command[1] = Periodic_RH_2;
                    break;
                case Repeatability_Medium:
                    command[1] = Periodic_RM_2;
                    break;
                case Repeatability_Low:
                    command[1] = Periodic_RL_2;
                    break;
            }
            break;
        case MPS_4:
            command[0] = Periodic_4;
            switch(repeatability){
                case Repeatability_High:
                    command[1] = Periodic_RH_4;
                    break;
                case Repeatability_Medium:
                    command[1] = Periodic_RM_4;
                    break;
                case Repeatability_Low:
                    command[1] = Periodic_RL_4;
                    break;
            }
            break;
        case MPS_10:
            command[0] = Periodic_10;
            switch(repeatability){
                case Repeatability_High:
                    command[1] = Periodic_RH_10;
                    break;
                case Repeatability_Medium:
                    command[1] = Periodic_RM_10;
                    break;
                case Repeatability_Low:
                    command[1] = Periodic_RL_10;
                    break;
            }
            break;
    }

    sht30_status_t status = sht30_write(sht30, command);

    if (status != ok) {
        ESP_LOGI(TAG_SHT30, "Error writing command.\n");
        return error;
    }

    return ok;
}

sht30_status_t sht30_fetch_data(sht30_t *sht30)
{
    uint8_t command[2] = {FetchCommand >> 8, FetchCommand & 0x00FF};
    uint8_t returnData[6];
    sht30_status_t status;

    status = sht30_write(sht30, command);

    if (status != ok) {
        ESP_LOGI(TAG_SHT30, "Error writing command.\n");
        return error;
    }

    status = sht30_read(sht30, returnData, 6);

    if (status == data_not_ready) { 
        ESP_LOGI(TAG_SHT30, "Data not ready.\n");
        return data_not_ready;
    }
    else if (status != ok) {
        ESP_LOGI(TAG_SHT30, "Error reading data.\n");
        return error;
    }

    uint8_t crc_temp = sht30_calculate_crc(sht30, returnData);

    if (crc_temp != returnData[2]) {
        ESP_LOGI(TAG_SHT30, "Temperature data not valid\n");
        return data_not_valid;
    }

    uint8_t crc_hum = sht30_calculate_crc(sht30, &returnData[3]);

    if (crc_hum != returnData[5]) {
        ESP_LOGI(TAG_SHT30, "Temperature data not valid\n");
        return data_not_valid;
    }

    sht30->temperature = (returnData[0] << 8) | returnData[1];
    sht30->humidity = (returnData[3] << 8) | returnData[4];

    return ok;
}

sht30_status_t sht30_art(sht30_t *sht30)
{
    uint8_t command[2] = {ARTCommand >> 8, ARTCommand & 0x00FF};
    sht30_status_t status;

    status = sht30_write(sht30, command);

    if (status != ok) {
        ESP_LOGI(TAG_SHT30, "Error %d\n",status);
        return status;
    }

    return ok;
}

sht30_status_t sht30_break(sht30_t *sht30)
{
    uint8_t command[2] = {BreakCommand >> 8, BreakCommand & 0x00FF};
    sht30_status_t status;

    status = sht30_write(sht30, command);

    if (status != ok) {
        ESP_LOGI(TAG_SHT30, "Error writing command.\n");
        return error;
    }

    return ok;
}

sht30_status_t sht30_soft_reset(sht30_t *sht30)
{
    uint8_t command[2] = {SoftResetCommand >> 8, SoftResetCommand & 0x00FF};
    sht30_status_t status;

    status = sht30_write(sht30, command);

    if (status != ok) {
        ESP_LOGI(TAG_SHT30, "Error writing command.\n");
        return error;
    }

    return ok;
}

sht30_status_t sht30_heater_control(sht30_t *sht30, sht30_heater_t control)
{
    uint8_t command[2] = {Heater};
    sht30_status_t status;

    switch(control) {
        case Heater_Disable:
            command[1] = HeaterDisable;
            break;
        case Heater_Enable:
            command[1] = HeaterEnable;
            break;
    }

    status = sht30_write(sht30, command);

    if (status != ok) {
        ESP_LOGI(TAG_SHT30, "Error writing command.\n");
        return error;
    }

    return ok;
}

sht30_status_t sht30_read_status_register(sht30_t *sht30)
{
    uint8_t command[2] = {StatusRegister >> 8, StatusRegister & 0x00FF};
    uint8_t returnData[3];
    sht30_status_t status;

    status = sht30_write(sht30, command);

    if (status != ok) {
        ESP_LOGI(TAG_SHT30, "Error writing command.\n");
        return error;
    }

    status = sht30_read(sht30, returnData, sizeof(returnData)/sizeof(returnData[0]));

    if (status != ok) {
        ESP_LOGI(TAG_SHT30, "Error reading data.\n");
        return error;
    }

    uint8_t crc_temp = sht30_calculate_crc(sht30, returnData);

    if (crc_temp != returnData[2]) {
        ESP_LOGI(TAG_SHT30, "Data not valid.\n");
        return data_not_valid;
    }

    ESP_LOGI(TAG_SHT30, "--------------READING STATUS REGISTER--------------");

    if (returnData[0] & (1 << 7)) {
        ESP_LOGI(TAG_SHT30, "At least one pending alert.");
    } else {
        ESP_LOGI(TAG_SHT30, "No pending alert.");
    }

    if (returnData[0] & (1 << 5)) {
        ESP_LOGI(TAG_SHT30, "Heater ON.");
    } else {
        ESP_LOGI(TAG_SHT30, "Heater OFF.");
    }

    if (returnData[0] & (1 << 3)) {
        ESP_LOGI(TAG_SHT30, "RH tracking alert.");
    } else {
        ESP_LOGI(TAG_SHT30, "No RH tracking alert.");
    }

    if (returnData[0] & (1 << 2)) {
        ESP_LOGI(TAG_SHT30, "T tracking alert.");
    } else {
        ESP_LOGI(TAG_SHT30, "No T tracking alert.");
    }

    if (returnData[1] & (1 << 4)) {
        ESP_LOGI(TAG_SHT30, "Reset detected.");
    } else {
        ESP_LOGI(TAG_SHT30, "No reset detected since last reset.");
    }

    if (returnData[1] & (1 << 1)) {
        ESP_LOGI(TAG_SHT30, "Last command not processed. It was either invalid, failed the integrated command checksum.");
    } else {
        ESP_LOGI(TAG_SHT30, "Last command executed succesfully.");
    }

    if (returnData[1] & (1 << 0)) {
        ESP_LOGI(TAG_SHT30, "Checksum of last transfer failed.");
    } else {
        ESP_LOGI(TAG_SHT30, "Checksum of last transfer was correct.");
    }

    ESP_LOGI(TAG_SHT30, "---------------------------------------------------");
                         
    return ok;
}

sht30_status_t sht30_clear_status_register(sht30_t *sht30)
{
    uint8_t command[2] = {ClrStatusRegister >> 8, ClrStatusRegister & 0x00FF};
    sht30_status_t status;

    status = sht30_write(sht30, command);

    if (status != ok) {
        ESP_LOGI(TAG_SHT30, "Error writing command.\n");
        return error;
    }

    return ok;
}

uint8_t sht30_calculate_crc(sht30_t *sht30, uint8_t *data)
{
    uint8_t crc = 0xFF;
    uint8_t polynomial = 0x31;

    for (int i = 0; i < 2; i++) {
        crc ^= data[i];

        for (int j = 0; j < 8; j++) {

            if (crc & 0x80) {
                crc = (crc << 1) ^ polynomial;
            } else {
                crc <<= 1;
            }
            crc &= 0xFF;

        }
    }

    crc ^= 0x00;

    return crc;
}

float sht30_read_temperature_celsius(sht30_t *sht30)
{
    return (-45 + 175*(((float)sht30->temperature)/((1 << 16) - 1)));
}

float sht30_read_temperature_fahreinheit(sht30_t *sht30)
{
    return (-49 + 315*(((float)sht30->temperature)/((1 << 16) - 1)));
}

float sht30_read_humidity(sht30_t *sht30)
{
    return (100*(((float)sht30->humidity)/((1 << 16) - 1)));
}