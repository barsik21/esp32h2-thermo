#pragma once

#include <stdint.h>
#include <stdlib.h>
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SHT30_ADDRESS_DEF           0x44            /*!< default i2c address of SHT30 device, ADDR pin is connected to ground*/
#define SHT30_ADDRESS_B             0x45            /*!< i2c address of SHT30 device, ADDR pin is connected to VCC*/
#define MAX_WAIT_TIME               0xFFFFFFFF      /*!< max wait time allowing clock stretching mode*/

/**
 * @brief Enumeration for SHT30 commands and registers.
 */
typedef enum {

    SingleShot_CS               = 0x2C,     /*!< Single Shot mode clock stretching enable */
    SingleShot_DCS              = 0x24,     /*!< Single Shot mode clock stretching disable */

    SingleShot_RH_CS            = 0x06,     /*!< Single Shot mode clock stretching enable repeatability high */
    SingleShot_RM_CS            = 0x0D,     /*!< Single Shot mode clock stretching enable repeatability medium */
    SingleShot_RL_CS            = 0x10,     /*!< Single Shot mode clock stretching enable repeatability low */

    SingleShot_RH_DCS           = 0x00,     /*!< Single Shot mode clock stretching disable repeatability high */
    SingleShot_RM_DCS           = 0x0B,     /*!< Single Shot mode clock stretching disable repeatability medium */
    SingleShot_RL_DCS           = 0x16,     /*!< Single Shot mode clock stretching disable repeatability low */


    Periodic_05                 = 0x20,     /*!< Periodic mode 0.5 measurements per seconds */
    Periodic_1                  = 0x21,     /*!< Periodic mode 1 measurements per seconds */
    Periodic_2                  = 0x22,     /*!< Periodic mode 2 measurements per seconds */
    Periodic_4                  = 0x23,     /*!< Periodic mode 4 measurements per seconds */
    Periodic_10                 = 0x10,     /*!< Periodic mode 10 measurements per seconds */

    Periodic_RH_05              = 0x32,     /*!< Periodic mode 0.5 measurements per seconds repeatability high */
    Periodic_RM_05              = 0x24,     /*!< Periodic mode 0.5 measurements per seconds repeatability medium */
    Periodic_RL_05              = 0x2F,     /*!< Periodic mode 0.5 measurements per seconds repeatability low */

    Periodic_RH_1               = 0x30,     /*!< Periodic mode 1 measurements per seconds repeatability high */
    Periodic_RM_1               = 0x26,     /*!< Periodic mode 1 measurements per seconds repeatability medium */
    Periodic_RL_1               = 0x2D,     /*!< Periodic mode 1 measurements per seconds repeatability low */

    Periodic_RH_2               = 0x36,     /*!< Periodic mode 2 measurements per seconds repeatability high */
    Periodic_RM_2               = 0x20,     /*!< Periodic mode 2 measurements per seconds repeatability medium */
    Periodic_RL_2               = 0x2B,     /*!< Periodic mode 2 measurements per seconds repeatability low */

    Periodic_RH_4               = 0x34,     /*!< Periodic mode 4 measurements per seconds repeatability high */
    Periodic_RM_4               = 0x22,     /*!< Periodic mode 4 measurements per seconds repeatability medium */
    Periodic_RL_4               = 0x29,     /*!< Periodic mode 4 measurements per seconds repeatability low */

    Periodic_RH_10              = 0x37,     /*!< Periodic mode 10 measurements per seconds repeatability high */
    Periodic_RM_10              = 0x21,     /*!< Periodic mode 10 measurements per seconds repeatability medium */
    Periodic_RL_10              = 0x2A,     /*!< Periodic mode 10 measurements per seconds repeatability low */

    FetchCommand                = 0xE000,   /*!< Periodic mode fetch data command */
    ARTCommand                  = 0x2B32,   /*!< Periodic mode accerelated data command */
    BreakCommand                = 0x3093,   /*!< Periodic mode break command */

    SoftResetCommand            = 0x30A2,   /*!< Software reset */    

    Heater                      = 0x30,     /*!< Internal heater control */
    HeaterEnable                = 0x6D,     /*!< Enable internal heater */           
    HeaterDisable               = 0x66,     /*!< Disable internal heater */

    StatusRegister              = 0xF32D,   /*!< Read status register */
    ClrStatusRegister           = 0x3041,   /*!< Clear status register */

    CommandLength               = 2,        /*!< Command lenght, every command consists of two 8 bit values */

}sht30_command_t;

/**
 * @brief Enumeration for repeatability settings.
 */
typedef enum {

    Repeatability_High    = 1,      /*!< High setting for repeatability */
    Repeatability_Medium  = 2,      /*!< Medium setting for repeatability */
    Repeatability_Low     = 3,      /*!< Low setting for repeatability */

}sht30_repeatability_t;

/**
 * @brief Enumeration for clock stretching settings.
 */
typedef enum {

    ClockStretching_Disable     = 0,    /*!< Clock stretching enable, when data for reading is not ready sht pulls the clock line low until the data is ready */
    ClockStretching_Enable      = 1,    /*!< Clock stretching disable */

}sht30_clock_stretching_t;

/**
 * @brief Enumeration for measurements per seconds.
 */
typedef enum {

    MPS_05      = 0,        /*!< 0.5 measurements per seconds */
    MPS_1       = 1,        /*!< 1 measurements per seconds */
    MPS_2       = 2,        /*!< 2 measurements per seconds */
    MPS_4       = 3,        /*!< 4 measurements per seconds */
    MPS_10      = 4,        /*!< 10 measurements per seconds */

}sht30_measurements_per_seconds_t;

/**
 * @brief Enumeration for heater control.
 */
typedef enum {

    Heater_Disable     = 0,     /*!< Heater disable */
    Heater_Enable      = 1,     /*!< Heater enable */

}sht30_heater_t;

/**
 * @brief Enumeration for SHT30 error status.
 */
typedef enum {

    ok,                         /*!< Value indicating success (no error) */
    data_not_ready,             /*!< The sensor is measuring still */
    data_not_valid,             /*!< Error in validating data */
    init_error,                 /*!< Error in incialization of i2c */   
    error                       /*!< Error */

}sht30_status_t;

/**
 * @brief Enumeration for SHT30 error status
 */
typedef struct {

    uint16_t humidity;          /*!< Variable for storing current humidity */
    uint16_t temperature;       /*!< Variable for storing current temperature */

}sht30_t;

/**
 * @brief Inicialization of I2C and SHT struct.
 *
 * @param[in] sht30 sht30 struct handle.
 * @param[in] _i2c_port number of i2c port.
 * @param[in] _scl_io_num number of SCL pin.
 * @param[in] _sda_io_num number of SDA pin.
 * @param[in] _device_address address of the sht30 device.
 * @param[in] _scl_speed_hz clock speed of i2c communication.
 * @param[in] _scl_wait_us wait time fot timeout of communication.
 * @return
 *      - ok: Succesfull inicialization of sht30.
 *      - init_error: Unsuccesfull inicialization of sht30.
 */
sht30_status_t sht30_init(sht30_t *sht30, uint8_t _i2c_port, uint8_t _scl_io_num, uint8_t _sda_io_num,
                        uint8_t _device_address, uint16_t _scl_speed_hz, uint32_t _scl_wait_us);

/**
 * @brief Performs a single read of temperature and humidity.
 *
 * @param[in] sht30 sht30 struct handle.
 * @param[in] repeatability specifies the repeatability.
 * @param[in] clock enable or disable clock stretching.
 * @return
 *      - ok: Succesfull read of data.
 *      - error: Unsuccesfull read of data.
 *      - data_not_ready: When specifying clock stretching disable, data might not be ready yet when trying to read.
 *      - data_not_valid: Temperature or humidity CRC was not valid, probably corrupted data.
 */
sht30_status_t sht30_single_shot(sht30_t *sht30, sht30_repeatability_t repeatability, sht30_clock_stretching_t clock);

/**
 * @brief Sends a command to start periodic mode, meaning the sensor reads data automatically. For reading data from sensor call sht30_fetch_data.
 *
 * @param[in] sht30 sht30 struct handle.
 * @param[in] repeatability specifies the repeatability.
 * @param[in] mps measurements per seconds.
 * @return
 *      - ok: Succesfull start of periodic mode.
 *      - error: Unsuccesfull start od periodic mode.
 */
sht30_status_t sht30_periodic(sht30_t *sht30, sht30_repeatability_t repeatability, sht30_measurements_per_seconds_t mps);

/**
 * @brief Fetches a data from sensor when in periodic mode.
 *
 * @param[in] sht30 sht30 struct handle.
 * @return
 *      - ok: Succesfull read of data.
 *      - error: Unsuccesfull read of data.
 *      - data_not_ready: Data might not be ready yet when trying to read.
 *      - data_not_valid: Temperature or humidity CRC was not valid, probably corrupted data.
 */
sht30_status_t sht30_fetch_data(sht30_t *sht30);

/**
 * @brief Activates ART(accelerated response time).
 *
 * @param[in] sht30 sht30 struct handle.
 * @return
 *      - ok: Succesfull.
 *      - error: Unsuccesfull.
 */
sht30_status_t sht30_art(sht30_t *sht30);

/**
 * @brief Stops periodic mode of sht30 sensor.
 *
 * @param[in] sht30 sht30 struct handle.
 * @return
 *      - ok: Succesfull.
 *      - error: Unsuccesfull.
 */
sht30_status_t sht30_break(sht30_t *sht30);

/**
 * @brief Performs a reset of the device.
 *
 * @param[in] sht30 sht30 struct handle.
 * @return
 *      - ok: Succesfull.
 *      - error: Unsuccesfull.
 */
sht30_status_t sht30_soft_reset(sht30_t *sht30);

/**
 * @brief Function for controlling the internal heater.
 *
 * @param[in] sht30 sht30 struct handle.
 * @param[in] control enable/disable heater.
 * @return
 *      - ok: Succesfull.
 *      - error: Unsuccesfull.
 */
sht30_status_t sht30_heater_control(sht30_t *sht30, sht30_heater_t control);

/**
 * @brief Read status register.
 *
 * @param[in] sht30 sht30 struct handle.
 * @return
 *      - ok: Succesfull.
 *      - error: Unsuccesfull.
 *      - data_not_valid: CRC check vas not valid.
 */
sht30_status_t sht30_read_status_register(sht30_t *sht30);

/**
 * @brief Clear status register.
 *
 * @param[in] sht30 sht30 struct handle.
 * @return
 *      - ok: Succesfull.
 *      - error: Unsuccesfull.
 */
sht30_status_t sht30_clear_status_register(sht30_t *sht30);

/**
 * @brief Writes command to the sht30 device.
 *
 * @param[in] sht30 sht30 struct handle.
 * @param[in] command pointer to array holding command for sht30.
 * @return
 *      - ok: Succesfull write.
 *      - error: Unsuccesfull write.
 */
sht30_status_t sht30_write(sht30_t *sht30, uint8_t *command);

/**
 * @brief Reads data from the sht30 device.
 *
 * @param[in] sht30 sht30 struct handle.
 * @param[in] dataRec pointer to buffer for storing data.
 * @param[in] len length of receiving data.
 * @return
 *      - ok: Succesfull write.
 *      - error: Unsuccesfull write.
 *      - data_not_ready: Data was not ready for reading.              
 */
sht30_status_t sht30_read(sht30_t *sht30, uint8_t *dataRec, size_t len);

/**
 * @brief Calculates the cyclic redundancy check.
 *
 * @param[in] sht30 sht30 struct handle.
 * @param[in] data pointer to buffer containing the data from which the CRC is to be calculated.
 * @return value of CRC.
 */
uint8_t sht30_calculate_crc(sht30_t *sht30, uint8_t *data);

/**
 * @brief Gets the temperature of last data that was read from device in celsius.
 *
 * @param[in] sht30 sht30 struct handle.
 * @return float value of the temperature.
 */
float sht30_read_temperature_celsius(sht30_t *sht30);

/**
 * @brief Gets the temperature of last data that was read from device in fahreinheit.
 *
 * @param[in] sht30 sht30 struct handle.
 * @return float value of the temperature.
 */
float sht30_read_temperature_fahreinheit(sht30_t *sht30);

/**
 * @brief Gets the humidity of last data that was read from device in percentage.
 *
 * @param[in] sht30 sht30 struct handle.
 * @return float value of the humidity.
 */
float sht30_read_humidity(sht30_t *sht30);


#ifdef __cplusplus
}
#endif