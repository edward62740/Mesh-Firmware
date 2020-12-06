#ifndef BME680_H
#define BME680_H
#include <stdint.h>
#include <stdbool.h>

#include "nrf_drv_twi.h"

#define BME680_CONCAT_BYTES(msb, lsb) (((uint16_t)msb << 8) | (uint16_t)lsb)

struct comp_params {

 uint16_t par_h1;
  uint16_t par_h2;
  int8_t par_h3;
  int8_t par_h4;
  int8_t par_h5;
  uint8_t par_h6;
  int8_t par_h7;
  int8_t par_gh1;
  int16_t par_gh2;
  int8_t par_gh3;
  uint16_t par_t1;
  int16_t par_t2;
  int8_t par_t3;
  uint16_t par_p1;
  int16_t par_p2;
  int8_t par_p3;
  int16_t par_p4;
  int16_t par_p5;
  int8_t par_p6;
  int8_t par_p7;
  int16_t par_p8;
  int16_t par_p9;
  uint8_t par_p10;
  uint8_t res_heat_range;
  int8_t res_heat_val;
  int8_t range_sw_err;
};

struct BME680_driver {
  bool sensor_available;
  uint16_t adc_h;    ///< RAW humidity
  uint32_t adc_t;    ///< RAW temp
  uint32_t adc_p;    ///< RAW pressure
  uint16_t adc_g;
  int32_t t_fine;   ///< calibrated temp
  uint8_t new_data;
  uint8_t heatr_stab;
  uint8_t gas_range;
  struct comp_params cp;  ///< calibration data
  int32_t  temperature; //finalised temp in 0.01° steps
  uint32_t pressure;    //finalised hum in x256 x100 hPa
  uint32_t humidity;    //finalised hum in  x1024
  uint32_t gas;
};

extern struct BME680_driver BME680;

enum BME680_MODE {
  BME680_MODE_SLEEP  = 0x00,
  BME680_MODE_FORCED = 0x01,
  BME680_MODE_NORMAL = 0x03
} BME680_Mode_t;

/** States of the module */
typedef enum
{
    BME680_RET_OK = 0,                  /**< Ok */
    BME680_NOT_SUPPORTED = 1,           /**< Feature not supported at the moment */
    BME680_INVALID = 2,                 /**< Returned data may be not valid, because of Power Down Mode or Data not ready */
    BME680_RET_NULL = 4,                /**< NULL Pointer detected */
    BME680_RET_ERROR_SELFTEST = 8,      /**< Selftest  failed */
    BME680_RET_ILLEGAL = 16,            /**< Unallowed configuration, i.e. adjusting configuration while not in sleep.*/
    BME680_RET_ERROR = 32               /**< Not otherwise specified error */
} BME680_Ret;

#define BME680REG_EAS_STATUS     0x1D
#define BME680REG_PRESS_MSB      0x1F
#define BME680REG_PRESS_LSB      0x20
#define BME680REG_PRESS_XLSB     0x21
#define BME680REG_TEMP_MSB       0x22
#define BME680REG_TEMP_LSB       0x23
#define BME680REG_TEMP_XLSB      0x24
#define BME680REG_HUM_MSB        0x25
#define BME680REG_HUM_LSB        0x26
#define BME680REG_GAS_MSB        0x2A
#define BME680REG_GAS_LSB        0x2B

#define BME680REG_IADC_HEAT_0    0x50

#define BME680REG_RES_HEAT_0     0x5A
#define BME680REG_GAS_WAIT_0     0x64
#define BME680REG_CTRL_GAS_0     0x70
#define BME680REG_CTRL_GAS_1     0x71
#define BME680REG_CTRL_HUM       0x72
#define BME680REG_CTRL_MEAS      0x74
#define BME680REG_CONFIG         0x75

#define BME680REG_ID             0xD0
#define BME680REG_RESET          0xE0
#define BME680REG_STATUS         0x73

#define BME680_ID_VALUE          0x60

#define BME680_OVERSAMPLING_SKIP 0x00
#define BME680_OVERSAMPLING_1    0x01
#define BME680_OVERSAMPLING_2    0x02
#define BME680_OVERSAMPLING_4    0x03
#define BME680_OVERSAMPLING_8    0x04
#define BME680_OVERSAMPLING_16   0x05
#define BME680_IIR_MASK          0x1C
#define BME680_IIR_OFF           0x00
#define BME680_IIR_2             0x04
#define BME680_IIR_4             0x08
#define BME680_IIR_8             0x0C
#define BME680_IIR_16            0x10

#define BME680_INTERVAL_MASK     0xE0

#define BME680_BURST_READ_LENGTH (8) // 8 bytes only for I²C
#define BME680_MAX_READ_LENGTH BME680_BURST_READ_LENGTH


/** Structure containing sensor data from all 3 sensors */
typedef struct {
  int32_t  temperature;
  uint32_t humidity;
  uint32_t pressure;
  uint32_t gas;
}BME680_data_t;

/**
 *  Initialises BME680 in sleep mode, all sensors enabled
 */
void BME680_init(const nrf_drv_twi_t *l_twi);

void BME680_deinit();

/**
 * triggers measurements on the device, wait for them to complete
 * then reads the data from the sensor
 * All together, temp, hum, press with x1 sample
 */
void BME680_measure(bool gas_calib, uint32_t calib_value);

/**
 * dumps config registers 0xF2, 0xF3, 0xF4, 0xF5
 * ctrl_hum, status, ctrl_meas, config
 */


int  BME680_is_measuring(void);

/**
 *  Read measurements from BME680 to nRF52.
 *  You have to call this manually, in normal mode you get latest stored values.
 *  Forced mode can be used as:
 *  bme_set_mode(BME680_MODE_FORCED)
 *  while(BME680_is_measuroing());
 *  BME680_read_measurements();
 */
void BME680_read_measurements();

/**
 * Returns temperature in DegC, resolution is 0.01 DegC.
 * Output value of “2134” equals 21.34 DegC.
 */
int32_t  BME680_get_temperature(void);

 /**
 * Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format
 * (24 integer bits and 8 fractional bits).
 * Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
 */
uint32_t BME680_get_pressure(void);

/**
 * Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format
 * (22 integer and 10 fractional bits).
 * Output value of “50532” represents 50532/1024 = 49.356 %RH
 */
uint32_t   BME680_get_humidity(void);

uint32_t BME680_get_gas(void);

#endif
