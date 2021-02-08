/*
 * bme280_if.h
 *
 *  Created on: Feb 8, 2021
 *      Author: Clayton
 */

#ifndef APPLICATION_LIB_BME280_BME280_IF_H_
#define APPLICATION_LIB_BME280_BME280_IF_H_

#include <ti/drivers/I2C.h>
#include "bme280_defs.h"
#include "bme280.h"

#ifdef __cplusplus
extern "C" {
#endif

uint8_t bme280_if_init(struct bme280_dev *dev, I2C_Handle *bus);
//void bme280_if_delay_ms(uint32_t period);
uint8_t bme280_if_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
uint8_t bme280_if_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
uint8_t bme280_if_get_all_sensor_data(struct bme280_data *data, struct bme280_dev *dev);
//uint8_t bme280_if_get_temperature(struct bme280_data *data);
//uint8_t bme280_if_get_humidity(struct bme280_data *data);
//uint8_t bme280_if_get_pressure(struct bme280_data *data);
uint8_t bme280_if_sleep_mode(struct bme280_dev *dev);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_LIB_BME280_BME280_IF_H_ */
