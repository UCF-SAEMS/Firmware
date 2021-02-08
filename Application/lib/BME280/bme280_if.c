/*
 * bme280_if.c
 *
 *  Created on: Feb 8, 2021
 *      Author: Clayton
 */

#include "bme280_if.h"

#include "unistd.h"
#include <stdint.h>
#include <stddef.h>
#include <unistd.h>
#include <ti/drivers/I2C.h>

#include <ti/sysbios/knl/Task.h>

static I2C_Transaction _i2cTransaction;
static I2C_Handle *_bus;
//I2C_Transaction _i2cTransaction;
//uint8_t _txBuffer[1];
//uint8_t _rxBuffer[2];
//uint8_t _deviceAddr;

uint8_t bme280_if_init(struct bme280_dev *dev, I2C_Handle *bus)
{
  _bus = bus;
//  if (chan > sizeof(devs))
//    return BME280_E_INVALID_LEN;
//
//  // make a pointer for easier access
//  struct bme280_dev *dev = &devs[chan];
//
//  if (chan < 8) {
//    dev->dev_id = BME280_I2C_ADDR_SEC;
//  } else {
//    dev->dev_id = BME280_I2C_ADDR_PRIM;
//  }
  uint8_t rslt = BME280_OK;

  /* Initialize ASF4 I2C Module */
//  i2c_m_sync_get_io_descriptor(&I2C_0, &I2C_SENSORS_io);
//  i2c_m_sync_enable (&I2C_0);

  dev->intf = BME280_I2C_INTF;
  dev->read = (bme280_read_fptr_t) bme280_if_i2c_read;
  dev->write = (bme280_write_fptr_t) bme280_if_i2c_write;
  dev->delay_us = (bme280_delay_us_fptr_t) usleep;
  /*Initialize BME280 */
  rslt = bme280_init(dev);

  if (rslt != BME280_OK)
    return rslt;

  /* Recommended mode of operation: Indoor navigation */
  rslt = bme280_get_sensor_settings(dev);
  dev->settings.osr_h = BME280_OVERSAMPLING_1X;
  dev->settings.osr_p = BME280_OVERSAMPLING_1X;
  dev->settings.osr_t = BME280_OVERSAMPLING_1X;
  dev->settings.filter = BME280_FILTER_COEFF_OFF;
  dev->settings.filter = BME280_FILTER_COEFF_OFF;
  rslt = bme280_set_sensor_settings((BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL), dev);
  rslt = bme280_get_sensor_settings(dev);

  return rslt;
}

uint8_t bme280_if_get_all_sensor_data(struct bme280_data *data, struct bme280_dev *dev)
{
  bme280_set_sensor_mode(BME280_FORCED_MODE, dev);
  usleep(2000);
  return bme280_get_sensor_data(BME280_ALL, data, dev);
}

uint8_t bme280_if_sleep_mode(struct bme280_dev *dev)
{
  bme280_set_sensor_mode(BME280_SLEEP_MODE, dev);
  return 0;
}


uint8_t bme280_if_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{

  _i2cTransaction.writeCount = 1;
  _i2cTransaction.writeBuf = &reg_addr;
  _i2cTransaction.readCount = len;
  _i2cTransaction.readBuf = reg_data;
  // Note, as this system only uses one BME it can be set here.
  //    Maybe some magic like this could be used with a lambda
  //    uint8_t (*fun_ptr)() = (uint8_t (*)()) intf_ptr;
  _i2cTransaction.slaveAddress = BME280_I2C_ADDR_PRIM;

  I2C_transfer(*_bus, &_i2cTransaction);

  return 0;
}

uint8_t bme280_if_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{


  uint8_t temp_data[30], i;
  temp_data[0] = reg_addr;
  for (i = 0; i < len; i++)
  {
    temp_data[i + 1] = reg_data[i];
  }

  _i2cTransaction.writeCount = len + 1;
  _i2cTransaction.writeBuf = temp_data;
  _i2cTransaction.readCount = 0;
  // Note, as this system only uses one BME it can be set here.
  //    Maybe some magic like this could be used with a lambda
  //    uint8_t (*fun_ptr)() = (uint8_t (*)()) intf_ptr;
  _i2cTransaction.slaveAddress = BME280_I2C_ADDR_PRIM;

  I2C_transfer(*_bus, &_i2cTransaction);

  return 0;
}
