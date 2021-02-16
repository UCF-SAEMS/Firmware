/*
 * lmp91000_if.h
 *
 *  Created on: Feb 14, 2021
 *      Author: Daniel Mulvaney
 */

#ifndef APPLICATION_LIB_LMP91000_LMP91000_IF_H_
#define APPLICATION_LIB_LMP91000_LMP91000_IF_H_

#ifdef __cplusplus
extern "C" {
#endif

uint8_t lmp91000_i2c_read(uint8_t *reg_data, uint32_t len);
uint8_t lmp91000_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_LIB_LMP91000_LMP91000_IF_H_ */
