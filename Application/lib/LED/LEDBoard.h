/*
 * LEDBoard.h
 *
 *  Created on: Jan 31, 2021
 *      Author: Clayton
 */

#ifndef APPLICATION_LIB_LED_LEDBOARD_H_
#define APPLICATION_LIB_LED_LEDBOARD_H_

#include <stdint.h>
#include <type_traits>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/gates/GateMutex.h>
#include <ti/drivers/SPI.h>
#ifdef __cplusplus
extern "C" {
#endif

class LEDBoard
{
public:
  unsigned int _indexSPI;

  GateMutex_Handle frameBuffer;
  SPI_Handle spiHandle;

  LEDBoard(unsigned int indexSPI);
  virtual ~LEDBoard();

  void init();

  void hsi(float H, float S, float I, bool receivedFromOff);
  void writetoled(uint8_t r, uint8_t g, uint8_t b, uint8_t w, uint8_t colorbrightness, uint8_t wbrightness);
};

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_LIB_LED_LEDBOARD_H_ */
