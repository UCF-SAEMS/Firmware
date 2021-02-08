/*
 * LEDBoard.cpp
 *
 *  Created on: Jan 31, 2021
 *      Author: Clayton
 */

#include <Application/lib/LED/LEDBoard.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/gates/GateMutex.h>
#include <ti/drivers/SPI.h>

#include "hsi2rgbw_pwm.h"
#include <math.h>
#include <stdint.h>

#include "WS2812.h"
#include <algorithm>

#include <hal_defs.h>

LEDBoard::LEDBoard(unsigned int indexSPI)
{
  _indexSPI = indexSPI;
}

LEDBoard::~LEDBoard()
{
  // TODO Auto-generated destructor stub
}

void LEDBoard::init()
{
  WS2812_init(_indexSPI);
}

void LEDBoard::hsi(float H, float S, float I)
{
  uint8_t r, g, b, w;
  float rgbw[4];  // 0 - 1 range of the PWM, map accordingly

  hsi2rgbw(H, S, I, rgbw);

  r = MAX(0, MIN(255, (int) floor(rgbw[0] * 256.0)));
  g = MAX(0, MIN(255, (int) floor(rgbw[1] * 256.0)));
  b = MAX(0, MIN(255, (int) floor(rgbw[2] * 256.0)));
  w = MAX(0, MIN(255, (int) floor(rgbw[3] * 256.0)));

  writetoled(r, g, b, w, 0, 0);
}

void LEDBoard::writetoled(uint8_t r, uint8_t g, uint8_t b, uint8_t w, uint8_t colorbrightness, uint8_t wbrightness)
{
  WS2812_setLEDcolor(0, r, g, b, false);
  WS2812_setLEDcolor(1, w, colorbrightness, wbrightness, true);
}

