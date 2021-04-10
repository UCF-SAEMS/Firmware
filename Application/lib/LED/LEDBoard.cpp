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

/* 
The function receives the Hue, Saturation, and Level Attribute as the Intensity
It also receives a boolean stating if the function was called from the 'Off' zigbee command or not
This will turn the light completely off (not visible), instead of leaving it at the lowest level
*/
void LEDBoard::hsi(float H, float S, float I, bool receivedFromOff)
{
  uint8_t r, g, b, w;
  float rgbw[4];  // 0 - 1 range of the PWM, map accordingly

  hsi2rgbw(H, S, I, rgbw);

  // The max element and max index from the rgbw array is retrieved
  int maxIndex = std::distance(rgbw, std::max_element(rgbw,rgbw+4));
  float maxElement = *std::max_element(rgbw,rgbw+4);

  /* 
  The range found for the light when it becomes 'not visible' on the LEDBoard is from 0.00 - 0.09
  If the max element from the array falls within this range and the function was not called from the Off Command
    
  The max value in the array is scaled accordingly starting at the minimum so it shall lie between a range of 0.09 - 0.1
  Because there are some visible values in the 0.090 - 0.10 range already without scaling, the range is increased to include
  any value that is less than 0.099 - this gets rid of any overlap there may be with the scaled values and unscaled values
  */
  if( maxElement < 0.099 && !receivedFromOff){
    rgbw[maxIndex] = 0.09 + (0.1*maxElement);
  }

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

