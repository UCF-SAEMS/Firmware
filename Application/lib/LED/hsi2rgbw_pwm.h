/**************************************************************************************************************
 *****                                                                                                    *****
 *****  Name: hsi2rgbw.h                                                                                  *****
 *****  Date: 22/12/2013                                                                                  *****
 *****  Auth: Frank Vannieuwkerke                                                                         *****
 *****  Func: library for converting HSI color space values to RGBW                                       *****
 *****                                                                                                    *****
 *****  Code ported from http://saikoled.com - Copyright 2013, Brian Neltner                              *****
 *****  http://blog.saikoled.com/post/44677718712/how-to-convert-from-hsi-to-rgb-white                    *****
 *****  http://blog.saikoled.com/post/43693602826/why-every-led-light-should-be-using-hsi-colorspace      *****
 *****  https://github.com/saikoLED/MyKi/blob/master/myki_16_bit_random_fade/myki_16_bit_random_fade.ino  *****
 *****  https://github.com/saikoLED/MyKi/blob/master/myki_16_bit_fade/myki_16_bit_fade.ino                *****
 *****                                                                                                    *****
 *****  This program is free software: you can redistribute it and/or modify                              *****
 *****  it under the terms of the GNU General Public License as published by                              *****
 *****  the Free Software Foundation, version 3 of the License.                                           *****
 *****                                                                                                    *****
 *****  This program is distributed in the hope that it will be useful,                                   *****
 *****  but WITHOUT ANY WARRANTY; without even the implied warranty of                                    *****
 *****  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                                     *****
 *****  GNU General Public License for more details.                                                      *****
 *****                                                                                                    *****
 *****  A copy of the GNU General Public License can be found at                                          *****
 *****  http://www.gnu.org/licenses/gpl.html                                                              *****
 **************************************************************************************************************/

#include <math.h>
#include <stdint.h>

#ifndef HSI2RGBW_PWM_H
#define HSI2RGBW_PWM_H

#ifdef __cplusplus
extern "C" {
#endif

/** Convert HSI to RGBW or RGB.
 *
 * @param Hue         = float (0..360)\n
 * @param Saturation  = float (0..1)\n
 * @param Intensity   = float (0..1)\n
 * @param rgbw        Optional pointer to float array[4]\n
 *                    This parameter can be omitted when PWM output is enabled:\n
 *                    hsi2rgbw(H, S, I)\n
 *
 * This function can be used to convert HSI to RGBW or RGB:\n
 *   call colorMode(RGBW) to enable HSI->RGBW\n
 *   call colorMode(RGB)  to enable HSI->RGB\n
 *   IMPORTANT NOTE : When RGB is selected and you want the function to return\n
 *                    the converted value, do declare a 4 element array!!\n
 *                    The value in the 4th element can be discarded.\n
 */
void hsi2rgbw(float H, float S, float I, float *rgbw = NULL);

#ifdef __cplusplus
}
#endif

#endif
