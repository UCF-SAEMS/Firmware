/**************************************************************************************************************
 *****                                                                                                    *****
 *****  Name: hsi2rgbw.cpp                                                                                *****
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

#include "hsi2rgbw_pwm.h"

void hsi2rgbw(float H, float S, float I, float *rgbw)
{
  if (rgbw == NULL)
    return;

  float cos_h, Srgb;
  H = fmod(H, 360); // cycle H around to 0-360 degrees
  H = M_PI * H / (float) 180; // Convert to radians.
  S = S > 0 ? (S < 1 ? S : 1) : 0; // clamp S and I to interval [0,1]
  I = I > 0 ? (I < 1 ? I : 1) : 0;
  Srgb = 1;
  // This section is modified by the addition of white so that it assumes
  // fully saturated colors, and then scales with white to lower saturation.
  //
  // Next, scale appropriately the pure color by mixing with the white channel.
  // Saturation is defined as "the ratio of colorfulness to brightness" so we will
  // do this by a simple ratio wherein the color values are scaled down by (1-S)
  // while the white LED is placed at S.

  // This will maintain constant brightness because in HSI, R+B+G = I. Thus,
  // S*(R+B+G) = S*I. If we add to this (1-S)*I, where I is the total intensity,
  // the sum intensity stays constant while the ratio of colorfulness to brightness
  // goes down by S linearly relative to total Intensity, which is constant.

  if (H < 2.09439)
  {
    cos_h = cos(H) / cos(1.047196667 - H);
    rgbw[0] = S * I / 3 * (1 + Srgb * cos_h);
    rgbw[1] = S * I / 3 * (1 + Srgb * (1 - cos_h));
    rgbw[2] = 0;
    rgbw[3] = (1 - S) * I;
  }
  else if (H < 4.188787)
  {
    H = H - 2.09439;
    cos_h = cos(H) / cos(1.047196667 - H);
    rgbw[1] = S * I / 3 * (1 + Srgb * cos_h);
    rgbw[2] = S * I / 3 * (1 + Srgb * (1 - cos_h));
    rgbw[0] = 0;
    rgbw[3] = (1 - S) * I;
  }
  else
  {
    H = H - 4.188787;
    cos_h = cos(H) / cos(1.047196667 - H);
    rgbw[2] = S * I / 3 * (1 + Srgb * cos_h);
    rgbw[0] = S * I / 3 * (1 + Srgb * (1 - cos_h));
    rgbw[1] = 0;
    rgbw[3] = (1 - S) * I;
  }
}
