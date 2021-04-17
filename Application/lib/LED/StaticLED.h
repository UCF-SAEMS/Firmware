/*
 * StaticLED.h
 *
 *  Created on: Jan 31, 2021
 *      Author: Clayton
 */

#ifndef APPLICATION_LIB_LED_STATICLED_H_
#define APPLICATION_LIB_LED_STATICLED_H_

#include <stdint.h>
#include <type_traits>
#include "Application/lib/MCP23017/MCP23017.h"

#ifdef __cplusplus
extern "C" {
#endif

struct RGB_States
{
  enum Value
  {
    NONE = 0, RED = (1 << 0), GREEN = (1 << 1), BLUE = (1 << 2), ALL = 0x7
  };

};

class StaticLED
{
public:
  MCP23017 *_mcp;
  uint8_t _pinR, _pinG, _pinB;
  bool stateR, stateG, stateB;

  StaticLED(MCP23017 &mcp, uint8_t pinR, uint8_t pinG, uint8_t pinB);
  StaticLED();
  virtual ~StaticLED();

  void set(int state);
  void set(bool red, bool green, bool blue);

  void toggle(int state);
};

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_LIB_LED_STATICLED_H_ */
