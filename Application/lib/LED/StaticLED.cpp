/*
 * StaticLED.cpp
 *
 *  Created on: Jan 31, 2021
 *      Author: Clayton
 */

#include <Application/lib/LED/StaticLED.h>

StaticLED::StaticLED(MCP23017 &mcp, uint8_t pinR, uint8_t pinG, uint8_t pinB)
{
  _mcp = &mcp;
  _pinR = pinR;
  _pinG = pinG;
  _pinB = pinB;
  stateR = stateG = stateB = false;

  _mcp->pinMode(_pinR, OUTPUT);
  _mcp->pinMode(_pinG, OUTPUT);
  _mcp->pinMode(_pinB, OUTPUT);

  set(RGB_States::NONE);
}

StaticLED::StaticLED(){
  // NULL constructor - allows for global declaration of StaticLED variable
}

StaticLED::~StaticLED()
{
  // TODO Auto-generated destructor stub
}

void StaticLED::set(bool red, bool green, bool blue)
{
  // The led is common anode so invert the signals
  _mcp->digitalWrite(_pinR, !red);
  _mcp->digitalWrite(_pinG, !green);
  _mcp->digitalWrite(_pinB, !blue);
}

void StaticLED::set(int state)
{
  stateR = (state & RGB_States::RED);
  stateG = (state & RGB_States::GREEN);
  stateB = (state & RGB_States::BLUE);

  set(stateR, stateG, stateB);
}

void StaticLED::toggle(int state)
{
  stateR ^= (bool) (state & RGB_States::RED);
  stateG ^= (bool) (state & RGB_States::GREEN);
  stateB ^= (bool) (state & RGB_States::BLUE);

  set(stateR, stateG, stateB);
}

