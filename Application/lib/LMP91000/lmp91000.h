/*
FILENAME:  LMP91000.h
 AUTHOR:    Orlando S. Hoilett
 EMAIL:     ohoilett@purdue.edu


 Please see .cpp file for extended descriptions, instructions, and version updates


 Linnes Lab code, firmware, and software is released under the MIT License
 (http://opensource.org/licenses/MIT).

 The MIT License (MIT)

 Copyright (c) 2016 Linnes Lab, Purdue University, West Lafayette, IN, USA

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights to
 use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 of the Software, and to permit persons to whom the Software is furnished to do
 so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.

 */




#ifndef LMP91000_H
#define LMP91000_H

#include <stdint.h>
#include <stddef.h>
#include <unistd.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/ADC.h>
#include "ti_drivers_config.h"

const double TEMP_INTERCEPT = 1555;
const double TEMPSLOPE = -8;
#define LMP91000_I2C_ADDRESS (0X48)

#define LMP91000_STATUS_REG  (0x00)    /* Read only status register */
#define LMP91000_LOCK_REG (0x01)    /* Protection Register */
#define LMP91000_TIACN_REG (0x10)    /* TIA Control Register */
#define LMP91000_REFCN_REG (0x11)    /* Reference Control Register*/
#define LMP91000_MODECN_REG (0x12)    /* Mode Control Register */

#define LMP91000_READY (0x01)
#define LMP91000_NOT_READY (0x00)

#define LMP91000_TIA_GAIN_EXT (0x00) //default
#define LMP91000_TIA_GAIN_2P75K (0x04)
#define LMP91000_TIA_GAIN_3P5K (0x08)
#define LMP91000_TIA_GAIN_7K (0x0C)
#define LMP91000_TIA_GAIN_14K (0x10)
#define LMP91000_TIA_GAIN_35K (0x14)
#define LMP91000_TIA_GAIN_120K (0x18)
#define LMP91000_TIA_GAIN_350K (0x1C)

#define LMP91000_RLOAD_10OHM (0X00)
#define LMP91000_RLOAD_33OHM (0X01)
#define LMP91000_RLOAD_50OHM (0X02)
#define LMP91000_RLOAD_100OHM (0X03) //default

#define LMP91000_REF_SOURCE_INT (0x00) //default
#define LMP91000_REF_SOURCE_EXT (0x80)

#define LMP91000_INT_Z_20PCT (0x00)
#define LMP91000_INT_Z_50PCT (0x20) //default
#define LMP91000_INT_Z_67PCT (0x40)
#define LMP91000_INT_Z_BYPASS (0x60)

#define LMP91000_BIAS_SIGN_NEG (0x00) //default
#define LMP91000_BIAS_SIGN_POS (0x10)

#define LMP91000_BIAS_0PCT (0x00) //default
#define LMP91000_BIAS_1PCT (0x01)
#define LMP91000_BIAS_2PCT (0x02)
#define LMP91000_BIAS_4PCT (0x03)
#define LMP91000_BIAS_6PCT (0x04)
#define LMP91000_BIAS_8PCT (0x05)
#define LMP91000_BIAS_10PCT (0x06)
#define LMP91000_BIAS_12PCT (0x07)
#define LMP91000_BIAS_14PCT (0x08)
#define LMP91000_BIAS_16PCT (0x09)
#define LMP91000_BIAS_18PCT (0x0A)
#define LMP91000_BIAS_20PCT   (0x0B)
#define LMP91000_BIAS_22PCT (0x0C)
#define LMP91000_BIAS_24PCT (0x0D)

#define LMP91000_FET_SHORT_DISABLED (0x00) //default
#define LMP91000_FET_SHORT_ENABLED (0x80)
#define LMP91000_OP_MODE_DEEP_SLEEP (0x00) //default
#define LMP91000_OP_MODE_GALVANIC (0x01)
#define LMP91000_OP_MODE_STANDBY (0x02)
#define LMP91000_OP_MODE_AMPEROMETRIC (0x03)
#define LMP91000_OP_MODE_TIA_OFF (0x06)
#define LMP91000_OP_MODE_TIA_ON (0x07)

#define LMP91000_WRITE_LOCK (0x01) //default
#define LMP91000_WRITE_UNLOCK (0x00)

#define LMP91000_NOT_PRESENT (0xA8)    // arbitrary library status code

const double TIA_GAIN[] = {2750,3500,7000,14000,35000,120000,350000};
const double TIA_BIAS[] = {0, 0.01, 0.02, 0.04, 0.06, 0.08, 0.1, 0.12, 0.14,
    0.16, 0.18, 0.2, 0.22, 0.24};
const uint8_t NUM_TIA_BIAS = 14;
const double TIA_ZERO[] = {0.2, 0.5, 0.67};


class LMP91000 {

private:
    I2C_Handle *_bus;
    I2C_Transaction _i2cTransaction;
    uint8_t _txBuffer[1];
    uint8_t _rxBuffer[2];
    uint8_t _deviceAddr;

    uint8_t MENB; //IO pin for enabling and disabling I2C commands
    uint8_t gain;
    uint8_t zero;


public:

    //CONSTRUCTORS
    LMP91000(I2C_Handle& bus, uint8_t address); //tested

    //sets and gets MENB pin for enabling and disabling I2C commands
    void setMENB(uint8_t pin);
    uint8_t getMENB();

    //sets and gets pin for reading output of temperature sensor
    void setTempSensor(uint8_t pin);
    uint8_t getTempSensor();

    //reads and writes to LMP91000 via I2C
    void write(uint8_t reg, uint8_t data);
    uint8_t read(uint8_t reg);

    //enables and disables LMP91000 for I2C commands
    //default state is not ready
    void enable();
    void disable();
    bool isReady();

    //locks and unlocks the transimpedance amplifier
    //and reference control registers for editing
    //default state is locked (read-only)
    void lock();
    void unlock();
    bool isLocked();

    //sets the gain of the transimpedance amplifier
    void setGain(uint8_t gain);
    double getGain();

    //sets the load for compensating voltage differences
    //between working and reference electrodes
    void setRLoad(uint8_t load);

    //sets the source for the bias voltage for the
    //electrochemical cell
    void setRefSource(uint8_t source);
    void setIntRefSource();
    void setExtRefSource();

    //sets reference voltage for transimpedance amplifier
    void setIntZ(uint8_t intZ);
    double getIntZ();

    //sets bias voltage for electrochemical cell
    void setBiasSign(uint8_t sign);
    void setNegBias();
    void setPosBias();
    void setBias(uint8_t bias);
    void setBias(uint8_t bias, signed char sign);

    //enable and disable FET for deep sleep mode
    void setFET(uint8_t selection);
    void disableFET();
    void enableFET();

    //set operating modes for the LMP91000
    void setMode(uint8_t mode);
    void sleep();
    void setTwoLead();
    void standby();
    void setThreeLead();
    void measureCell();
    void setTemp();
    uint32_t getTemp(ADC_Handle handle);
    uint32_t getCurrentExtern(ADC_Handle handle, uint8_t extGain);
    uint32_t getCurrent(ADC_Handle handle);

    //reading the output of the LMP91000
    uint32_t getADC(ADC_Handle handle);

};

#endif
