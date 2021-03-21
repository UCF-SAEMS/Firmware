/*
 * Copyright (c) 2018, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "stdint.h"
#include <stdlib.h>
#include <string.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/I2C.h>
#include "ti_drivers_config.h"

#include "sensirion_arch_config.h"
#include "sensirion_common.h"
#include "sensirion_i2c.h"

I2C_Handle *_sensirion_bus;
I2C_Transaction _sensirion_i2cTransaction;

void sensirion_i2c_init(I2C_Handle *bus) {
  _sensirion_bus = bus;
  memset(&_sensirion_i2cTransaction, 0, sizeof(_sensirion_i2cTransaction));
}

/**
 * Release all resources initialized by sensirion_i2c_init().
 */
void sensirion_i2c_release(void) {
}


int8_t sensirion_i2c_read(uint8_t address, uint8_t *data, uint16_t count) {
  int32_t ret;

  _sensirion_i2cTransaction.writeCount = 0;
  _sensirion_i2cTransaction.writeBuf = NULL;
  _sensirion_i2cTransaction.readCount = count;
  _sensirion_i2cTransaction.readBuf = (void*) data;
  _sensirion_i2cTransaction.slaveAddress = address;

  // false to indicate an error
  ret = (I2C_transfer(*_sensirion_bus, &_sensirion_i2cTransaction) == false) ? _sensirion_i2cTransaction.status : NO_ERROR;

  return ret;
}

int8_t sensirion_i2c_write(uint8_t address, const uint8_t *data, uint16_t count) {
  int32_t ret;

  _sensirion_i2cTransaction.writeCount = count;
  _sensirion_i2cTransaction.writeBuf = (void*) data;
  _sensirion_i2cTransaction.readCount = 0;
  _sensirion_i2cTransaction.readBuf = NULL;
  _sensirion_i2cTransaction.slaveAddress = address;

  // false to indicate an error
  ret = (I2C_transfer(*_sensirion_bus, &_sensirion_i2cTransaction) == false) ? _sensirion_i2cTransaction.status : NO_ERROR;

  return ret;
}

void sensirion_sleep_usec(uint32_t useconds) {
  Task_sleep(useconds / Clock_tickPeriod);
}
