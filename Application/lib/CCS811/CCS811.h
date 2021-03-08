//CCS811.h

/*
  ScioSense_CCS811.h - Library for the CCS811 digital gas sensor for monitoring indoor air quality from ams.
  2020 apr 06  v12  Christoph Friese  adjusted to ScioSense as product shifted from ams
  2019 sep 17  v11  Christoph Friese  changed I2C communication to adjust to M0
  2019 jan 22  v10  Maarten Pennings  Added F() on all strings, added get/set_baseline()
  2019 jan 15   v9  Maarten Pennings  Flash&i2cwrite now use const array
  2018 dec 06   v8  Maarten Pennings  Added firmware flash routine
  2018 Dec 04   v7  Maarten Pennings  Added support for older CCS811's (fw 1100)
  2018 Nov 11   v6  Maarten Pennings  uint16 -> uint16_t
  2018 Nov 02   v5  Maarten Pennings  Added clearing of ERROR_ID
  2018 Oct 23   v4  Maarten Pennings  Added envdata/i2cdelay
  2018 Oct 21   v3  Maarten Pennings  Added hw-version
  2018 Oct 21   v2  Maarten Pennings  Simplified I2C, added version mngt
  2017 Dec 11   v1  Maarten Pennings  Created
*/


#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/I2C.h>
#include "ti_drivers_config.h"

#ifndef _SCIOSENSE_CCS811_H_
#define _SCIOSENSE_CCS811_H_



// To help diagnose problems, the begin() and flash() functions print diagnostic messages to Serial.
// If you do not want that, make the PRINT macros in ccs811.cpp empty.
#include <stdint.h>


// Version of this CCS811 driver
#define CCS811_VERSION                     11

// I2C slave address for ADDR 0 respectively 1
#define CCS811_SLAVEADDR_0                 0x5A
#define CCS811_SLAVEADDR_1                 0x5B


// The values for mode in ccs811_start()
#define CCS811_MODE_IDLE                   0
#define CCS811_MODE_1SEC                   1
#define CCS811_MODE_10SEC                  2
#define CCS811_MODE_60SEC                  3


// The flags for errstat in ccs811_read()
// ERRSTAT is a merge of two hardware registers: ERROR_ID (bits 15-8) and STATUS (bits 7-0)
// Also bit 1 (which is always 0 in hardware) is set to 1 when an I2C read error occurs
#define CCS811_ERRSTAT_ERROR               0x0001 // There is an error, the ERROR_ID register (0xE0) contains the error source
#define CCS811_ERRSTAT_I2CFAIL             0x0002 // Bit flag added by software: I2C transaction error
#define CCS811_ERRSTAT_DATA_READY          0x0008 // A new data sample is ready in ALG_RESULT_DATA
#define CCS811_ERRSTAT_APP_VALID           0x0010 // Valid application firmware loaded
#define CCS811_ERRSTAT_FW_MODE             0x0080 // Firmware is in application mode (not boot mode)
#define CCS811_ERRSTAT_WRITE_REG_INVALID   0x0100 // The CCS811 received an I�C write request addressed to this station but with invalid register address ID
#define CCS811_ERRSTAT_READ_REG_INVALID    0x0200 // The CCS811 received an I�C read request to a mailbox ID that is invalid
#define CCS811_ERRSTAT_MEASMODE_INVALID    0x0400 // The CCS811 received an I�C request to write an unsupported mode to MEAS_MODE
#define CCS811_ERRSTAT_MAX_RESISTANCE      0x0800 // The sensor resistance measurement has reached or exceeded the maximum range
#define CCS811_ERRSTAT_HEATER_FAULT        0x1000 // The heater current in the CCS811 is not in range
#define CCS811_ERRSTAT_HEATER_SUPPLY       0x2000 // The heater voltage is not being applied correctly

// begin() and flash() prints errors to help diagnose startup problems.
// Change these macro's to empty to suppress those prints.
#define PRINTLN(s)    Serial.println(s)
#define PRINT(s)      Serial.print(s)
#define PRINTLN2(s,m) Serial.println(s,m)
#define PRINT2(s,m)   Serial.print(s,m)

// Timings
#define CCS811_WAIT_AFTER_RESET_US     2000 // The CCS811 needs a wait after reset
#define CCS811_WAIT_AFTER_APPSTART_US  1000 // The CCS811 needs a wait after app start
#define CCS811_WAIT_AFTER_WAKE_US        50 // The CCS811 needs a wait after WAKE signal
#define CCS811_WAIT_AFTER_APPERASE_MS   500 // The CCS811 needs a wait after app erase (300ms from spec not enough)
#define CCS811_WAIT_AFTER_APPVERIFY_MS   70 // The CCS811 needs a wait after app verify
#define CCS811_WAIT_AFTER_APPDATA_MS     50 // The CCS811 needs a wait after writing app data


// Main interface =====================================================================================================


// CCS811 registers/mailboxes, all 1 byte except when stated otherwise
#define CCS811_STATUS           0x00
#define CCS811_MEAS_MODE        0x01
#define CCS811_ALG_RESULT_DATA  0x02 // up to 8 bytes
#define CCS811_RAW_DATA         0x03 // 2 bytes
#define CCS811_ENV_DATA         0x05 // 4 bytes
#define CCS811_THRESHOLDS       0x10 // 5 bytes
#define CCS811_BASELINE         0x11 // 2 bytes
#define CCS811_HW_ID            0x20
#define CCS811_HW_VERSION       0x21
#define CCS811_FW_BOOT_VERSION  0x23 // 2 bytes
#define CCS811_FW_APP_VERSION   0x24 // 2 bytes
#define CCS811_ERROR_ID         0xE0
#define CCS811_APP_ERASE        0xF1 // 4 bytes
#define CCS811_APP_DATA         0xF2 // 9 bytes
#define CCS811_APP_VERIFY       0xF3 // 0 bytes
#define CCS811_APP_START        0xF4 // 0 bytes
#define CCS811_SW_RESET         0xFF // 4 bytes

// These flags should not be set. They flag errors.
#define CCS811_ERRSTAT_HWERRORS            ( CCS811_ERRSTAT_ERROR | CCS811_ERRSTAT_WRITE_REG_INVALID | CCS811_ERRSTAT_READ_REG_INVALID | CCS811_ERRSTAT_MEASMODE_INVALID | CCS811_ERRSTAT_MAX_RESISTANCE | CCS811_ERRSTAT_HEATER_FAULT | CCS811_ERRSTAT_HEATER_SUPPLY )
#define CCS811_ERRSTAT_ERRORS              ( CCS811_ERRSTAT_I2CFAIL | CCS811_ERRSTAT_HWERRORS )
// These flags should normally be set - after a measurement. They flag data available (and valid app running).
#define CCS811_ERRSTAT_OK                  ( CCS811_ERRSTAT_DATA_READY | CCS811_ERRSTAT_APP_VALID | CCS811_ERRSTAT_FW_MODE )
// These flags could be set after a measurement. They flag data is not yet available (and valid app running).
#define CCS811_ERRSTAT_OK_NODATA           ( CCS811_ERRSTAT_APP_VALID | CCS811_ERRSTAT_FW_MODE )


class ScioSense_CCS811 {
    private:
    I2C_Handle *_bus;
    I2C_Transaction _i2cTransaction;


  public: // Main interface
    ScioSense_CCS811(I2C_Handle& bus, uint8_t slaveaddr);               // Pin number connected to nWAKE (nWAKE can also be bound to GND, then pass -1), slave address (5A or 5B)
    bool begin( void );                                                       // Reset the CCS811, switch to app mode and check HW_ID. Returns false on problems.
    bool start( int mode );                                                   // Switched CCS811 to `mode`, use constants CCS811_MODE_XXX. Returns false on I2C problems.
    //void read( uint16_t*eco2, uint16_t*etvoc, uint16_t*errstat,uint16_t*raw); // Get measurement results from the CCS811 (all args may be NULL), check status via errstat, e.g. ccs811_errstat(errstat)
    //void read();                                                              // Get measurement results from the CCS811 (all args may be NULL), check status via errstat, e.g. ccs811_errstat(errstat)
    const char * errstat_str(uint16_t errstat);                               // Returns a string version of an errstat. Note, each call, this string is updated.
    bool available() { return this->_available; }

  public: // Extra interface
    int  hardware_version(void);                                              // Gets version of the CCS811 hardware (returns -1 on I2C failure)
    int  bootloader_version(void);                                            // Gets version of the CCS811 bootloader (returns -1 on I2C failure)
    int  application_version(void);                                           // Gets version of the CCS811 application (returns -1 on I2C failure)
    int  get_errorid(void);                                                   // Gets the ERROR_ID [same as 'err' part of 'errstat' in 'read'] (returns -1 on I2C failure)
    bool set_envdata(uint16_t t, uint16_t h);                                 // Writes t and h to ENV_DATA (see datasheet for format). Returns false on I2C problems.
    bool set_envdata210(uint16_t t, uint16_t h);                              // Writes t and h (in ENS210 format) to ENV_DATA. Returns false on I2C problems.
    bool get_baseline(uint16_t *baseline);                                    // Reads (encoded) baseline from BASELINE. Returns false on I2C problems. Get it, just before power down (but only when sensor was on at least 20min) - see CCS811_AN000370
    bool set_baseline(uint16_t baseline);                                     // Writes (encoded) baseline to BASELINE. Returns false on I2C problems. Set it, after power up (and after 20min)
    bool flash(const uint8_t * image, int size);                              // Flashes the firmware of the CCS811 with size bytes from image - image _must_ be in PROGMEM
    uint16_t getECO2() { return _eCO2; };
    uint16_t getTVOC() { return _eTVOC; };
    uint16_t getErrstat() { return _errstat; };
    uint16_t getRaw() { return _raw; };
  public: // Advanced interface: i2cdelay
    uint8_t read(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t num);// Get current delay
    void write(uint8_t addr, uint8_t reg, uint8_t data[], uint8_t num);
  protected: // Helper interface: i2c wrapper
    uint8_t read8(uint8_t addr, uint8_t reg);
    uint8_t write8(uint8_t addr, uint8_t reg, uint8_t value);

  private:
    bool _available = false;
    int  _nwake;                                                              // Pin number for nWAKE pin (or -1)
    int  _slaveaddr;                                                          // Slave address of the CCS811
    int  _i2cdelay_us;                                                        // Delay in us just before an I2C repeated start condition
    int  _appversion;                                                         // Version of the app firmware inside the CCS811 (for workarounds)
    uint16_t _eCO2;
    uint16_t _eTVOC;
    uint16_t _errstat;
    uint16_t _raw;
};


#endif
