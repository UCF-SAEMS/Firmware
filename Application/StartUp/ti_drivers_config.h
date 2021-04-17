/*
 *  ======== ti_drivers_config.h ========
 *  Configured TI-Drivers module declarations
 *
 *  The macros defines herein are intended for use by applications which
 *  directly include this header. These macros should NOT be hard coded or
 *  copied into library source code.
 *
 *  Symbols declared as const are intended for use with libraries.
 *  Library source code must extern the correct symbol--which is resolved
 *  when the application is linked.
 *
 *  DO NOT EDIT - This file is generated for the CC1352P_2_LAUNCHXL
 *  by the SysConfig tool.
 */
#ifndef ti_drivers_config_h
#define ti_drivers_config_h

#define CONFIG_SYSCONFIG_PREVIEW

#define CONFIG_CC1352P_2_LAUNCHXL
#ifndef DeviceFamily_CC13X2
#define DeviceFamily_CC13X2
#endif

#include <ti/devices/DeviceFamily.h>

#include <stdint.h>

/* support C++ sources */
#ifdef __cplusplus
extern "C" {
#endif


/*
 *  ======== CCFG ========
 */


/*
 *  ======== ADC ========
 */

/* DIO23 */
extern const uint_least8_t              CO_OUT_CONST;
#define CO_OUT                          0
/* DIO24 */
extern const uint_least8_t              POWER_POLL_CONST;
#define POWER_POLL                      1
#define CONFIG_TI_DRIVERS_ADC_COUNT     2


/*
 *  ======== AESCBC ========
 */

extern const uint_least8_t                  CONFIG_AESCBC_0_CONST;
#define CONFIG_AESCBC_0                     0
#define CONFIG_TI_DRIVERS_AESCBC_COUNT      1


/*
 *  ======== AESCCM ========
 */

extern const uint_least8_t                  CONFIG_AESCCM_0_CONST;
#define CONFIG_AESCCM_0                     0
#define CONFIG_TI_DRIVERS_AESCCM_COUNT      1


/*
 *  ======== AESECB ========
 */

extern const uint_least8_t                  CONFIG_AESECB_0_CONST;
#define CONFIG_AESECB_0                     0
#define CONFIG_TI_DRIVERS_AESECB_COUNT      1


/*
 *  ======== ECDH ========
 */

extern const uint_least8_t              CONFIG_ECDH_0_CONST;
#define CONFIG_ECDH_0                   0
#define CONFIG_TI_DRIVERS_ECDH_COUNT    1


/*
 *  ======== ECDSA ========
 */

extern const uint_least8_t                  CONFIG_ECDSA_0_CONST;
#define CONFIG_ECDSA_0                      0
#define CONFIG_TI_DRIVERS_ECDSA_COUNT       1


/*
 *  ======== ECJPAKE ========
 */

extern const uint_least8_t                  CONFIG_ECJPAKE_0_CONST;
#define CONFIG_ECJPAKE_0                    0
#define CONFIG_TI_DRIVERS_ECJPAKE_COUNT     1


/*
 *  ======== GPIO ========
 */

/* DIO27 */
extern const uint_least8_t              PIR_SENSOR_CONST;
#define PIR_SENSOR                      0
/* DIO7 */
extern const uint_least8_t              GPIO_ETH_CS_CONST;
#define GPIO_ETH_CS                     1
/* DIO18 */
extern const uint_least8_t              GPIO_RTS_CONST;
#define GPIO_RTS                        2
#define CONFIG_TI_DRIVERS_GPIO_COUNT    3

/* LEDs are active high */
#define CONFIG_GPIO_LED_ON  (1)
#define CONFIG_GPIO_LED_OFF (0)

#define CONFIG_LED_ON  (CONFIG_GPIO_LED_ON)
#define CONFIG_LED_OFF (CONFIG_GPIO_LED_OFF)


/*
 *  ======== I2C ========
 */

/*
 *  SCL: DIO20
 *  SDA: DIO5
 */
extern const uint_least8_t              CONFIG_I2C_0_CONST;
#define CONFIG_I2C_0                    0
#define CONFIG_TI_DRIVERS_I2C_COUNT     1

/* ======== I2C Addresses and Speeds ======== */
#include <ti/drivers/I2C.h>

/* ---- CONFIG_I2C_0 I2C bus components ---- */

/* no components connected to CONFIG_I2C_0 */

/* max speed unspecified, defaulting to 100 Kbps */
#define CONFIG_I2C_0_MAXSPEED   (100U) /* Kbps */
#define CONFIG_I2C_0_MAXBITRATE ((I2C_BitRate)I2C_100kHz)


/*
 *  ======== NVS ========
 */

extern const uint_least8_t              CONFIG_NVSINTERNAL_CONST;
#define CONFIG_NVSINTERNAL              0
#define CONFIG_TI_DRIVERS_NVS_COUNT     1


/*
 *  ======== PIN ========
 */
#include <ti/drivers/PIN.h>

extern const PIN_Config BoardGpioInitTable[];

/* Parent Signal: CONFIG_DISPLAY_UART_MINE TX, (DIO12) */
#define CONFIG_PIN_5                   0x0000000c
/* Parent Signal: CONFIG_DISPLAY_UART_MINE RX, (DIO13) */
#define CONFIG_PIN_6                   0x0000000d
/* Parent Signal: CO_OUT ADC Pin, (DIO23) */
#define CONFIG_PIN_7                   0x00000017
/* Parent Signal: POWER_POLL ADC Pin, (DIO24) */
#define CONFIG_PIN_9                   0x00000018
/* Parent Signal: PIR_SENSOR GPIO Pin, (DIO27) */
#define CONFIG_PIN_8                   0x0000001b
/* Parent Signal: GPIO_ETH_CS GPIO Pin, (DIO7) */
#define CONFIG_PIN_14                   0x00000007
/* Parent Signal: GPIO_RTS GPIO Pin, (DIO18) */
#define CONFIG_PIN_11                   0x00000012
/* Parent Signal: CONFIG_I2C_0 SDA, (DIO5) */
#define CONFIG_PIN_1                   0x00000005
/* Parent Signal: CONFIG_I2C_0 SCL, (DIO20) */
#define CONFIG_PIN_2                   0x00000014
/* Parent Signal: CONFIG_GPTIMER_0 PWM Pin, (DIO15) */
#define CONFIG_PIN_10                   0x0000000f
/* Parent Signal: CONFIG_SPI_LEDBOARD SCLK, (DIO16) */
#define CONFIG_PIN_21                   0x00000010
/* Parent Signal: CONFIG_SPI_LEDBOARD MISO, (DIO11) */
#define CONFIG_PIN_20                   0x0000000b
/* Parent Signal: CONFIG_SPI_LEDBOARD MOSI, (DIO26) */
#define CONFIG_PIN_4                   0x0000001a
/* Parent Signal: CONFIG_SPI_W5500 SCLK, (DIO10) */
#define CONFIG_PIN_15                   0x0000000a
/* Parent Signal: CONFIG_SPI_W5500 MISO, (DIO8) */
#define CONFIG_PIN_13                   0x00000008
/* Parent Signal: CONFIG_SPI_W5500 MOSI, (DIO9) */
#define CONFIG_PIN_12                   0x00000009
/* SKY13317-373LF RF Antenna Switch, Parent Signal: /ti/drivers/RF RF Antenna Pin 0, (DIO28) */
#define CONFIG_RF_24GHZ                   0x0000001c
/* SKY13317-373LF RF Antenna Switch, Parent Signal: /ti/drivers/RF RF Antenna Pin 1, (DIO29) */
#define CONFIG_RF_HIGH_PA                   0x0000001d
/* SKY13317-373LF RF Antenna Switch, Parent Signal: /ti/drivers/RF RF Antenna Pin 2, (DIO30) */
#define CONFIG_RF_SUB1GHZ                   0x0000001e
#define CONFIG_TI_DRIVERS_PIN_COUNT    19


/*
 *  ======== PWM ========
 */

/* DIO15 */
extern const uint_least8_t              CONFIG_PWM_4KHZ_CONST;
#define CONFIG_PWM_4KHZ                 0
#define CONFIG_TI_DRIVERS_PWM_COUNT     1


/*
 *  ======== RF ========
 */
#define Board_DIO_30_RFSW 0x0000001e


/*
 *  ======== SHA2 ========
 */

extern const uint_least8_t              CONFIG_SHA2_0_CONST;
#define CONFIG_SHA2_0                   0
#define CONFIG_TI_DRIVERS_SHA2_COUNT    1


/*
 *  ======== SPI ========
 */

/*
 *  MOSI: DIO26
 *  MISO: DIO11
 *  SCLK: DIO16
 */
extern const uint_least8_t              CONFIG_SPI_LEDBOARD_CONST;
#define CONFIG_SPI_LEDBOARD             0
/*
 *  MOSI: DIO9
 *  MISO: DIO8
 *  SCLK: DIO10
 */
extern const uint_least8_t              CONFIG_SPI_W5500_CONST;
#define CONFIG_SPI_W5500                1
#define CONFIG_TI_DRIVERS_SPI_COUNT     2


/*
 *  ======== TRNG ========
 */

extern const uint_least8_t              CONFIG_TRNG_0_CONST;
#define CONFIG_TRNG_0                   0
#define CONFIG_TI_DRIVERS_TRNG_COUNT    1


/*
 *  ======== UART ========
 */

/*
 *  TX: DIO12
 *  RX: DIO13
 */
extern const uint_least8_t              CONFIG_DISPLAY_UART_MINE_CONST;
#define CONFIG_DISPLAY_UART_MINE        0
#define CONFIG_TI_DRIVERS_UART_COUNT    1


/*
 *  ======== GPTimer ========
 */

extern const uint_least8_t                  CONFIG_GPTIMER_0_CONST;
#define CONFIG_GPTIMER_0                    0
#define CONFIG_TI_DRIVERS_GPTIMER_COUNT     1


/*
 *  ======== Board_init ========
 *  Perform all required TI-Drivers initialization
 *
 *  This function should be called once at a point before any use of
 *  TI-Drivers.
 */
extern void Board_init(void);

/*
 *  ======== Board_initGeneral ========
 *  (deprecated)
 *
 *  Board_initGeneral() is defined purely for backward compatibility.
 *
 *  All new code should use Board_init() to do any required TI-Drivers
 *  initialization _and_ use <Driver>_init() for only where specific drivers
 *  are explicitly referenced by the application.  <Driver>_init() functions
 *  are idempotent.
 */
#define Board_initGeneral Board_init

#ifdef __cplusplus
}
#endif

#endif /* include guard */
