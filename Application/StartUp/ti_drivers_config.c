/*
 *  ======== ti_drivers_config.c ========
 *  Configured TI-Drivers module definitions
 *
 *  DO NOT EDIT - This file is generated for the CC1352P_2_LAUNCHXL
 *  by the SysConfig tool.
 */

#include <stddef.h>
#include <stdint.h>

#ifndef DeviceFamily_CC13X2
#define DeviceFamily_CC13X2
#endif

#include <ti/devices/DeviceFamily.h>

#include "ti_drivers_config.h"

/*
 *  ============================= Display =============================
 */

#include <ti/display/Display.h>
#include <ti/display/DisplayUart.h>

#define CONFIG_Display_COUNT 1

#define Display_UARTBUFFERSIZE 1024
static char displayUARTBuffer[Display_UARTBUFFERSIZE];

DisplayUart_Object displayUartObject;

const DisplayUart_HWAttrs displayUartHWAttrs = {
    .uartIdx      = CONFIG_DISPLAY_UART_MINE,
    .baudRate     = 115200,
    .mutexTimeout = (unsigned int)(-1),
    .strBuf       = displayUARTBuffer,
    .strBufLen    = Display_UARTBUFFERSIZE
};

const Display_Config Display_config[CONFIG_Display_COUNT] = {
    /* CONFIG_Display_0 */
    {
        .fxnTablePtr = &DisplayUartAnsi_fxnTable,
        .object      = &displayUartObject,
        .hwAttrs     = &displayUartHWAttrs
    },
};

const uint_least8_t Display_count = CONFIG_Display_COUNT;

/*
 *  =============================== ADC ===============================
 */

#include <ti/drivers/ADC.h>
#include <ti/drivers/adc/ADCCC26XX.h>

#define CONFIG_ADC_COUNT 2

/*
 *  ======== adcCC26xxObjects ========
 */
ADCCC26XX_Object adcCC26xxObjects[CONFIG_ADC_COUNT];

/*
 *  ======== adcCC26xxHWAttrs ========
 */
const ADCCC26XX_HWAttrs adcCC26xxHWAttrs[CONFIG_ADC_COUNT] = {
    /* CO_OUT */
    {
        .adcDIO              = IOID_23,
        .adcCompBInput       = ADC_COMPB_IN_AUXIO7,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_85P3_US,
        .inputScalingEnabled = true,
        .refVoltage          = 3300000,
        .triggerSource       = ADCCC26XX_TRIGGER_MANUAL,
        .returnAdjustedVal   = true
    },
    /* POWER_POLL */
    {
        .adcDIO              = IOID_24,
        .adcCompBInput       = ADC_COMPB_IN_AUXIO6,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_2P7_US,
        .inputScalingEnabled = true,
        .refVoltage          = 3300000,
        .triggerSource       = ADCCC26XX_TRIGGER_MANUAL,
        .returnAdjustedVal   = false
    },
};

/*
 *  ======== ADC_config ========
 */
const ADC_Config ADC_config[CONFIG_ADC_COUNT] = {
    /* CO_OUT */
    {
        .fxnTablePtr = &ADCCC26XX_fxnTable,
        .object = &adcCC26xxObjects[CO_OUT],
        .hwAttrs = &adcCC26xxHWAttrs[CO_OUT]
    },
    /* POWER_POLL */
    {
        .fxnTablePtr = &ADCCC26XX_fxnTable,
        .object = &adcCC26xxObjects[POWER_POLL],
        .hwAttrs = &adcCC26xxHWAttrs[POWER_POLL]
    },
};

const uint_least8_t CO_OUT_CONST = CO_OUT;
const uint_least8_t POWER_POLL_CONST = POWER_POLL;
const uint_least8_t ADC_count = CONFIG_ADC_COUNT;

/*
 *  =============================== AESCBC ===============================
 */

#include <ti/drivers/AESCBC.h>
#include <ti/drivers/aescbc/AESCBCCC26XX.h>

#define CONFIG_AESCBC_COUNT 1
AESCBCCC26XX_Object aescbcCC26XXObjects[CONFIG_AESCBC_COUNT];

/*
 *  ======== aescbcCC26XXHWAttrs ========
 */
const AESCBCCC26XX_HWAttrs aescbcCC26XXHWAttrs[CONFIG_AESCBC_COUNT] = {
    {
        .intPriority = (~0),
    },
};

const AESCBC_Config AESCBC_config[CONFIG_AESCBC_COUNT] = {
    {   /* CONFIG_AESCBC_0 */
        .object  = &aescbcCC26XXObjects[CONFIG_AESCBC_0],
        .hwAttrs = &aescbcCC26XXHWAttrs[CONFIG_AESCBC_0]
    },
};

const uint_least8_t CONFIG_AESCBC_0_CONST = CONFIG_AESCBC_0;
const uint_least8_t AESCBC_count = CONFIG_AESCBC_COUNT;

/*
 *  =============================== AESCCM ===============================
 */

#include <ti/drivers/AESCCM.h>
#include <ti/drivers/aesccm/AESCCMCC26XX.h>

#define CONFIG_AESCCM_COUNT 1
AESCCMCC26XX_Object aesccmCC26XXObjects[CONFIG_AESCCM_COUNT];

/*
 *  ======== aesccmCC26XXHWAttrs ========
 */
const AESCCMCC26XX_HWAttrs aesccmCC26XXHWAttrs[CONFIG_AESCCM_COUNT] = {
    {
        .intPriority = 0x40,
    },
};

const AESCCM_Config AESCCM_config[CONFIG_AESCCM_COUNT] = {
    {   /* CONFIG_AESCCM_0 */
        .object  = &aesccmCC26XXObjects[CONFIG_AESCCM_0],
        .hwAttrs = &aesccmCC26XXHWAttrs[CONFIG_AESCCM_0]
    },
};

const uint_least8_t CONFIG_AESCCM_0_CONST = CONFIG_AESCCM_0;
const uint_least8_t AESCCM_count = CONFIG_AESCCM_COUNT;

/*
 *  =============================== AESECB ===============================
 */

#include <ti/drivers/AESECB.h>
#include <ti/drivers/aesecb/AESECBCC26XX.h>

#define CONFIG_AESECB_COUNT 1
AESECBCC26XX_Object aesecbCC26XXObjects[CONFIG_AESECB_COUNT];

/*
 *  ======== aesecbCC26XXHWAttrs ========
 */
const AESECBCC26XX_HWAttrs aesecbCC26XXHWAttrs[CONFIG_AESECB_COUNT] = {
    {
        .intPriority = 0x20,
    },
};

const AESECB_Config AESECB_config[CONFIG_AESECB_COUNT] = {
    {   /* CONFIG_AESECB_0 */
        .object  = &aesecbCC26XXObjects[CONFIG_AESECB_0],
        .hwAttrs = &aesecbCC26XXHWAttrs[CONFIG_AESECB_0]
    },
};


const uint_least8_t CONFIG_AESECB_0_CONST = CONFIG_AESECB_0;
const uint_least8_t AESECB_count = CONFIG_AESECB_COUNT;

/*
 *  =============================== DMA ===============================
 */

#include <ti/drivers/dma/UDMACC26XX.h>
#include <ti/devices/cc13x2_cc26x2/driverlib/udma.h>
#include <ti/devices/cc13x2_cc26x2/inc/hw_memmap.h>

UDMACC26XX_Object udmaCC26XXObject;

const UDMACC26XX_HWAttrs udmaCC26XXHWAttrs = {
    .baseAddr        = UDMA0_BASE,
    .powerMngrId     = PowerCC26XX_PERIPH_UDMA,
    .intNum          = INT_DMA_ERR,
    .intPriority     = (~0)
};

const UDMACC26XX_Config UDMACC26XX_config[1] = {
    {
        .object         = &udmaCC26XXObject,
        .hwAttrs        = &udmaCC26XXHWAttrs,
    },
};

/*
 *  =============================== ECDH ===============================
 */

#include <ti/drivers/ECDH.h>
#include <ti/drivers/ecdh/ECDHCC26X2.h>

#define CONFIG_ECDH_COUNT 1

ECDHCC26X2_Object ecdhCC26X2Objects[CONFIG_ECDH_COUNT];

/*
 *  ======== ecdhCC26X2HWAttrs ========
 */
const ECDHCC26X2_HWAttrs ecdhCC26X2HWAttrs[CONFIG_ECDH_COUNT] = {
    {
        .intPriority = (~0),
    },
};

const ECDH_Config ECDH_config[CONFIG_ECDH_COUNT] = {
    {   /* CONFIG_ECDH_0 */
        .object         = &ecdhCC26X2Objects[CONFIG_ECDH_0],
        .hwAttrs        = &ecdhCC26X2HWAttrs[CONFIG_ECDH_0]
    },
};

const uint_least8_t CONFIG_ECDH_0_CONST = CONFIG_ECDH_0;
const uint_least8_t ECDH_count = CONFIG_ECDH_COUNT;

/*
 *  =============================== ECDSA ===============================
 */

#include <ti/drivers/ECDSA.h>
#include <ti/drivers/ecdsa/ECDSACC26X2.h>

#define CONFIG_ECDSA_COUNT 1
ECDSACC26X2_Object ecdsaCC26X2Objects[CONFIG_ECDSA_COUNT];

/*
 *  ======== ecdsaCC26X2HWAttrs ========
 */
const ECDSACC26X2_HWAttrs ecdsaCC26X2HWAttrs[CONFIG_ECDSA_COUNT] = {
    {
        .intPriority = (~0),
        .trngIntPriority = (~0),
    },
};

const ECDSA_Config ECDSA_config[CONFIG_ECDSA_COUNT] = {
    {   /* CONFIG_ECDSA_0 */
        .object         = &ecdsaCC26X2Objects[CONFIG_ECDSA_0],
        .hwAttrs        = &ecdsaCC26X2HWAttrs[CONFIG_ECDSA_0]
    },
};

const uint_least8_t CONFIG_ECDSA_0_CONST = CONFIG_ECDSA_0;
const uint_least8_t ECDSA_count = CONFIG_ECDSA_COUNT;

/*
 *  =============================== ECJPAKE ===============================
 */

#include <ti/drivers/ECJPAKE.h>
#include <ti/drivers/ecjpake/ECJPAKECC26X2.h>

#define CONFIG_ECJPAKE_COUNT 1
ECJPAKECC26X2_Object ecjpakeCC26X2Objects[CONFIG_ECJPAKE_COUNT];

/*
 *  ======== ecjpakeCC26X2HWAttrs ========
 */
const ECJPAKECC26X2_HWAttrs ecjpakeCC26X2HWAttrs[CONFIG_ECJPAKE_COUNT] = {
    {
        .intPriority = (~0),
    },
};

const ECJPAKE_Config ECJPAKE_config[CONFIG_ECJPAKE_COUNT] = {
    {   /* CONFIG_ECJPAKE_0 */
        .object         = &ecjpakeCC26X2Objects[CONFIG_ECJPAKE_0],
        .hwAttrs        = &ecjpakeCC26X2HWAttrs[CONFIG_ECJPAKE_0]
    },
};

const uint_least8_t CONFIG_ECJPAKE_0_CONST = CONFIG_ECJPAKE_0;
const uint_least8_t ECJPAKE_count = CONFIG_ECJPAKE_COUNT;

/*
 *  =============================== GPIO ===============================
 */

#include <ti/drivers/GPIO.h>
#include <ti/drivers/gpio/GPIOCC26XX.h>

#define CONFIG_GPIO_COUNT 3

/*
 *  ======== gpioPinConfigs ========
 *  Array of Pin configurations
 */
GPIO_PinConfig gpioPinConfigs[] = {
    /* PIR_SENSOR */
    GPIOCC26XX_DIO_27 | GPIO_CFG_IN_PD | GPIO_CFG_IN_INT_RISING,
    /* GPIO_ETH_CS */
    GPIOCC26XX_DIO_07 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_MED | GPIO_CFG_OUT_HIGH,
    /* GPIO_RTS */
    GPIOCC26XX_DIO_18 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_MED | GPIO_CFG_OUT_HIGH,
};

/*
 *  ======== gpioCallbackFunctions ========
 *  Array of callback function pointers
 *
 *  NOTE: Unused callback entries can be omitted from the callbacks array to
 *  reduce memory usage by enabling callback table optimization
 *  (GPIO.optimizeCallbackTableSize = true)
 */
GPIO_CallbackFxn gpioCallbackFunctions[] = {
    /* PIR_SENSOR */
    NULL,
    /* GPIO_ETH_CS */
    NULL,
    /* GPIO_RTS */
    NULL,
};

const uint_least8_t PIR_SENSOR_CONST = PIR_SENSOR;
const uint_least8_t GPIO_ETH_CS_CONST = GPIO_ETH_CS;
const uint_least8_t GPIO_RTS_CONST = GPIO_RTS;

/*
 *  ======== GPIOCC26XX_config ========
 */
const GPIOCC26XX_Config GPIOCC26XX_config = {
    .pinConfigs = (GPIO_PinConfig *)gpioPinConfigs,
    .callbacks = (GPIO_CallbackFxn *)gpioCallbackFunctions,
    .numberOfPinConfigs = 3,
    .numberOfCallbacks = 3,
    .intPriority = (~0)
};

/*
 *  =============================== I2C ===============================
 */

#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CCC26XX.h>
#include <ti/drivers/power/PowerCC26XX.h>

#define CONFIG_I2C_COUNT 1

/*
 *  ======== i2cCC26xxObjects ========
 */
I2CCC26XX_Object i2cCC26xxObjects[CONFIG_I2C_COUNT];

/*
 *  ======== i2cCC26xxHWAttrs ========
 */
const I2CCC26XX_HWAttrsV1 i2cCC26xxHWAttrs[CONFIG_I2C_COUNT] = {
    /* CONFIG_I2C_0 */
    {
        .baseAddr    = I2C0_BASE,
        .powerMngrId = PowerCC26XX_PERIPH_I2C0,
        .intNum      = INT_I2C_IRQ,
        .intPriority = (~0),
        .swiPriority = 0,
        .sclPin      = IOID_20,
        .sdaPin      = IOID_5
    },
};

/*
 *  ======== I2C_config ========
 */
const I2C_Config I2C_config[CONFIG_I2C_COUNT] = {
    /* CONFIG_I2C_0 */
    {
        .object      = &i2cCC26xxObjects[CONFIG_I2C_0],
        .hwAttrs     = &i2cCC26xxHWAttrs[CONFIG_I2C_0]
    },
};

const uint_least8_t CONFIG_I2C_0_CONST = CONFIG_I2C_0;
const uint_least8_t I2C_count = CONFIG_I2C_COUNT;

/*
 *  =============================== NVS ===============================
 */

#include <ti/drivers/NVS.h>
#include <ti/drivers/nvs/NVSCC26XX.h>

/*
 *  NVSCC26XX Internal NVS flash region definitions
 *
 * Place uninitialized char arrays at addresses
 * corresponding to the 'regionBase' addresses defined in
 * the configured NVS regions. These arrays are used as
 * place holders so that the linker will not place other
 * content there.
 *
 * For GCC targets, the char arrays are each placed into
 * the shared ".nvs" section. The user must add content to
 * their GCC linker command file to place the .nvs section
 * at the lowest 'regionBase' address specified in their NVS
 * regions.
 */

#if defined(__TI_COMPILER_VERSION__) || defined(__clang__)

static char flashBuf0[0x4000] __attribute__ ((retain, noinit, location(0x52000)));

#elif defined(__IAR_SYSTEMS_ICC__)

__no_init static char flashBuf0[0x4000] @ 0x52000;

#elif defined(__GNUC__)

__attribute__ ((section (".nvs")))
static char flashBuf0[0x4000];

#endif

NVSCC26XX_Object nvsCC26XXObjects[1];

static const NVSCC26XX_HWAttrs nvsCC26XXHWAttrs[1] = {
    /* CONFIG_NVSINTERNAL */
    {
        .regionBase = (void *) flashBuf0,
        .regionSize = 0x4000
    },
};

#define CONFIG_NVS_COUNT 1

const NVS_Config NVS_config[CONFIG_NVS_COUNT] = {
    /* CONFIG_NVSINTERNAL */
    {
        .fxnTablePtr = &NVSCC26XX_fxnTable,
        .object = &nvsCC26XXObjects[0],
        .hwAttrs = &nvsCC26XXHWAttrs[0],
    },
};

const uint_least8_t CONFIG_NVSINTERNAL_CONST = CONFIG_NVSINTERNAL;
const uint_least8_t NVS_count = CONFIG_NVS_COUNT;

/*
 *  =============================== PIN ===============================
 */
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>

#define CONFIG_PIN_COUNT 19

const PIN_Config BoardGpioInitTable[CONFIG_PIN_COUNT + 1] = {
    /* Parent Signal: CONFIG_DISPLAY_UART_MINE TX, (DIO12) */
    CONFIG_PIN_5 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MED,
    /* Parent Signal: CONFIG_DISPLAY_UART_MINE RX, (DIO13) */
    CONFIG_PIN_6 | PIN_INPUT_EN | PIN_PULLDOWN | PIN_IRQ_DIS,
    /* Parent Signal: CO_OUT ADC Pin, (DIO23) */
    CONFIG_PIN_7 | PIN_INPUT_EN | PIN_NOPULL | PIN_IRQ_DIS,
    /* Parent Signal: POWER_POLL ADC Pin, (DIO24) */
    CONFIG_PIN_9 | PIN_INPUT_EN | PIN_NOPULL | PIN_IRQ_DIS,
    /* Parent Signal: PIR_SENSOR GPIO Pin, (DIO27) */
    CONFIG_PIN_8 | PIN_INPUT_EN | PIN_PULLDOWN | PIN_IRQ_DIS,
    /* Parent Signal: GPIO_ETH_CS GPIO Pin, (DIO7) */
    CONFIG_PIN_14 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MED,
    /* Parent Signal: GPIO_RTS GPIO Pin, (DIO18) */
    CONFIG_PIN_11 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MED,
    /* Parent Signal: CONFIG_I2C_0 SDA, (DIO5) */
    CONFIG_PIN_1 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_DIS,
    /* Parent Signal: CONFIG_I2C_0 SCL, (DIO20) */
    CONFIG_PIN_2 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_DIS,
    /* Parent Signal: CONFIG_GPTIMER_0 PWM Pin, (DIO15) */
    CONFIG_PIN_10 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MED,
    /* Parent Signal: CONFIG_SPI_LEDBOARD SCLK, (DIO16) */
    CONFIG_PIN_21 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MED,
    /* Parent Signal: CONFIG_SPI_LEDBOARD MISO, (DIO11) */
    CONFIG_PIN_20 | PIN_INPUT_EN | PIN_NOPULL | PIN_IRQ_DIS,
    /* Parent Signal: CONFIG_SPI_LEDBOARD MOSI, (DIO26) */
    CONFIG_PIN_4 | PIN_INPUT_EN | PIN_NOPULL | PIN_IRQ_DIS,
    /* Parent Signal: CONFIG_SPI_W5500 SCLK, (DIO10) */
    CONFIG_PIN_15 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MED,
    /* Parent Signal: CONFIG_SPI_W5500 MISO, (DIO8) */
    CONFIG_PIN_13 | PIN_INPUT_EN | PIN_NOPULL | PIN_IRQ_DIS,
    /* Parent Signal: CONFIG_SPI_W5500 MOSI, (DIO9) */
    CONFIG_PIN_12 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MED,
    /* SKY13317-373LF RF Antenna Switch, Parent Signal: /ti/drivers/RF RF Antenna Pin 0, (DIO28) */
    CONFIG_RF_24GHZ | PIN_INPUT_EN | PIN_NOPULL | PIN_IRQ_DIS,
    /* SKY13317-373LF RF Antenna Switch, Parent Signal: /ti/drivers/RF RF Antenna Pin 1, (DIO29) */
    CONFIG_RF_HIGH_PA | PIN_INPUT_EN | PIN_NOPULL | PIN_IRQ_DIS,
    /* SKY13317-373LF RF Antenna Switch, Parent Signal: /ti/drivers/RF RF Antenna Pin 2, (DIO30) */
    CONFIG_RF_SUB1GHZ | PIN_INPUT_EN | PIN_NOPULL | PIN_IRQ_DIS,

    PIN_TERMINATE
};

const PINCC26XX_HWAttrs PINCC26XX_hwAttrs = {
    .intPriority = (~0),
    .swiPriority = 0
};

/*
 *  =============================== PWM ===============================
 */

#include <ti/drivers/PWM.h>
#include <ti/drivers/pwm/PWMTimerCC26XX.h>

/* include driverlib definitions */
#include <ti/devices/cc13x2_cc26x2/driverlib/ioc.h>
#include <ti/devices/cc13x2_cc26x2/inc/hw_ints.h>
#include <ti/devices/cc13x2_cc26x2/inc/hw_memmap.h>

#define CONFIG_PWM_COUNT 1

/*
 *  ======== pwmCC26XXObjects ========
 */
PWMTimerCC26XX_Object pwmTimerCC26XXObjects[CONFIG_PWM_COUNT];

/*
 *  ======== pwmCC26XXHWAttrs ========
 */
const PWMTimerCC26XX_HwAttrs pwmTimerCC26XXHWAttrs[CONFIG_PWM_COUNT] = {
    /* CONFIG_PWM_4KHZ */
    {
        .pwmPin = IOID_15,
        .gpTimerUnit = CONFIG_GPTIMER_0
    },
};

/*
 *  ======== PWM_config ========
 */
const PWM_Config PWM_config[CONFIG_PWM_COUNT] = {
    /* CONFIG_PWM_4KHZ */
    {
        .fxnTablePtr = &PWMTimerCC26XX_fxnTable,
        .object = &pwmTimerCC26XXObjects[CONFIG_PWM_4KHZ],
        .hwAttrs = &pwmTimerCC26XXHWAttrs[CONFIG_PWM_4KHZ]
    },
};

const uint_least8_t CONFIG_PWM_4KHZ_CONST = CONFIG_PWM_4KHZ;
const uint_least8_t PWM_count = CONFIG_PWM_COUNT;

/*
 *  =============================== Power ===============================
 */
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26X2.h>
#include "ti_drivers_config.h"

extern void PowerCC26XX_standbyPolicy(void);
extern bool PowerCC26XX_calibrate(unsigned int);

const PowerCC26X2_Config PowerCC26X2_config = {
    .enablePolicy             = true,
    .policyInitFxn            = NULL,
    .policyFxn                = PowerCC26XX_standbyPolicy,
    .calibrateFxn             = PowerCC26XX_calibrate,
    .calibrateRCOSC_LF        = true,
    .calibrateRCOSC_HF        = true,
    .enableTCXOFxn            = NULL
};


/*
 *  =============================== RF Driver ===============================
 */
#include <ti/drivers/rf/RF.h>

/*
 * RF driver callback function, called by the driver on global driver events.
 */
static void RF_globalCallbackFunction (RF_Handle client, RF_GlobalEvent events, void* arg);

/*
 * Callback function to handle custom / application specific behavior
 */
extern void __attribute__((weak)) rfDriverCallback (RF_Handle client, RF_GlobalEvent events, void *arg);

/*
 * Callback function to handle antenna switching
 */
extern void __attribute__((weak)) rfDriverCallbackAntennaSwitching (RF_Handle client, RF_GlobalEvent events, void *arg);

/*
 * Platform-specific driver configuration
 */
const RFCC26XX_HWAttrsV2 RFCC26XX_hwAttrs = {
    .hwiPriority        = (~0),
    .swiPriority        = (uint8_t)0,
    .xoscHfAlwaysNeeded = true,
    .globalCallback     = &RF_globalCallbackFunction,
    .globalEventMask    = RF_GlobalEventInit | RF_GlobalEventRadioPowerDown | RF_GlobalEventRadioSetup
};

/*
 *  ======== RF_globalCallbackFunction ========
 *  This function is called by the driver on global driver events.
 *  It will call specific callback functions to further handle the triggering events.
 */
static void RF_globalCallbackFunction(RF_Handle client, RF_GlobalEvent events, void *arg)
{
    rfDriverCallback(client, events, arg);
    rfDriverCallbackAntennaSwitching(client, events, arg);
}

/*
 *  ======== rfDriverCallback ========
 *  Handle events triggered by the RF driver for custom / application specific behavior.
 */
void __attribute__((weak)) rfDriverCallback(RF_Handle client, RF_GlobalEvent events, void *arg)
{
    /* ======== PLEASE READ THIS ========
    *
    * This function is declared weak for the application to override it.
    * A new definition of 'rfDriverCallback' is required if you want to
    * handle the events listed in '.globalEventMask'.
    *
    * Please copy this function definition to create your own, but make
    * sure to remove '__attribute__((weak))' for your definition.
    *
    * According to '.globalEventMask', this function will be triggered by:
    *   - RF_GlobalEventInit
    *   - RF_GlobalEventRadioPowerDown
    *   - RF_GlobalEventRadioSetup
    * 
    * An example of how to handle these events would be:
    * 
    *   --- Code snippet begin ---
    * 
    *   if(events & RF_GlobalEventInit) {
    *       // Perform action for this event
    *   }
    *   else if (events & RF_GlobalEventRadioPowerDown) {
    *       // Perform action for this event
    *   }
    *   else if (events & RF_GlobalEventRadioSetup) {
    *       // Perform action for this event
    *   }
    * 
    *   --- Code snippet end ---
    */
}


/*
 * ======== Antenna switching ========
 */
static PIN_Handle antennaPins;
static PIN_State antennaState;

static void initAntennaSwitch()
{
    PIN_Config antennaConfig[] = {
        CONFIG_RF_24GHZ | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,      /* Path disabled */
        CONFIG_RF_HIGH_PA | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,      /* Path disabled */
        CONFIG_RF_SUB1GHZ | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,      /* Path disabled */
        PIN_TERMINATE
    };
    antennaPins = PIN_open(&antennaState, antennaConfig);
}

/*
 * ======== rfDriverCallbackAntennaSwitching ========
 * Sets up the antenna switch depending on the current PHY configuration.
 * 
 * Truth table:
 *
 * Path        DIO28 DIO29 DIO30
 * =========== ===== ===== =====
 * Off         0     0     0
 * Sub-1 GHz   0     0     1
 * 2.4 GHz     1     0     0
 * 20 dBm TX   0     1     0
 */
void __attribute__((weak)) rfDriverCallbackAntennaSwitching(RF_Handle client, RF_GlobalEvent events, void *arg)
{
    /* Protect against repeated RF_init */
    static bool initialized = false;

    /* Local variable. */
    bool    sub1GHz   = false;
    uint8_t loDivider = 0;

    if (!initialized && events & RF_GlobalEventInit) {
        /* Don't do it again */
        initialized = true;
        initAntennaSwitch();
    }
    else if (events & RF_GlobalEventRadioSetup) {
        /* Switch off all paths. */
        PINCC26XX_setOutputValue(CONFIG_RF_24GHZ, 0);
        PINCC26XX_setOutputValue(CONFIG_RF_HIGH_PA, 0);
        PINCC26XX_setOutputValue(CONFIG_RF_SUB1GHZ, 0);

        /* Decode the current PA configuration. */
        RF_TxPowerTable_PAType paType = (RF_TxPowerTable_PAType)RF_getTxPower(client).paType;

        /* Decode the generic argument as a setup command. */
        RF_RadioSetup* setupCommand = (RF_RadioSetup*)arg;

        switch (setupCommand->common.commandNo) {
            case (CMD_RADIO_SETUP):
            case (CMD_BLE5_RADIO_SETUP):
                    loDivider = RF_LODIVIDER_MASK & setupCommand->common.loDivider;

                    /* Sub-1GHz front-end. */
                    if (loDivider != 0) {
                        sub1GHz = true;
                    }
                    break;
            case (CMD_PROP_RADIO_DIV_SETUP):
                    loDivider = RF_LODIVIDER_MASK & setupCommand->prop_div.loDivider;

                    /* Sub-1GHz front-end. */
                    if (loDivider != 0) {
                        sub1GHz = true;
                    }
                    break;
            default:break;
        }

        if (sub1GHz) {
            /* Sub-1 GHz */
            if (paType == RF_TxPowerTable_HighPA) {
                /* PA enable --> HIGH PA
                 * LNA enable --> Sub-1 GHz
                 */
                PINCC26XX_setMux(antennaPins, CONFIG_RF_24GHZ, PINCC26XX_MUX_GPIO);
                /* Note: RFC_GPO3 is a work-around because the RFC_GPO1 (PA enable signal) is sometimes not
                         de-asserted on CC1352 Rev A. */
                PINCC26XX_setMux(antennaPins, CONFIG_RF_HIGH_PA, PINCC26XX_MUX_RFC_GPO3);
                PINCC26XX_setMux(antennaPins, CONFIG_RF_SUB1GHZ, PINCC26XX_MUX_RFC_GPO0);
            } else {
                /* RF core active --> Sub-1 GHz */
                PINCC26XX_setMux(antennaPins, CONFIG_RF_24GHZ, PINCC26XX_MUX_GPIO);
                PINCC26XX_setMux(antennaPins, CONFIG_RF_HIGH_PA, PINCC26XX_MUX_GPIO);
                PINCC26XX_setMux(antennaPins, CONFIG_RF_SUB1GHZ, PINCC26XX_MUX_GPIO);
                PINCC26XX_setOutputValue(CONFIG_RF_SUB1GHZ, 1);
            }
        } else {
            /* 2.4 GHz */
            if (paType == RF_TxPowerTable_HighPA)
            {
                /* PA enable --> HIGH PA
                 * LNA enable --> 2.4 GHz
                 */
                PINCC26XX_setMux(antennaPins, CONFIG_RF_24GHZ, PINCC26XX_MUX_RFC_GPO0);
                /* Note: RFC_GPO3 is a work-around because the RFC_GPO1 (PA enable signal) is sometimes not
                         de-asserted on CC1352 Rev A. */
                PINCC26XX_setMux(antennaPins, CONFIG_RF_HIGH_PA, PINCC26XX_MUX_RFC_GPO3);
                PINCC26XX_setMux(antennaPins, CONFIG_RF_SUB1GHZ, PINCC26XX_MUX_GPIO);
            } else {
                /* RF core active --> 2.4 GHz */
                PINCC26XX_setMux(antennaPins, CONFIG_RF_24GHZ, PINCC26XX_MUX_GPIO);
                PINCC26XX_setMux(antennaPins, CONFIG_RF_HIGH_PA, PINCC26XX_MUX_GPIO);
                PINCC26XX_setMux(antennaPins, CONFIG_RF_SUB1GHZ, PINCC26XX_MUX_GPIO);
                PINCC26XX_setOutputValue(CONFIG_RF_24GHZ, 1);
            }
        }
    }
    else if (events & RF_GlobalEventRadioPowerDown) {
        /* Switch off all paths. */
        PINCC26XX_setOutputValue(CONFIG_RF_24GHZ, 0);
        PINCC26XX_setOutputValue(CONFIG_RF_HIGH_PA, 0);
        PINCC26XX_setOutputValue(CONFIG_RF_SUB1GHZ, 0);

        /* Reset the IO multiplexer to GPIO functionality */
        PINCC26XX_setMux(antennaPins, CONFIG_RF_24GHZ, PINCC26XX_MUX_GPIO);
        PINCC26XX_setMux(antennaPins, CONFIG_RF_HIGH_PA, PINCC26XX_MUX_GPIO);
        PINCC26XX_setMux(antennaPins, CONFIG_RF_SUB1GHZ, PINCC26XX_MUX_GPIO);
    }
}

/*
 *  =============================== SHA2 ===============================
 */

#include <ti/drivers/SHA2.h>
#include <ti/drivers/sha2/SHA2CC26X2.h>

#define CONFIG_SHA2_COUNT 1

SHA2CC26X2_Object sha2CC26X2Objects[CONFIG_SHA2_COUNT];

/*
 *  ======== sha2CC26X2HWAttrs ========
 */
const SHA2CC26X2_HWAttrs sha2CC26X2HWAttrs[CONFIG_SHA2_COUNT] = {
    {
        .intPriority = (~0),
    },
};

const SHA2_Config SHA2_config[CONFIG_SHA2_COUNT] = {
    {   /* CONFIG_SHA2_0 */
        .object         = &sha2CC26X2Objects[CONFIG_SHA2_0],
        .hwAttrs        = &sha2CC26X2HWAttrs[CONFIG_SHA2_0]
    },
};

const uint_least8_t CONFIG_SHA2_0_CONST = CONFIG_SHA2_0;
const uint_least8_t SHA2_count = CONFIG_SHA2_COUNT;

/*
 *  =============================== SPI DMA ===============================
 */
#include <ti/drivers/SPI.h>
#include <ti/drivers/spi/SPICC26X2DMA.h>

#define CONFIG_SPI_COUNT 2

/*
 *  ======== spiCC26X2DMAObjects ========
 */
SPICC26X2DMA_Object spiCC26X2DMAObjects[CONFIG_SPI_COUNT];

/*
 *  ======== spiCC26X2DMAHWAttrs ========
 */
const SPICC26X2DMA_HWAttrs spiCC26X2DMAHWAttrs[CONFIG_SPI_COUNT] = {
    /* CONFIG_SPI_LEDBOARD */
    {
        .baseAddr = SSI0_BASE,
        .intNum = INT_SSI0_COMB,
        .intPriority = (~0),
        .swiPriority = 0,
        .powerMngrId = PowerCC26XX_PERIPH_SSI0,
        .defaultTxBufValue = ~0,
        .rxChannelBitMask = 1<<UDMA_CHAN_SSI0_RX,
        .txChannelBitMask = 1<<UDMA_CHAN_SSI0_TX,
        .minDmaTransferSize = 10,
        .mosiPin = IOID_26,
        .misoPin = IOID_11,
        .clkPin  = IOID_16,
        .csnPin  = PIN_UNASSIGNED
    },
    /* CONFIG_SPI_W5500 */
    {
        .baseAddr = SSI1_BASE,
        .intNum = INT_SSI1_COMB,
        .intPriority = (~0),
        .swiPriority = 0,
        .powerMngrId = PowerCC26XX_PERIPH_SSI1,
        .defaultTxBufValue = ~0,
        .rxChannelBitMask = 1<<UDMA_CHAN_SSI1_RX,
        .txChannelBitMask = 1<<UDMA_CHAN_SSI1_TX,
        .minDmaTransferSize = 10,
        .mosiPin = IOID_9,
        .misoPin = IOID_8,
        .clkPin  = IOID_10,
        .csnPin  = PIN_UNASSIGNED
    },
};

/*
 *  ======== SPI_config ========
 */
const SPI_Config SPI_config[CONFIG_SPI_COUNT] = {
    /* CONFIG_SPI_LEDBOARD */
    {
        .fxnTablePtr = &SPICC26X2DMA_fxnTable,
        .object = &spiCC26X2DMAObjects[CONFIG_SPI_LEDBOARD],
        .hwAttrs = &spiCC26X2DMAHWAttrs[CONFIG_SPI_LEDBOARD]
    },
    /* CONFIG_SPI_W5500 */
    {
        .fxnTablePtr = &SPICC26X2DMA_fxnTable,
        .object = &spiCC26X2DMAObjects[CONFIG_SPI_W5500],
        .hwAttrs = &spiCC26X2DMAHWAttrs[CONFIG_SPI_W5500]
    },
};

const uint_least8_t CONFIG_SPI_LEDBOARD_CONST = CONFIG_SPI_LEDBOARD;
const uint_least8_t CONFIG_SPI_W5500_CONST = CONFIG_SPI_W5500;
const uint_least8_t SPI_count = CONFIG_SPI_COUNT;

/*
 *  =============================== TRNG ===============================
 */

#include <ti/drivers/TRNG.h>
#include <ti/drivers/trng/TRNGCC26XX.h>

#define CONFIG_TRNG_COUNT 1

TRNGCC26XX_Object trngCC26XXObjects[CONFIG_TRNG_COUNT];

/*
 *  ======== trngCC26XXHWAttrs ========
 */
static const TRNGCC26XX_HWAttrs trngCC26XXHWAttrs[CONFIG_TRNG_COUNT] = {
    {
        .intPriority = (~0),
        .swiPriority = 0,
        .samplesPerCycle = 240000
    },
};

const TRNG_Config TRNG_config[CONFIG_TRNG_COUNT] = {
    {   /* CONFIG_TRNG_0 */
        .object         = &trngCC26XXObjects[CONFIG_TRNG_0],
        .hwAttrs        = &trngCC26XXHWAttrs[CONFIG_TRNG_0]
    },
};

const uint_least8_t CONFIG_TRNG_0_CONST = CONFIG_TRNG_0;
const uint_least8_t TRNG_count = CONFIG_TRNG_COUNT;

/*
 *  =============================== Temperature ===============================
 */
#include <ti/drivers/Temperature.h>
#include <ti/drivers/temperature/TemperatureCC26X2.h>

const TemperatureCC26X2_Config TemperatureCC26X2_config = {
    .intPriority = (~0),
};

/*
 *  =============================== UART ===============================
 */

#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTCC26XX.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26X2.h>
#include <ti/devices/cc13x2_cc26x2/inc/hw_memmap.h>
#include <ti/devices/cc13x2_cc26x2/inc/hw_ints.h>

#define CONFIG_UART_COUNT 1

UARTCC26XX_Object uartCC26XXObjects[CONFIG_UART_COUNT];

static unsigned char uartCC26XXRingBuffer0[32];

static const UARTCC26XX_HWAttrsV2 uartCC26XXHWAttrs[CONFIG_UART_COUNT] = {
  {
    .baseAddr           = UART1_BASE,
    .intNum             = INT_UART1_COMB,
    .intPriority        = (~0),
    .swiPriority        = 0,
    .powerMngrId        = PowerCC26X2_PERIPH_UART1,
    .ringBufPtr         = uartCC26XXRingBuffer0,
    .ringBufSize        = sizeof(uartCC26XXRingBuffer0),
    .rxPin              = IOID_13,
    .txPin              = IOID_12,
    .ctsPin             = PIN_UNASSIGNED,
    .rtsPin             = PIN_UNASSIGNED,
    .txIntFifoThr       = UARTCC26XX_FIFO_THRESHOLD_1_8,
    .rxIntFifoThr       = UARTCC26XX_FIFO_THRESHOLD_4_8,
    .errorFxn           = NULL
  },
};

const UART_Config UART_config[CONFIG_UART_COUNT] = {
    {   /* CONFIG_DISPLAY_UART_MINE */
        .fxnTablePtr = &UARTCC26XX_fxnTable,
        .object      = &uartCC26XXObjects[CONFIG_DISPLAY_UART_MINE],
        .hwAttrs     = &uartCC26XXHWAttrs[CONFIG_DISPLAY_UART_MINE]
    },
};

const uint_least8_t CONFIG_DISPLAY_UART_MINE_CONST = CONFIG_DISPLAY_UART_MINE;
const uint_least8_t UART_count = CONFIG_UART_COUNT;

/*
 *  =============================== GPTimer ===============================
 */

#include <ti/drivers/timer/GPTimerCC26XX.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/devices/cc13x2_cc26x2/inc/hw_memmap.h>
#include <ti/devices/cc13x2_cc26x2/inc/hw_ints.h>

#define CONFIG_GPTIMER_COUNT 1

/*
 *  ======== gptimerCC26XXObjects ========
 */
GPTimerCC26XX_Object gptimerCC26XXObjects[CONFIG_GPTIMER_COUNT];

/*
 *  ======== gptimerCC26XXHWAttrs ========
 */
const GPTimerCC26XX_HWAttrs gptimerCC26XXHWAttrs[CONFIG_GPTIMER_COUNT] = {
    /* CONFIG_GPTIMER_0, used by CONFIG_PWM_4KHZ */
    {
        .baseAddr = GPT0_BASE,
        .intNum      = INT_GPT0A,
        .intPriority = (~0),
        .powerMngrId = PowerCC26XX_PERIPH_GPT0,
        .pinMux      = GPT_PIN_0A
    },
};

/*
 *  ======== GPTimer_config ========
 */
const GPTimerCC26XX_Config GPTimerCC26XX_config[CONFIG_GPTIMER_COUNT] = {
    /* CONFIG_GPTIMER_0 */
    {
        .object    = &gptimerCC26XXObjects[CONFIG_GPTIMER_0],
        .hwAttrs   = &gptimerCC26XXHWAttrs[CONFIG_GPTIMER_0],
        .timerPart = GPT_A
    },
};

const uint_least8_t CONFIG_GPTIMER_0_CONST = CONFIG_GPTIMER_0;
const uint_least8_t GPTimer_count = CONFIG_GPTIMER_COUNT;

#include <stdbool.h>

#include <ti/devices/cc13x2_cc26x2/driverlib/ioc.h>
#include <ti/devices/cc13x2_cc26x2/driverlib/cpu.h>

#include <ti/drivers/pin/PINCC26XX.h>

/*
 *  ======== Board_sendExtFlashByte ========
 */
void Board_sendExtFlashByte(PIN_Handle pinHandle, uint8_t byte)
{
    uint8_t i;

    /* SPI Flash CS */
    PIN_setOutputValue(pinHandle, IOID_20, 0);

    for (i = 0; i < 8; i++) {
        PIN_setOutputValue(pinHandle, IOID_10, 0);  /* SPI Flash CLK */

        /* SPI Flash MOSI */
        PIN_setOutputValue(pinHandle, IOID_9, (byte >> (7 - i)) & 0x01);
        PIN_setOutputValue(pinHandle, IOID_10, 1);  /* SPI Flash CLK */

        /*
         * Waste a few cycles to keep the CLK high for at
         * least 45% of the period.
         * 3 cycles per loop: 8 loops @ 48 Mhz = 0.5 us.
         */
        CPUdelay(8);
    }

    PIN_setOutputValue(pinHandle, IOID_10, 0);  /* CLK */
    PIN_setOutputValue(pinHandle, IOID_20, 1);  /* CS */

    /*
     * Keep CS high at least 40 us
     * 3 cycles per loop: 700 loops @ 48 Mhz ~= 44 us
     */
    CPUdelay(700);
}

/*
 *  ======== Board_wakeUpExtFlash ========
 */
void Board_wakeUpExtFlash(void)
{
    PIN_Config extFlashPinTable[] = {
        /* SPI Flash CS */
        IOID_20 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL |
                PIN_INPUT_DIS | PIN_DRVSTR_MED,
        PIN_TERMINATE
    };
    PIN_State extFlashPinState;
    PIN_Handle extFlashPinHandle = PIN_open(&extFlashPinState, extFlashPinTable);

    /*
     *  To wake up we need to toggle the chip select at
     *  least 20 ns and ten wait at least 35 us.
     */

    /* Toggle chip select for ~20ns to wake ext. flash */
    PIN_setOutputValue(extFlashPinHandle, IOID_20, 0);
    /* 3 cycles per loop: 1 loop @ 48 Mhz ~= 62 ns */
    CPUdelay(1);
    PIN_setOutputValue(extFlashPinHandle, IOID_20, 1);
    /* 3 cycles per loop: 560 loops @ 48 Mhz ~= 35 us */
    CPUdelay(560);

    PIN_close(extFlashPinHandle);
}

/*
 *  ======== Board_shutDownExtFlash ========
 */
void Board_shutDownExtFlash(void)
{
    /*
     *  To be sure we are putting the flash into sleep and not waking it,
     *  we first have to make a wake up call
     */
    Board_wakeUpExtFlash();

    PIN_Config extFlashPinTable[] = {
        /* SPI Flash CS*/
        IOID_20 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL |
                PIN_INPUT_DIS | PIN_DRVSTR_MED,
        /* SPI Flash CLK */
        IOID_10 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL |
                PIN_INPUT_DIS | PIN_DRVSTR_MED,
        /* SPI Flash MOSI */
        IOID_9 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL |
                PIN_INPUT_DIS | PIN_DRVSTR_MED,
        /* SPI Flash MISO */
        IOID_8 | PIN_INPUT_EN | PIN_PULLDOWN,
        PIN_TERMINATE
    };
    PIN_State extFlashPinState;
    PIN_Handle extFlashPinHandle = PIN_open(&extFlashPinState, extFlashPinTable);

    uint8_t extFlashShutdown = 0xB9;

    Board_sendExtFlashByte(extFlashPinHandle, extFlashShutdown);

    PIN_close(extFlashPinHandle);
}


#include <ti/drivers/Board.h>

/*
 *  ======== Board_initHook ========
 *  Perform any board-specific initialization needed at startup.  This
 *  function is declared weak to allow applications to override it if needed.
 */
void __attribute__((weak)) Board_initHook(void)
{
}

/*
 *  ======== Board_init ========
 *  Perform any initialization needed before using any board APIs
 */
void Board_init(void)
{
    /* ==== /ti/drivers/Power initialization ==== */
    Power_init();

    /* ==== /ti/drivers/PIN initialization ==== */
    if (PIN_init(BoardGpioInitTable) != PIN_SUCCESS) {
        /* Error with PIN_init */
        while (1);
    }
    /* ==== /ti/devices/CCFGTemplate initialization ==== */

    /* ==== /ti/drivers/RF initialization ==== */


    Board_shutDownExtFlash();

    Board_initHook();
}

