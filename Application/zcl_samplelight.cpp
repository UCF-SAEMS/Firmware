/**************************************************************************************************
  Filename:       zcl_sampleLight.c
  Revised:        $Date: 2014-10-24 16:04:46 -0700 (Fri, 24 Oct 2014) $
  Revision:       $Revision: 40796 $


  Description:    Zigbee Cluster Library - sample light application.


  Copyright 2006-2014 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
  This application implements a ZigBee Light, based on Z-Stack 3.0. It can be configured as an
  On/Off light or as a dimmable light, by undefining or defining ZCL_LEVEL_CTRL, respectively.

  This application is based on the common sample-application user interface. Please see the main
  comment in zcl_sampleapp_ui.c. The rest of this comment describes only the content specific for
  this sample application.

  Application-specific UI peripherals being used:

  - LEDs:
    LED1 reflect the current light state (On / Off accordingly).

  Application-specific menu system:

    <TOGGLE LIGHT> Toggle the local light and display its status and level
      Press OK to toggle the local light on and off.
      This screen shows the following information
        Line1: (only populated if ZCL_LEVEL_CTRL is defined)
          LEVEL XXX - xxx is the current level of the light if the light state is ON, or the target level
            of the light when the light state is off. The target level is the level that the light will be
            set to when it is switched from off to on using the on or the toggle commands.
        Line2:
          LIGHT OFF / ON: shows the current state of the light.
      Note when ZCL_LEVEL_CTRL is enabled:
        - If the light state is ON and the light level is X, and then the light receives the OFF or TOGGLE
          commands: The level will decrease gradually until it reaches 1, and only then the light state will
          be changed to OFF. The level then will be restored to X, with the state staying OFF. At this stage
          the light is not lighting, and the level represent the target level for the next ON or TOGGLE
          commands.
        - If the light state is OFF and the light level is X, and then the light receives the ON or TOGGLE
          commands; The level will be set to 1, the light state will be set to ON, and then the level will
          increase gradually until it reaches level X.
        - Any level-setting command will affect the level directly, and may also affect the on/off state,
          depending on the command's arguments.

*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#define CUI_DISABLE

#include <stdint.h>
#include <stddef.h>
#include <unistd.h>
#include <stdio.h>
#include <math.h>

#include "rom_jt_154.h"
#include "zcomdef.h"
#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"

#include "zcl_samplelight.h"
#include <string.h>
#include "bdb_interface.h"

#include "ti_drivers_config.h"
#include "nvintf.h"
#include "zstackmsg.h"
#include "zcl_port.h"

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>
#include "zstackapi.h"
#include "util_timer.h"

#include <ti/drivers/apps/Button.h>
#include <ti/drivers/apps/LED.h>
#include <ti/drivers/PWM.h>

// *********************** SAEMS-Specific Includes ***********************
#include "zcl_ms.h"               // Sensor Clusters
#include "zcl_lighting.h"         // Color Control Cluster
#include <ti/drivers/GPIO.h>      // GPIO (control, init) for sensor
#include <ti/drivers/I2C.h>       // I2C for measuring and polling
#include <ti/drivers/PWM.h>       // PWM for LED Control (?)
// ***********************************************************************

#include "ti_zstack_config.h"


#if !defined (DISABLE_GREENPOWER_BASIC_PROXY) && (ZG_BUILD_RTR_TYPE)
#include "gp_common.h"
#endif

#if defined ( BDB_TL_INITIATOR )
#ifdef __cplusplus
extern "C" {
#endif
#include "touchlink_initiator_app.h"
#elif defined ( BDB_TL_TARGET )
#include "touchlink_target_app.h"
#endif

#if defined(USE_DMM) && defined(BLE_START)
#include "ti_dmm_application_policy.h"
#include "remote_display.h"
#include "mac_util.h"
#endif // defined(USE_DMM) && defined(BLE_START)
#ifdef PER_TEST
#include "per_test.h"
#endif
#if defined (ENABLE_GREENPOWER_COMBO_BASIC)
#include "gp_sink.h"
#endif


#if defined (ENABLE_GREENPOWER_COMBO_BASIC)
#if ZG_BUILD_ENDDEVICE_TYPE
#error: End devices cannot have Green Power Combo enabled
#endif
#endif


/* Display Header files */
#include <ti/display/Display.h>
#include <ti/display/DisplayUart.h>
#include <ti/display/DisplayExt.h>
#include <ti/display/AnsiColor.h>
#include "stdio.h"
#include "stdint.h"
#include "stdbool.h"
#include "stddef.h"

// Import ADC Driver definitions
#include <ti/drivers/ADC.h>


  // Import ADC Driver definitions
//#include <ti/drivers/ADC.h>

// Hardware includes
#include "myconfig.h"

#include "lib/MCP23017/MCP23017.h"
#include "lib/LED/StaticLED.h"
#include "lib/LED/LEDBoard.h"

#include "lib/BME280/bme280.h"
#include "lib/BME280/bme280_defs.h"
#include "lib/BME280/bme280_if.h"



#include "lib/LMP91000/lmp91000.h"
#include "lib/CCS811/CCS811.h"
#include "lib/Sensirion/SPS30/sps30.h"
#include "lib/Sensirion/SCD30/scd30.h"
#include "lib/ADPD188/ADPD188.h"
#include "lib/ADPD188/adpd_i2c.h"
#include "lib/ADPD188/adpd_gpio.h"

/*********************************************************************
 * MACROS                                                           */
// ======================================================================
// ---------------------- SAEMS-SPECIFIED DEFINES -----------------------
// ======================================================================
#define ZCL_DATA_UPDATE
//#define ZCL_MEASURE_TESTING

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
bool hardwareReady = false; // Changes to true once the i2c and serial interfaces are ready


/*********************************************************************
 * GLOBAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

#ifdef BDB_REPORTING
#if BDBREPORTING_MAX_ANALOG_ATTR_SIZE == 8
  uint8_t reportableChange[] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
#endif
#if BDBREPORTING_MAX_ANALOG_ATTR_SIZE == 4
  uint8_t reportableChange[] = {0x01, 0x00, 0x00, 0x00};
#endif
#if BDBREPORTING_MAX_ANALOG_ATTR_SIZE == 2
  uint8_t reportableChange[] = {0x01, 0x00};
#endif
#endif

// Semaphore used to post events to the application thread
static Semaphore_Handle appSemHandle;
static Semaphore_Struct appSem;

/* App service ID used for messaging with stack service task */
uint8_t  appServiceTaskId;
/* App service task events, set by the stack service task when sending a message */
static uint32_t appServiceTaskEvents;

static endPointDesc_t  zclSampleLightEpDesc = {0};

#if ZG_BUILD_ENDDEVICE_TYPE
static Clock_Handle EndDeviceRejoinClkHandle;
static Clock_Struct EndDeviceRejoinClkStruct;
#endif

static Clock_Handle DiscoveryClkHandle;
static Clock_Struct DiscoveryClkStruct;


#if defined(USE_DMM) && defined(BLE_START)
static Clock_Struct SyncAttrClkStruct;
#endif // defined(USE_DMM) && defined(BLE_START)

// Passed in function pointers to the NV driver
static NVINTF_nvFuncts_t *pfnZdlNV = NULL;

#if defined(USE_DMM) && defined(BLE_START) || !defined(CUI_DISABLE)
static uint16_t zclSampleLight_BdbCommissioningModes;
#endif // defined(USE_DMM) && defined(BLE_START) || !defined(CUI_DISABLE)

afAddrType_t zclSampleLight_DstAddr;

#ifdef ZCL_LEVEL_CTRL
uint8_t zclSampleLight_LevelChangeCmd; // current level change was triggered by an on/off command
uint8_t zclSampleLight_LevelLastLevel;  // to save the Current Level before the light was turned OFF
#endif

#if defined (Z_POWER_TEST)
#ifndef Z_POWER_TEST_DATA_TX_INTERVAL
#define Z_POWER_TEST_DATA_TX_INTERVAL 5000
#endif
#if defined (POWER_TEST_POLL_DATA)
static uint16_t powerTestZEDAddr = 0xFFFE;
#endif
#endif // Z_POWER_TEST

#ifndef CUI_DISABLE
CONST char zclSampleLight_appStr[] = APP_TITLE_STR;
CUI_clientHandle_t gCuiHandle;
static LED_Handle gRedLedHandle;
static uint32_t gSampleLightInfoLine;
#endif

#if defined (BDB_TL_TARGET) || defined (BDB_TL_INITIATOR)
// Touchlink BDB Finding and Binding callback function
static void tl_BDBFindingBindingCb(void);
#endif // defined ( BDB_TL_TARGET ) || defined (BDB_TL_INITIATOR)

// ==================================================================================================================
// ------------------------------------ SAEMS-SPECIFIED VARIABLES AND STRUCTURES ------------------------------------
// ==================================================================================================================
int measure_Testing = 0;
int debounce = 0;
int motion_state = 0;
uint16_t CCS811_co2 = 0;
uint16_t SCD30_co2 = 0;

static uint16_t zclSampleLight_BdbCommissioningModes;
uint8_t OnOff = 0xFE; // initialize to invalid
uint8_t LightLevel = 0xFE;
uint8_t LastLightLevel = 0xFE;

Display_Handle display;

I2C_Handle i2c;
I2C_Params i2cParams;
I2C_Transaction i2cTransaction;
ADC_Params ADCparams;
ADC_Handle adc;
PWM_Handle four_khz_out_handle;
PWM_Params four_khz_out_params;

static Clock_Handle SensorDataClkHandle;
static Clock_Struct SensorDataClkStruct;
static Clock_Handle MotionSensorClkHandle;
static Clock_Struct MotionSensorClkStruct;
static Clock_Handle OnOffClkHandle;
static Clock_Struct OnOffClkStruct;

static Clock_Handle CarbonMonoxide_Alarm_ClkHandle;
static Clock_Struct CarbonMonoxide_Alarm_ClkStruct;
static Clock_Handle Smoke_Alarm_ClkHandle;
static Clock_Struct Smoke_Alarm_ClkStruct;
static Clock_Handle ALARM_CLKHANDLE;
static Clock_Struct ALARM_CLKSTRUCT;

bool CO_ALARM = false;
uint8_t CO_BroadcastMsg[] = {"\0\0\30CO"};
uint8_t CO_transID = 0;

bool SMOKE_ALARM = false;
uint8_t Smoke_BroadcastMsg[] = {"\0\0\30Smoke"};;
uint8_t Smoke_transID = 0;

struct bme280_data bme_data;
struct bme280_dev bme_dev;
struct sps30_measurement m;
struct adpd188_dev *adpd_dev;
struct adpd188_init_param adpd_param;

LMP91000 lmp = LMP91000(i2c, LMP91000_I2C_ADDRESS);
ScioSense_CCS811 ccs = ScioSense_CCS811(i2c, CCS811_SLAVEADDR_1);
LEDBoard ledboard = LEDBoard(CONFIG_SPI_LEDBOARD);
MCP23017 mcp = MCP23017(i2c, 0b0100001);
StaticLED led = StaticLED();
// ==================================================================================================================
// ==================================================================================================================
static uint8_t endPointDiscovered = 0x00;

//Discovery in progress state. This last 3 seconds to get the responses.
static uint8_t discoveryInprogress = 0x00;
#define DISCOVERY_IN_PROGRESS_TIMEOUT   3000


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void zclSampleLight_initialization(void);
static void zclSampleLight_process_loop(void);
static void zclSampleLight_initParameters(void);
static void zclSampleLight_processZStackMsgs(zstackmsg_genericReq_t *pMsg);
static void SetupZStackCallbacks(void);
static void zclSampleLight_processAfIncomingMsgInd(zstack_afIncomingMsgInd_t *pInMsg);
static void zclSampleLight_initializeClocks(void);
#if ZG_BUILD_ENDDEVICE_TYPE
static void zclSampleLight_processEndDeviceRejoinTimeoutCallback(UArg a0);
#endif
static void zclSampleLight_processDiscoveryTimeoutCallback(UArg a0);

static void zclSampleLight_Init( void );

static void zclSampleLight_BasicResetCB( void );
static void zclSampleLight_IdentifyQueryRspCB(zclIdentifyQueryRsp_t *pRsp);
#ifndef CUI_DISABLE
static void zclSampleLight_processKey(uint8_t key, Button_EventMask buttonEvents);
static void zclSampleLight_RemoveAppNvmData(void);
static void zclSampleLight_InitializeStatusLine(CUI_clientHandle_t gCuiHandle);
static void zclSampleLight_UpdateStatusLine(void);
#endif
static uint8_t zclSampleLight_SceneStoreCB(zclSceneReq_t *pReq);
static void  zclSampleLight_SceneRecallCB(zclSceneReq_t *pReq);
static void  SAEMS_OnOffCB( uint8_t cmd );
ZStatus_t zclSampleLight_ReadWriteAttrCB( uint16_t clusterId, uint16_t attrId, uint8_t oper,
                                          uint8_t *pValue, uint16_t *pLen );


static void zclSampleLight_ProcessCommissioningStatus(bdbCommissioningModeMsg_t *bdbCommissioningModeMsg);


#ifdef ZCL_LEVEL_CTRL
static void SAEMS_LevelControlMoveToLevelCB( zclLCMoveToLevel_t *pCmd );
#endif

// Functions to process ZCL Foundation incoming Command/Response messages
static uint8_t zclSampleLight_ProcessIncomingMsg( zclIncoming_t *msg );
#ifdef ZCL_READ
static uint8_t zclSampleLight_ProcessInReadRspCmd( zclIncoming_t *pInMsg );
#endif
#ifdef ZCL_WRITE
static uint8_t zclSampleLight_ProcessInWriteRspCmd( zclIncoming_t *pInMsg );
#endif
static uint8_t zclSampleLight_ProcessInDefaultRspCmd( zclIncoming_t *pInMsg );
#ifdef ZCL_DISCOVER
static uint8_t zclSampleLight_ProcessInDiscCmdsRspCmd( zclIncoming_t *pInMsg );
static uint8_t zclSampleLight_ProcessInDiscAttrsRspCmd( zclIncoming_t *pInMsg );
static uint8_t zclSampleLight_ProcessInDiscAttrsExtRspCmd( zclIncoming_t *pInMsg );
#endif

#if defined (ENABLE_GREENPOWER_COMBO_BASIC)
static void zclSampleLight_GPSink_Toggle(zclGpNotification_t *zclGpNotification);
static void zclSampleLight_GPSink_On(zclGpNotification_t *zclGpNotification);
static void zclSampleLight_GPSink_Off(zclGpNotification_t *zclGpNotification);
static void zclSampleLight_GPSink_Identify(zclGpNotification_t *zclGpNotification);
#endif

#if defined (BDB_TL_TARGET) || defined (BDB_TL_INITIATOR)
tl_BDBFindingBindingCb_t tl_FindingBindingCb =
{
  tl_BDBFindingBindingCb
};
#endif // defined ( BDB_TL_TARGET ) || defined (BDB_TL_INITIATOR)

#if defined(USE_DMM) && defined(BLE_START)
// Clock callback functions
static void zclSampleLight_processSyncAttrTimeoutCallback(UArg a0);

// Remote display callback functions
static void setLightAttrCb(RemoteDisplayLightAttr_t lightAttr, void *const value, uint8_t len);
static void getLightAttrCb(RemoteDisplayLightAttr_t lightAttr, void *value, uint8_t len);

// Provisioning callback functions
static void provisionConnectCb(void);
static void provisionDisconnectCb(void);
static void setProvisioningCb(RemoteDisplay_ProvisionAttr_t provisioningAttr, void *const value, uint8_t len);
static void getProvisioningCb(RemoteDisplay_ProvisionAttr_t provisioningAttr, void *value, uint8_t len);
#endif // defined(USE_DMM) && defined(BLE_START)

#ifdef DMM_OAD
static void zclSampleLight_dmmPausePolicyCb(uint16_t _pause);
/*********************************************************************
 * DMM Policy Callbacks
 */
static DMMPolicy_AppCbs_t dmmPolicyAppCBs =
{
     zclSampleLight_dmmPausePolicyCb
};
#endif

// ===============================================================================================================
// ------------------------------------------ SAEMS-SPECIFIED PROTOTYPES -----------------------------------------
// ===============================================================================================================
#ifdef ZCL_LIGHTING
ZStatus_t SAEMS_ColorControlMoveToHueAndSaturationCB( zclCCMoveToHueAndSaturation_t *pCmd );
#endif // ZCL_LIGHTING

static void SAEMS_SensorsCallback(UArg a0);
static void SAEMS_getSensorData(void);
float scaledHue(void);
float scaledSaturation(void);
float scaledIntensity(void);

static void send_COAlarm_Broadcast(void);
static void send_SmokeAlarm_Broadcast(void);

// ===============================================================================================================
// ===============================================================================================================


/*********************************************************************
 * CONSTANTS
 */

#define LEVEL_CHANGED_BY_LEVEL_CMD  0
#define LEVEL_CHANGED_BY_ON_CMD     1
#define LEVEL_CHANGED_BY_OFF_CMD    2



/*********************************************************************
 * REFERENCED EXTERNALS
 */
extern int16_t zdpExternalStateTaskID;

/*********************************************************************
 * ZCL General Profile Callback table
 */
static zclGeneral_AppCallbacks_t zclSampleLight_CmdCallbacks =
{
  zclSampleLight_BasicResetCB,            // Basic Cluster Reset command
  NULL,                                   // Identfiy cmd
  NULL,                                   // Identify Query command
  zclSampleLight_IdentifyQueryRspCB,      // Identify Query Response command
  NULL,                                   // Identify Trigger Effect command
#ifdef ZCL_ON_OFF
  SAEMS_OnOffCB,                           // On/Off cluster commands
  NULL,                                   // On/Off cluster enhanced command Off with Effect
  NULL,                                   // On/Off cluster enhanced command On with Recall Global Scene
  NULL,                                   // On/Off cluster enhanced command On with Timed Off
#endif
#ifdef ZCL_LEVEL_CTRL
  SAEMS_LevelControlMoveToLevelCB,                      // Level Control Move to Level command
  NULL,                                                 // Level Control Move command
  NULL,                                                 // Level Control Step command
  NULL,                                                 // Level Control Stop command
  NULL,                                                 // Level Control Stop command
#endif
#ifdef ZCL_GROUPS
  NULL,                                   // Group Response commands
#endif
#ifdef ZCL_SCENES
  zclSampleLight_SceneStoreCB,           // Scene Store Request command
  zclSampleLight_SceneRecallCB,          // Scene Recall Request command
  NULL,                                  // Scene Response command
#endif
#ifdef ZCL_ALARMS
  NULL,                                  // Alarm (Response) commands
#endif
#ifdef SE_UK_EXT
  NULL,                                  // Get Event Log command
  NULL,                                  // Publish Event Log command
#endif
  NULL,                                  // RSSI Location command
  NULL                                   // RSSI Location Response command
};

// ======================================
// >>> ZCL Lighting Command Callbacks <<<
// ======================================
static zclLighting_AppCallbacks_t zclSAEMS_CmdCallbacks =
{
  NULL,                                                     // Color Control Move to Hue (0x00)
  NULL,                                                     // Color Control Move Hue (0x01)
  NULL,                                                     // Color Control Step Hue (0x02)
  NULL,                                                     // Color Control Move to Saturation (0x03)
  NULL,                                                     // Color Control Move Saturation (0x04)
  NULL,                                                     // Color Control Step Saturation (0x05)
  SAEMS_ColorControlMoveToHueAndSaturationCB,               // Color Control Move to Hue and Saturation (0x06)
  NULL,                                                     // Color Control Move to Color
  NULL,                                                     // Color Control Move Color
  NULL,                                                     // Color Control Step Color
  NULL,                                                     // Color Control Move to Color Temperature
  NULL,                                                     // Color Control Enhanced Move to Hue
  NULL,                                                     // Color Control Enhanced Move Hue
  NULL,                                                     // Color Control Enhanced Step Hue
  NULL,                                                     // Color Control Enhanced Move to Hue and Saturation
  NULL,                                                     // Color Control Color Loop Set
  NULL,                                                     // Color Control Stop Move Step
  NULL,                                                     // Color Control Move Color Temperature
  NULL                                                      // Color Control Step Color Temperature
};

#if defined (ENABLE_GREENPOWER_COMBO_BASIC)
GpSink_AppCallbacks_t zclSampleLight_GpSink_AppCallbacks =
{
#ifdef ZCL_IDENTIFY
    zclSampleLight_GPSink_Identify,     //IdentifyCmd;
#endif
#ifdef ZCL_SCENES
    NULL,                               //RecallSceneCmd;
    NULL,                               //StoreSceneCmd;
#endif
#ifdef ZCL_ON_OFF
    zclSampleLight_GPSink_Off,          //OffCmd;
    zclSampleLight_GPSink_On,           //OnCmd;
    zclSampleLight_GPSink_Toggle,       //ToggleCmd;
#endif
#ifdef ZCL_LEVEL_CTRL
    NULL,                               //LevelControlStopCmd;
    NULL,                               //MoveUpCmd;
    NULL,                               //MoveDownCmd;
    NULL,                               //StepUpCmd;
    NULL,                               //StepDownCmd;
    NULL,                               //MoveUpWithOnOffCmd;
    NULL,                               //MoveDownWithOnOffCmd;
    NULL,                               //StepUpWithOnOffCmd;
    NULL,                               //StepDownWithOnOffCmd;
#endif
    NULL,                               //MoveHueStopCmd;
    NULL,                               //MoveHueUpCmd;
    NULL,                               //MoveHueDownCmd;
    NULL,                               //StepHueUpCmd;
    NULL,                               //StepHueDownCmd;
    NULL,                               //MoveSaturationStopCmd;
    NULL,                               //MoveSaturationUpCmd;
    NULL,                               //MoveSaturationDownCmd;
    NULL,                               //StepSaturationUpCmd;
    NULL,                               //StepSaturationDownCmd;
    NULL,                               //MoveColorCmd;
    NULL,                               //StepColorCmd;
#ifdef ZCL_DOORLOCK
    NULL,                               //LockDoorCmd;
    NULL,                               //UnlockDoorCmd;
#endif
    NULL,                               //AttributeReportingCmd;
    NULL,                               //MfrSpecificReportingCmd;
    NULL,                               //MultiClusterReportingCmd;
    NULL,                               //MfrSpecificMultiReportingCmd;
    NULL,                               //RequestAttributesCmd;
    NULL,                               //ReadAttributeRspCmd;
    NULL,                               //zclTunnelingCmd;
};
#endif

#if defined(USE_DMM) && defined(BLE_START)
RemoteDisplay_clientProvisioningtCbs_t zclSwitch_ProvissioningCbs =
{
    setProvisioningCb,
    getProvisioningCb,
    provisionConnectCb,
    provisionDisconnectCb
};

RemoteDisplay_LightCbs_t zclSwitch_LightCbs =
{
    setLightAttrCb,
    getLightAttrCb
};

zstack_DevState provState = zstack_DevState_HOLD;
uint16_t provPanId = ZDAPP_CONFIG_PAN_ID;
uint32_t provChanMask = DEFAULT_CHANLIST;

DMMPolicy_StackRole DMMPolicy_StackRole_Zigbee =
#if ZG_BUILD_ENDDEVICE_TYPE
    DMMPolicy_StackRole_ZigbeeEndDevice;
#elif ZG_BUILD_RTRONLY_TYPE
    DMMPolicy_StackRole_ZigbeeRouter;
#elif ZG_BUILD_COORDINATOR_TYPE
    DMMPolicy_StackRole_ZigbeeCoordinator;
#endif

#endif // defined(USE_DMM) && defined(BLE_START)

#ifdef ZCL_LIGHTING
// =================================================================================================================
// ------------------------------------------- SAEMS-SPECIFIED FUNCTIONS -------------------------------------------
// =================================================================================================================
/******************************************************************************************
 * @fn        SAEMS_ColorControlMoveToHueAndSaturationCB
 *
 * @brief     Callback function to adjust the Hue and Saturation of the LED Board to a specified value
 *
 * @param     pCmd - ZigBee command parameters
 *
 * @return    none
 */
ZStatus_t SAEMS_ColorControlMoveToHueAndSaturationCB( zclCCMoveToHueAndSaturation_t *pCmd ){
  // Get the hue and saturation values from the "Move to Hue and Saturation" command and save it to the
  // hue and saturation variables
  SAEMS_ColorControl_CurrentHue = pCmd->hue;
  SAEMS_ColorControl_CurrentSaturation = pCmd->saturation;

  // Pass the 'new hue and saturation' to the LED Board Driver Function
  ledboard.hsi(scaledHue(), scaledSaturation(), scaledIntensity(), false);

  zstack_bdbRepChangedAttrValueReq_t ReqHue;
        ReqHue.attrID = ATTRID_COLOR_CONTROL_CURRENT_HUE;
        ReqHue.cluster = ZCL_CLUSTER_ID_LIGHTING_COLOR_CONTROL;
        ReqHue.endpoint = SAMPLELIGHT_ENDPOINT;
  Zstackapi_bdbRepChangedAttrValueReq(appServiceTaskId,&ReqHue);

  zstack_bdbRepChangedAttrValueReq_t ReqSaturation;
        ReqSaturation.attrID = ATTRID_COLOR_CONTROL_CURRENT_HUE;
        ReqSaturation.cluster = ZCL_CLUSTER_ID_LIGHTING_COLOR_CONTROL;
        ReqSaturation.endpoint = SAMPLELIGHT_ENDPOINT;
  Zstackapi_bdbRepChangedAttrValueReq(appServiceTaskId,&ReqSaturation);
  
  return ZSuccess;
}
#endif // ZCL_LIGHTING

// >>>>>> Correctly Scaled Values for passing to the HSI function <<<<<<
float scaledHue(void){
    // Constant multiplier obtained from scaling range 0-255 to 0-360
    return( 1.4117 * ((float)SAEMS_ColorControl_CurrentHue) );
}
float scaledSaturation(void){
    // Constant multiplier obtained from scaling range 0-255 to 0-1
    return( 0.00392 * ((float)SAEMS_ColorControl_CurrentSaturation) );
}
float scaledIntensity(void){
    // Constant multiplier obtained from scaling range 0-255 to 0-1
    return( 0.00392 * ((float)zclSampleLight_getCurrentLevelAttribute()) );
}

/*********************************************************************
 * @fn      SAEMS_OnOffCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received an On/Off Command for this application.
 *
 * @param   cmd - COMMAND_ON_OFF_ON, COMMAND_ON_OFF_OFF or COMMAND_ON_OFF_TOGGLE
 *
 * @return  none
 */
static void SAEMS_OnOffCB( uint8_t cmd )
{
  afIncomingMSGPacket_t *pPtr = zcl_getRawAFMsg();

  OnOff = 0xFE; // initialize to invalid
  LightLevel = 0xFE;

  zclSampleLight_DstAddr.addr.shortAddr = pPtr->srcAddr.addr.shortAddr;

  // Turn on the light
  if ( cmd == COMMAND_ON_OFF_ON ){
    OnOff = LIGHT_ON;
  }
  // Turn off the light
  else if ( cmd == COMMAND_ON_OFF_OFF ){
    OnOff = LIGHT_OFF;
    LastLightLevel = zclSampleLight_getCurrentLevelAttribute();
  }


  if( ((zclSampleLight_getOnOffAttribute() == LIGHT_ON) && (OnOff == LIGHT_ON)) ||
      ((zclSampleLight_getOnOffAttribute() == LIGHT_OFF) && (OnOff == LIGHT_OFF)) )
  {
    // if light is on and received an on command, ignore it.
    // if light is off and received an off command, ignore it.
    return;
  }

  if(zclSampleLight_getCurrentLevelAttribute() > 0){
    UtilTimer_start( &OnOffClkStruct );
  }
  else if(zclSampleLight_getCurrentLevelAttribute() == 0){
    UtilTimer_start( &OnOffClkStruct );
  }
}

#ifdef ZCL_LEVEL_CTRL
/*********************************************************************
 * @fn      SAEMS_LevelControlMoveToLevelCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received a LevelControlMoveToLevel Command for this application.
 *
 * @param   pCmd - ZigBee command parameters
 *
 * @return  none
 */
static void SAEMS_LevelControlMoveToLevelCB( zclLCMoveToLevel_t *pCmd )
{
  uint8_t newLevel = pCmd->level;

  if(newLevel == 0)
    zclSampleLight_updateOnOffAttribute(LIGHT_OFF);
  else if(newLevel > 0)
    zclSampleLight_updateOnOffAttribute(LIGHT_ON);

  zclSampleLight_updateCurrentLevelAttribute(newLevel);

  ledboard.hsi( scaledHue(), scaledSaturation(), scaledIntensity(), false );
}
#endif

/*************************************************************************
 * @fn      SAEMS_SensorsCallback
 *
 * @brief   Timeout handler function for getting data from the sensors and passing to the Hub
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
static void SAEMS_SensorsCallback(UArg a0){

  (void)a0;     // Parameter is not used

  appServiceTaskEvents |= SAMPLELIGHT_POLL_CONTROL_TIMEOUT_EVT;

  // Wake up the application thread when it waits for the clock event
  Semaphore_post(appSemHandle);
}

int led_state = 0;
uint32_t reading_num = 1;
/*************************************************************************
 * @fn      SAEMS_getSensorData
 *
 * @brief   Gets data from the sensors through I2C/Digital
 *
 * @param   none
 *
 * @return  none
 */
static void SAEMS_getSensorData(void){
    // Using driver functions, get data from I2C lines and store in the new struct
    // TO-DO:
    //--------------------------------------------------------------------------------------
    // Temperature, Humidity, Pressure
    bme280_if_get_all_sensor_data(&bme_data, &bme_dev);

    sensorDataNew.temperature = 0.1 *bme_data.temperature;
    sensorDataNew.pressure =    0.1 *bme_data.pressure;
    sensorDataNew.humidity =    0.01 *bme_data.humidity;
    //--------------------------------------------------------------------------------------
    // VOC & CO2
    ccs.sample();

    sensorDataNew.carbondioxide = ccs.getECO2();
    sensorDataNew.voc = ccs.getTVOC();
    CCS811_co2 = sensorDataNew.carbondioxide;
    //--------------------------------------------------------------------------------------
    // Particulates
    sensirion_sleep_usec(SPS30_MEASUREMENT_DURATION_USEC); /* wait 1s */
    int ret = sps30_read_measurement(&m);
    if (ret < 0)
    {
      printf("error reading measurement\n");
    }
    else
    {
      sensorDataNew.typicalparticlesize = 100 * m.typical_particle_size;
      sensorDataNew.pm1mass             = 100 * m.mc_1p0;
      sensorDataNew.pm2mass             = 100 * m.mc_2p5;
      sensorDataNew.pm4mass             = 100 * m.mc_4p0;
      sensorDataNew.pm10mass            = 100 * m.mc_10p0;
      sensorDataNew.pm1number           = 100 * m.nc_1p0;
      sensorDataNew.pm2number           = 100 * m.nc_2p5;          
      sensorDataNew.pm4number           = 100 * m.nc_4p0;
      sensorDataNew.pm10number          = 100 * m.nc_10p0;   
    }
    //--------------------------------------------------------------------------------------
    // Carbon Monoxide
    adc = ADC_open(CO_OUT, &ADCparams);
    double nA = lmp.getCurrent(adc);
    double co_sensitivity = 7.00;
    ADC_close( adc );

    sensorDataNew.carbonmonoxide = (uint16_t) ( ceil(nA / co_sensitivity) );
    // IF THE CARBON MONOXIDE MEASUREMENT IS ABOVE 50 -> SET OFF THE ALARM
    if(sensorDataNew.carbonmonoxide > 50)
      CO_ALARM = true;
    //--------------------------------------------------------------------------------------
    // Smoke
      uint16_t samples;
      uint16_t rxreg;
      adpd188_reg_read(adpd_dev, ADPD188_REG_STATUS, &rxreg);    
      uint16_t fifonumrx = rxreg >> 8;
      
      if( fifonumrx >= 4 ){
        adpd188_reg_read(adpd_dev, 0x64, &samples);
        adpd188_reg_read(adpd_dev, 0x68, &samples);
        adpd188_reg_read(adpd_dev, 0x60, &samples);
        sensorDataNew.smoke = samples;
        adpd188_reg_read(adpd_dev, 0x60, &samples);
        fifonumrx = fifonumrx - 4;
      }

      // NEED TO ADD IF STATEMENT FOR SMOKE LEVEL

    //--------------------------------------------------------------------------------------
    // Power System Usage

    // Voltage is obtained from the 5V_MainDet (DIO24)
    adc = ADC_open(POWER_POLL, &ADCparams);
    uint16_t voltage;
    ADC_convert(adc, &voltage);
    uint32_t voltage_uV = ADC_convertToMicroVolts(adc, voltage);
    ADC_close(adc);

    // If the voltage is less than 1V
    if( voltage_uV < 1000000){
      // Toggle LED between Green and Off - the device is powered from the battery
      if(led_state == 0){
        led.set( RGB_States::GREEN );     led_state = 1;
      }
      else if(led_state == 1){
        led.set(false, false, false);     led_state = 0;
      }
    }
    else{
      // Otherwise, display steady Green and Red on the LED - the device is powered through PoE
      led.set(RGB_States::RED | RGB_States::GREEN);
    } 
      
    //--------------------------------------------------------------------------------------
    printf("--------------------------------------------------------- \n");
    printf("|#%d         S.A.E.M.S Device Measurements               |\n", reading_num++);
    printf("--------------------------------------------------------- \n");
    printf("|     Sensor     |   Measurement   |        Value       |\n");
    printf("--------------------------------------------------------- \n");
    printf("|     BME280     |   Temperature   |      %.2f C\t|\n", 0.1f*sensorDataNew.temperature);
    printf("|     BME280     |     Pressure    |     %.1f hPa\t|\n", 0.1f*sensorDataNew.pressure);
    printf("|     BME280     |     Humidity    |      %.2f %%\t|\n", 0.1f*sensorDataNew.humidity);
    printf("|   EKMC1691111  |      Motion     |        %d\t\t|\n", sensorDataNew.occupancy);
    printf("|     CCS811     |       CO2       |       %d ppm\t|\n", sensorDataNew.carbondioxide);
    printf("|     CCS811     |       VOC       |        %d ppb\t|\n", sensorDataNew.voc);
    printf("--------------------------------------------------------- \n");
    printf("|                |    1.0 Mass     |       %.2f\t\t|\r\n"
           "|                |    1.0 Count    |       %.2f\t|\r\n"
           "|                |    2.5 Mass     |       %.2f\t\t|\r\n"
           "|     SPS30      |    2.5 Count    |       %.2f\t|\r\n"
           "| (Particulates) |    4.0 Mass     |       %.2f\t\t|\r\n"
           "|                |    4.0 Count    |       %.2f\t|\r\n"
           "|                |    10.0 Mass    |       %.2f\t\t|\r\n"
           "|                |   10.0 Count    |       %.2f\t|\n",
           sensorDataNew.pm1mass/100.00, sensorDataNew.pm1number/100.00, 
           sensorDataNew.pm2mass/100.00, sensorDataNew.pm2number/100.00,
           sensorDataNew.pm4mass/100.00, sensorDataNew.pm4number/100.00, 
           sensorDataNew.pm10mass/100.00, sensorDataNew.pm10number/100.00);
    printf("--------------------------------------------------------- \n");
    printf("|    LMP91000    |       CO       |     %.2f ppm\t|\n", sensorDataNew.carbonmonoxide/1.00);
    printf("|    ADPD188BI   |      Smoke     |        %d\t\t|\n", sensorDataNew.smoke);
    printf("--------------------------------------------------------- \n");
   
    // CO Alarm Handling
    if(CO_ALARM){
      // Start the CO Alarm and the Alarm Timer
      UtilTimer_start( &ALARM_CLKSTRUCT );
      UtilTimer_start( &CarbonMonoxide_Alarm_ClkStruct );
      // Send Broadcast message to other SAEMS routers in the network
      send_COAlarm_Broadcast();
      // needs to be handled in zclSampleLight_processAfIncomingMsgInd
    }
    // Smoke Alarm Handling
    if(SMOKE_ALARM){
      // Start the Smoke Alarm and the Alarm Timer
      UtilTimer_start( &ALARM_CLKSTRUCT );
      UtilTimer_start( &Smoke_Alarm_ClkStruct );
      // Send Broadcast message to other SAEMS routers in the network
      send_SmokeAlarm_Broadcast();
      // needs to be handled in zclSampleLight_processAfIncomingMsgInd
    }
      
    
    //--------------------------------------------------------------------------------------
    // The following is sample data...
    #ifdef ZCL_MEASURE_TESTING
    if(measure_Testing == 0){
      sensorDataNew.temperature = 1000;
      sensorDataNew.humidity = 1000;
      sensorDataNew.pressure = 1000;
      sensorDataNew.carbonmonoxide = 1000;
      sensorDataNew.carbondioxide = 1000;
      sensorDataNew.smoke = 1000;
      sensorDataNew.voc = 1000;
      sensorDataNew.occupancy = 0;

      measure_Testing = 1;
    } else{
        sensorDataNew.temperature = 1250;
        sensorDataNew.humidity = 1250;
        sensorDataNew.pressure = 1250;
        sensorDataNew.carbonmonoxide = 1250;
        sensorDataNew.carbondioxide = 1250;
        sensorDataNew.smoke = 1250;
        sensorDataNew.voc = 1250;
        sensorDataNew.occupancy = 1;

        measure_Testing = 0;
    }
    #endif
}

/************************************************************
 * @fn        send_COAlarm_Broadcast
 * 
 * @brief     Helper function for sending Broadcast for CO Alarm
 * 
 * @param     none  
 * 
 * @return    none
 */
static void send_COAlarm_Broadcast(){
  // Get the ZCL Frame Counter
  zstack_getZCLFrameCounterRsp_t pRsp;
  Zstackapi_getZCLFrameCounterReq(appServiceTaskId, &pRsp);

  // Increment the transaction ID of the message by 1
  CO_BroadcastMsg[1] = CO_transID++;

  // Construct the Raw Data message to send out over broadcast
  zstack_afDataReq_t COreq;
  COreq.dstAddr.addrMode = zstack_AFAddrMode_BROADCAST;
  COreq.dstAddr.addr.shortAddr = NWK_BROADCAST_SHORTADDR_DEVALL;
  COreq.dstAddr.endpoint = SAMPLELIGHT_ENDPOINT;
  COreq.pRelayList = NULL;
  COreq.n_relayList = 0;
  COreq.srcEndpoint = SAMPLELIGHT_ENDPOINT;
  COreq.clusterID = SAEMS_CO_ALARM_CLUSTER_ID;
  COreq.transID = &pRsp.zclFrameCounter;
  COreq.options.ackRequest = FALSE;
  COreq.options.apsSecurity = FALSE;
  COreq.options.limitConcentrator = FALSE;
  COreq.options.skipRouting = FALSE;
  COreq.options.suppressRouteDisc = FALSE;
  COreq.options.wildcardProfileID = FALSE;
  COreq.radius = AF_DEFAULT_RADIUS;
  COreq.n_payload = sizeof(CO_BroadcastMsg);
  COreq.pPayload = CO_BroadcastMsg;

  // Check the status for any errors
  zstack_ZStatusValues status = Zstackapi_AfDataReq(appServiceTaskId, &COreq);
  // Reset the associated boolean variable to FALSE
  CO_ALARM = false;
}

/************************************************************
 * @fn        send_SmokeAlarm_Broadcast
 * 
 * @brief     Helper function for sending Broadcast for Smoke Alarm
 * 
 * @param     none  
 * 
 * @return    none
 */
static void send_SmokeAlarm_Broadcast(){
  // Get the ZCL Frame Counter
  zstack_getZCLFrameCounterRsp_t pRsp;
  Zstackapi_getZCLFrameCounterReq(appServiceTaskId, &pRsp);

  // Increment the transaction ID of the message by 1
  Smoke_BroadcastMsg[1] = Smoke_transID++;

  // Construct the Raw Data message to send out over broadcast
  zstack_afDataReq_t SMOKEreq;
  SMOKEreq.dstAddr.addrMode = zstack_AFAddrMode_BROADCAST;
  SMOKEreq.dstAddr.addr.shortAddr = NWK_BROADCAST_SHORTADDR_DEVALL;
  SMOKEreq.dstAddr.endpoint = SAMPLELIGHT_ENDPOINT;
  SMOKEreq.pRelayList = NULL;
  SMOKEreq.n_relayList = 0;
  SMOKEreq.srcEndpoint = SAMPLELIGHT_ENDPOINT;
  SMOKEreq.clusterID = SAEMS_SMOKE_ALARM_CLUSTER_ID;
  SMOKEreq.transID = &pRsp.zclFrameCounter;
  SMOKEreq.options.ackRequest = FALSE;
  SMOKEreq.options.apsSecurity = FALSE;
  SMOKEreq.options.limitConcentrator = FALSE;
  SMOKEreq.options.skipRouting = FALSE;
  SMOKEreq.options.suppressRouteDisc = FALSE;
  SMOKEreq.options.wildcardProfileID = FALSE;
  SMOKEreq.radius = AF_DEFAULT_RADIUS;
  SMOKEreq.n_payload = sizeof(Smoke_BroadcastMsg);
  SMOKEreq.pPayload = Smoke_BroadcastMsg;

  // Check the status for any errors
  zstack_ZStatusValues status = Zstackapi_AfDataReq(appServiceTaskId, &SMOKEreq);
  // Reset the associated boolean variable to FALSE
  SMOKE_ALARM = false;
}
/************************************************************
 * @fn        SAEMS_detectedMotionInterrupt
 * 
 * @brief     Interrupt function for detect motion from PIR sensor
 * 
 * @param     index - GPIO pin which calls the interrupt
 * 
 * @return    none
*/
void SAEMS_detectedMotionInterrupt(uint_least8_t index){
  if( debounce == 0){
    // Stop any timer that is currently running
    UtilTimer_stop( &MotionSensorClkStruct );
    printf("MOTION DETECTED!\n");
    // Set occupancy to "Occupied: 1"
    sensorDataNew.occupancy = 1;
    printf("Setting Occupancy: %d\t", sensorDataNew.occupancy);
    // Set debounce to 1 and motion_state to 0
    debounce = 1;     motion_state = 0;
    // Start the timer for debouncing for 3 seconds
    printf("Debounce Timer: 3 secs\n");
    UtilTimer_setTimeout( MotionSensorClkHandle, 3000);
    UtilTimer_start( &MotionSensorClkStruct );
  }
}

/*************************************************************
 * @fn        SAEMS_MotionSensorCB
 * 
 * @brief     Timeout handler for debouncing for 3secs and renewing for 5mins
 * 
 * @param     a0 - this parameter is unused
 * 
 * @return    none
 */ 
static void SAEMS_MotionSensorCB(UArg a0){
  (void)a0;
  // Motion State 0: Debouncing input for only 1 sample and wait for the defined interval before polling again
  if( motion_state == 0){
    printf("Input has been debounced...\t");
    debounce = 0;     motion_state = 1;
    printf("Waiting \"Defined SmartThings Interval\" secs to poll again\n");
    // Disable the motion sensor interrupt and wait for the set amount of time before enabling 
    GPIO_disableInt(PIR_SENSOR);
    UtilTimer_setTimeout( MotionSensorClkHandle, (sensorDataCurrent.pollingRate*1000) );
    UtilTimer_start( &MotionSensorClkStruct );
  }
  // Motion State 1: Polling Rest Period finishes
  else if( motion_state == 1){
    GPIO_enableInt(PIR_SENSOR);
    // Poll for motion for 15 seconds 
    motion_state = 2;
    UtilTimer_setTimeout( MotionSensorClkHandle, 15000 );
    UtilTimer_start( &MotionSensorClkStruct );
  }
  // Motion State 2: No motion detected during the polling period
  else if( motion_state == 2){
    printf("NO MOTION DETECTED!\n");
    sensorDataNew.occupancy = 0;
    printf("Setting Occupancy: %d\t", sensorDataNew.occupancy);
    motion_state = 0;
  }
}

/*****************************************************************
 * @fn      SAEMS_OnOff_Timeout_Callback
 * 
 * @brief   Timeout handler function for ramping up/down ON-OFF COMMANDS
 * 
 * @param   a0 - this parameter is not used
 * 
 * @return  none
 */
static void SAEMS_OnOff_Timeout_Callback( UArg a0 ){
  
  (void)a0; // Parameter is not used

  appServiceTaskEvents |= SAMPLELIGHT_LEVEL_CTRL_EVT;

  // Wake up the application thread when it waits for clock event
  Semaphore_post(appSemHandle);
}

/*****************************************************************
 * @fn      SAEMS_OnOff_Ramp
 * 
 * @brief   Function that ramps the light level On or Off based on the command 
 * 
 * @param   none
 * 
 * @return  none*
 */
static void SAEMS_OnOff_Ramp( void ){
  printf("SAMES_OnOff_Ramp()\n");

  printf("Last Light Level: %d\n", LastLightLevel);

  // Received ON_OFF_ON COMMAND
  if( OnOff == LIGHT_ON ){
  printf("In the LIGHT_ON branch...\n");
  // get the current light level, increment, update the light level, and send to LED Board
  LightLevel = zclSampleLight_getCurrentLevelAttribute();
  printf("> Current Level: %d\n", LightLevel);

  // if statement to get rid of the uint8_t overflow when level is at 100%
  if( LightLevel + 10 >= LastLightLevel){
    LightLevel = LastLightLevel;
  }else{
    LightLevel += 10;
  }

  printf("> New Level: %d\n", LightLevel);
  zclSampleLight_updateCurrentLevelAttribute( LightLevel );
  ledboard.hsi( scaledHue(), scaledSaturation(), scaledIntensity(), false );

    // Re-start the timer until the condition has been met to stop ramping
    if( zclSampleLight_getCurrentLevelAttribute() >= LastLightLevel ){
      // CONDITION MET: Do not restart the timer
      printf("CONDITION MET!!\n");
      zclSampleLight_updateCurrentLevelAttribute(LastLightLevel);
      zclSampleLight_updateOnOffAttribute(OnOff);
    }else if( zclSampleLight_getCurrentLevelAttribute() < LastLightLevel ){
      // CONDITION NOT MET: Restart the timer
      printf("Condition NOT MET: Restarting the timer!!\n");
      UtilTimer_start( &OnOffClkStruct );
    }

  }

  // Received ON_OFF_OFF COMMAND
  else if( OnOff == LIGHT_OFF ){
  printf("In the LIGHT_OFF branch...\n");
  // get the current light level, decrement, update the light level, and send to LED Board
  LightLevel = zclSampleLight_getCurrentLevelAttribute();
  printf("> Current Level: %d\n", LightLevel);

  // if statement to get rid of the uint8_t overflow when level is at 0%
  if( LightLevel < 10 ){
    LightLevel = LastLightLevel;
  }else{
    LightLevel -= 10;
  }

  printf("> New Level: %d\n", LightLevel);
  zclSampleLight_updateCurrentLevelAttribute( LightLevel );
  ledboard.hsi( scaledHue(), scaledSaturation(), scaledIntensity(), true );

    // Re-start the timer until the condition has been met to stop ramping
    if( zclSampleLight_getCurrentLevelAttribute() <= 10  ){
      // CONDITION MET: Do not restart the timer
      printf("CONDITION MET!!\n");
      zclSampleLight_updateCurrentLevelAttribute(0);
      zclSampleLight_updateOnOffAttribute(OnOff);
    }else if( zclSampleLight_getCurrentLevelAttribute() > 10 ){
      // CONDITION NOT MET: Restart the timer
      printf("Condition NOT MET: Restarting the timer!!\n");
      UtilTimer_start( &OnOffClkStruct );
    }

  }
}

int CO_alarm_state = 0;
/*****************************************************************
 * @fn          SAEMS_CarbonMonoxide_Alarm_handler
 * 
 * @brief       Handler function for controlling alarm on Carbon Monoxide 
 * 
 * @param       a0 - not in use
 * 
 * @return      none
 */
static void SAEMS_CarbonMonoxide_Alarm_handler(UArg a0){
  (void)a0;

  // make the alarm beep every .5 sec
  if(CO_alarm_state == 0){
    // CARBON MONOXIDE: ALARM ON
    PWM_start(four_khz_out_handle);
    CO_alarm_state = 1;
  }
  else if(CO_alarm_state == 1){
    // CARBON MONOXIDE: ALARM OFF
    PWM_stop(four_khz_out_handle);
    CO_alarm_state = 0;
  }
}

int Smoke_alarm_state = 0;
/*****************************************************************
 * @fn        SAEMS_Smoke_Alarm_handler
 * 
 * @brief     Handler function for controlling alarm for Smoke Detection
 * 
 * @param     a0 - not in use
 * 
 * @return    none
 */
static void SAEMS_Smoke_Alarm_handler(UArg a0){
  (void)a0;

  // make the alarm beep every 1.5 sec
  if(CO_alarm_state == 0){
    // SMOKE: ALARM ON
    PWM_start(four_khz_out_handle);
    CO_alarm_state = 1;
  }
  else if(CO_alarm_state == 1){
    // SMOKE: ALARM OFF
    PWM_stop(four_khz_out_handle);
    CO_alarm_state = 0;
  }
}

/*****************************************************************
 * @fn        SAEMS_ALARM_handler
 * 
 * @brief     Handler function for controlling the duration of an alarm
 * 
 * @param     a0 - not in use
 * 
 * @return    none
 */
static void SAEMS_ALARM_handler(UArg a0){
  (void)a0;
  // If CO or smoke is greater than their respective values 
  //  - Continue the alarm
  //  - Otherwise, stop the alarm
  if(sensorDataNew.carbonmonoxide > 50){
    UtilTimer_start( &ALARM_CLKSTRUCT );
  }else{
    UtilTimer_stop( &ALARM_CLKSTRUCT );
    UtilTimer_stop( &CarbonMonoxide_Alarm_ClkStruct );
  }

  if(sensorDataNew.smoke > 9999){
    UtilTimer_start( &ALARM_CLKSTRUCT );
  }else{
    UtilTimer_start( &ALARM_CLKSTRUCT );
    UtilTimer_stop( &Smoke_Alarm_ClkStruct );
  }
}
/*****************************************************************
 * @fn          SAEMS_Sensors_Initialization
 * 
 * @brief       Helper function to initialize the sensor subsystem on start-up
 * 
 * @param       none
 * 
 * @return      none
 */

void SAEMS_Sensors_Initialization(){
  //------------------------------------------------------------
  ccs = ScioSense_CCS811(i2c, CCS811_SLAVEADDR_1);
  ccs.begin();
  ccs.start(1);
  //------------------------------------------------------------
  adpd188_start(&i2c);
  uint16_t rxtemp[1];
  adpd188_init(&adpd_dev, &adpd_param);
  adpd188_reg_write(adpd_dev, 0x4b, 0x2612 | (1 << 7));
  adpd188_mode_set(adpd_dev, ADPD188_PROGRAM);
  adpd188_smoke_detect_setup(adpd_dev);
  adpd188_reg_read(adpd_dev, 0x4b, rxtemp);
  adpd188_reg_write(adpd_dev, ADPD188_REG_STATUS, 0x80FF);
  adpd188_mode_set(adpd_dev, ADPD188_NORMAL);
  //------------------------------------------------------------
  lmp.unlock();
  lmp.setBiasSign(1);
  lmp.setRLoad(3);
  lmp.setIntZ(0);
  lmp.setGain(1);
  lmp.setIntRefSource();
  lmp.setBias(0);
  lmp.setThreeLead();
  ADC_close(adc);
  //------------------------------------------------------------
  bme_dev = { 0 };
  bme_data = { 0 };
  bme280_if_init(&bme_dev, &i2c);
  //------------------------------------------------------------
  sensirion_i2c_init(&i2c);
  int16_t ret;
  if (sps30_probe() != 0){
    printf("SPS sensor probing failed\n");
    sensirion_sleep_usec(1000000); /* wait 1s */
  }
  printf("SPS sensor probing successful\n");

  ret = sps30_start_measurement();
  if (ret < 0)
    printf("error starting measurement\n");
  printf("measurements started\n");
  //------------------------------------------------------------
  // Set the default polling rate (60s = 1 min)
  sensorDataCurrent.pollingRate = 60;
  //------------------------------------------------------------
}
// ================================================================================================================
// ================================================================================================================

/*******************************************************************************
 * @fn          sampleApp_task
 *
 * @brief       Application task entry point for the Z-Stack
 *              Sample Application
 *
 * @param       pfnNV - pointer to the NV functions
 *
 * @return      none
 */
    int x = 0;
#include <xdc/runtime/System.h>

     MCP23017 *mcpptr;

void sampleApp_task(NVINTF_nvFuncts_t *pfnNV)
{
  // Save and register the function pointers to the NV drivers
  pfnZdlNV = pfnNV;
  zclport_registerNV(pfnZdlNV, ZCL_PORT_SCENE_TABLE_NV_ID);

  // Initialize application
  zclSampleLight_initialization();

  GPIO_init();
  Display_init();
  SPI_init();
  I2C_init();
  ADC_init();
  GPIO_init();
  PWM_init();

  PWM_Params_init(&four_khz_out_params);
  four_khz_out_params.periodUnits  = PWM_PERIOD_HZ;               // Period is in Hz
  four_khz_out_params.periodValue  = 4000;                        // 4kHz
  four_khz_out_params.dutyUnits    = PWM_DUTY_FRACTION;           // Duty is fraction of period
  four_khz_out_params.dutyValue    = PWM_DUTY_FRACTION_MAX / 2;   // 50% duty cycle
  four_khz_out_handle = PWM_open(CONFIG_PWM_4KHZ, &four_khz_out_params);
  if (four_khz_out_handle == NULL)
    printf("CONFIG_PWM_4KHZ did not open\\n");

  I2C_Params_init(&i2cParams);
  i2cParams.bitRate = I2C_100kHz;
  i2c = I2C_open(CONFIG_I2C_0, &i2cParams);

  // initialize optional ADC parameters
  ADC_Params_init(&ADCparams);
  ADCparams.isProtected = true;
  adc = ADC_open(CO_OUT, &ADCparams);

  SAEMS_Sensors_Initialization();

  // Set up the io expander
  mcp = MCP23017(i2c, 0b0100001);
  mcp.init();
  mcpptr = &mcp;

#if SAEMS_HARDWARE_VERSION == 0
  led = StaticLED(mcp, MCP_PinMap::I_LED_B, MCP_PinMap::I_LED_G, MCP_PinMap::I_LED_R);
#elif SAEMS_HARDWARE_VERSION == 1
  StaticLED led = StaticLED(mcp, MCP_PinMap::I_LED_R, MCP_PinMap::I_LED_G, MCP_PinMap::I_LED_B);
#endif

  led.set(RGB_States::NONE);

  ledboard.init();
  ledboard.hsi( scaledHue(), scaledSaturation(), scaledIntensity(), false );
  hardwareReady = true;

  GPIO_setCallback( PIR_SENSOR, SAEMS_detectedMotionInterrupt );
  GPIO_enableInt( PIR_SENSOR );

  if( MotionSensorClkHandle != NULL){
    printf("Successfully created the motion timer!\n");
  }

  // ** In case the device is deleted from Smart Things or becomes unpaired in the network **
  //Zstackapi_bdbResetLocalActionReq( appServiceTaskId );
  // No return from task process
  zclSampleLight_process_loop();
}



/*******************************************************************************
 * @fn          zclSampleLight_initialization
 *
 * @brief       Initialize the application
 *
 * @param       none
 *
 * @return      none
 */
static void zclSampleLight_initialization(void)
{
    /* Initialize user clocks */
    zclSampleLight_initializeClocks();

    /* create semaphores for messages / events
     */
    Semaphore_Params semParam;
    Semaphore_Params_init(&semParam);
    semParam.mode = ti_sysbios_knl_Semaphore_Mode_COUNTING;
    Semaphore_construct(&appSem, 0, &semParam);
    appSemHandle = Semaphore_handle(&appSem);

    appServiceTaskId = OsalPort_registerTask(Task_self(), appSemHandle, &appServiceTaskEvents);

    //Initialize stack
    zclSampleLight_Init();

#if defined (Z_POWER_TEST)
    zstack_sysSetTxPowerReq_t txPowerReq;
    zstack_sysSetTxPowerRsp_t txPowerRsp;
    txPowerReq.requestedTxPower = POWER_TEST_TX_PWR;
    Zstackapi_sysSetTxPowerReq(appServiceTaskId, &txPowerReq, &txPowerRsp);
    OsalPort_setEvent(appServiceTaskId, SAMPLEAPP_POWER_TEST_START_EVT);
#endif

#ifdef DMM_OAD
    // register the app callbacks
    DMMPolicy_registerAppCbs(dmmPolicyAppCBs, DMMPolicy_StackRole_Zigbee);
#endif
}



/*******************************************************************************
 * @fn      SetupZStackCallbacks
 *
 * @brief   Setup the Zstack Callbacks wanted
 *
 * @param   none
 *
 * @return  none
 */
static void SetupZStackCallbacks(void)
{
    zstack_devZDOCBReq_t zdoCBReq = {0};

    // Register for Callbacks, turn on:
    //  Device State Change,
    //  ZDO Match Descriptor Response,
    zdoCBReq.has_devStateChange = true;
    zdoCBReq.devStateChange = true;
    zdoCBReq.has_matchDescRsp = true;
    zdoCBReq.matchDescRsp = true;
    zdoCBReq.has_ieeeAddrRsp = true;
    zdoCBReq.ieeeAddrRsp = true;
#if defined Z_POWER_TEST
#if defined (POWER_TEST_POLL_DATA)
    zdoCBReq.has_deviceAnnounce = true;
    zdoCBReq.deviceAnnounce = true;
#endif
#endif // Z_POWER_TEST

    (void)Zstackapi_DevZDOCBReq(appServiceTaskId, &zdoCBReq);
}



/*********************************************************************
 * @fn          zclSampleLight_Init
 *
 * @brief       Initialization function for the zclGeneral layer.
 *
 * @param       none
 *
 * @return      none
 */
static void zclSampleLight_Init( void )
{
#ifdef BDB_REPORTING
      zstack_bdbRepAddAttrCfgRecordDefaultToListReq_t Req = {0};
#endif

  // Set destination address to indirect
  zclSampleLight_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  zclSampleLight_DstAddr.endPoint = 0;
  zclSampleLight_DstAddr.addr.shortAddr = 0;

  //Register Endpoint
  zclSampleLightEpDesc.endPoint = SAMPLELIGHT_ENDPOINT;
  zclSampleLightEpDesc.simpleDesc = &zclSampleLight_SimpleDesc;
  zclport_registerEndpoint(appServiceTaskId, &zclSampleLightEpDesc);

#if defined (ENABLE_GREENPOWER_COMBO_BASIC)
  zclGp_RegisterCBForGPDCommand(&zclSampleLight_GpSink_AppCallbacks);
#endif


  // Register the ZCL General Cluster Library callback functions
  zclGeneral_RegisterCmdCallbacks( SAMPLELIGHT_ENDPOINT, &zclSampleLight_CmdCallbacks );

  #ifdef ZCL_LIGHTING
  // >>>> Register the ZCL Lighting Cluster Library callback functions <<<<
  /* If this function throws an "undefined symbol" error, then be sure that files zcl_lighting.c and .h 
  are included within the build path */
  zclLighting_RegisterCmdCallbacks( SAMPLELIGHT_ENDPOINT, &zclSAEMS_CmdCallbacks );
  #endif // ZCL_LIGHTING

  // Register the application's attribute list and reset to default values
  zclSampleLight_ResetAttributesToDefaultValues();
  zcl_registerAttrList( SAMPLELIGHT_ENDPOINT, zclSampleLight_NumAttributes, zclSampleLight_Attrs );

  // Register the Application to receive the unprocessed Foundation command/response messages
  zclport_registerZclHandleExternal(SAMPLELIGHT_ENDPOINT, zclSampleLight_ProcessIncomingMsg);

#if !defined (DISABLE_GREENPOWER_BASIC_PROXY) && (ZG_BUILD_RTR_TYPE)
  gp_endpointInit(appServiceTaskId);
#endif

  //Write the bdb initialization parameters
  zclSampleLight_initParameters();

  //Setup ZDO callbacks
  SetupZStackCallbacks();

#if defined ( BDB_TL_INITIATOR ) && !defined(CUI_DISABLE)
  zclSampleLight_BdbCommissioningModes |= BDB_COMMISSIONING_MODE_INITIATOR_TL;
#endif

#ifdef ZCL_LEVEL_CTRL
  zclSampleLight_LevelLastLevel = zclSampleLight_getCurrentLevelAttribute();
#endif

#ifdef BDB_REPORTING
  //Adds the default configuration values for the temperature attribute of the ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT cluster, for endpoint SAMPLETEMPERATURESENSOR_ENDPOINT
  //Default maxReportingInterval value is 10 seconds
  //Default minReportingInterval value is 3 seconds
  //Default reportChange value is 300 (3 degrees)
  Req.attrID = ATTRID_ON_OFF_ON_OFF;
  Req.cluster = ZCL_CLUSTER_ID_GENERAL_ON_OFF;
  Req.endpoint = SAMPLELIGHT_ENDPOINT;
  Req.maxReportInt = 10;
  Req.minReportInt = 0;
  OsalPort_memcpy(Req.reportableChange,reportableChange,BDBREPORTING_MAX_ANALOG_ATTR_SIZE);

  Zstackapi_bdbRepAddAttrCfgRecordDefaultToListReq(appServiceTaskId,&Req);

#ifdef ZCL_LEVEL_CTRL
  Req.attrID = ATTRID_LEVEL_CURRENT_LEVEL;
  Req.cluster = ZCL_CLUSTER_ID_GENERAL_LEVEL_CONTROL;
  Req.endpoint = SAMPLELIGHT_ENDPOINT;
  Req.maxReportInt = 10;
  Req.minReportInt = 0;
  OsalPort_memcpy(Req.reportableChange,reportableChange,BDBREPORTING_MAX_ANALOG_ATTR_SIZE);

  Zstackapi_bdbRepAddAttrCfgRecordDefaultToListReq(appServiceTaskId,&Req);
#endif
#endif


#ifdef ZCL_DISCOVER
  // Register the application's command list
  zcl_registerCmdList( SAMPLELIGHT_ENDPOINT, zclCmdsArraySize, zclSampleLight_Cmds );
#endif

  zcl_registerReadWriteCB(SAMPLELIGHT_ENDPOINT,zclSampleLight_ReadWriteAttrCB,NULL);

#if defined (BDB_TL_TARGET) || defined (BDB_TL_INITIATOR)
  touchLinkApp_registerFindingBindingCb(tl_FindingBindingCb);
#endif // defined ( BDB_TL_TARGET ) || (BDB_TL_INITIATOR)

#if defined(USE_DMM) && defined(BLE_START)
  RemoteDisplay_registerClientProvCbs(zclSwitch_ProvissioningCbs);
  RemoteDisplay_registerLightCbs(zclSwitch_LightCbs);
#endif // defined(USE_DMM) && defined(BLE_START)

#if !defined(CUI_DISABLE) || defined(USE_DMM) && defined(BLE_START)
  // set up default application BDB commissioning modes based on build type
  if(ZG_BUILD_COORDINATOR_TYPE && ZG_DEVICE_COORDINATOR_TYPE)
  {
    zclSampleLight_BdbCommissioningModes = BDB_COMMISSIONING_MODE_NWK_FORMATION | BDB_COMMISSIONING_MODE_NWK_STEERING | BDB_COMMISSIONING_MODE_FINDING_BINDING;
  }
  else if (ZG_BUILD_JOINING_TYPE && ZG_DEVICE_JOINING_TYPE)
  {
    zclSampleLight_BdbCommissioningModes = BDB_COMMISSIONING_MODE_NWK_STEERING | BDB_COMMISSIONING_MODE_FINDING_BINDING;
  }
#endif // !defined(CUI_DISABLE) || defined(USE_DMM) && defined(BLE_START)

#ifndef CUI_DISABLE
  gCuiHandle = UI_Init( appServiceTaskId,                     // Application Task ID
           &appServiceTaskEvents,                // The events processed by the sample application
           appSemHandle,                         // Semaphore to post the events in the application thread
           &zclSampleLight_IdentifyTime,
           &zclSampleLight_BdbCommissioningModes,   // a pointer to the applications bdbCommissioningModes
           zclSampleLight_appStr,
           zclSampleLight_processKey,
           zclSampleLight_RemoveAppNvmData         // A pointer to the app-specific NV Item reset function
           );

  //Request the Red LED for App
  LED_Params ledParams;
  LED_Params_init(&ledParams);
  gRedLedHandle = LED_open(CONFIG_LED_RED, &ledParams);


  //Initialize the sampleLight UI status line
  zclSampleLight_InitializeStatusLine(gCuiHandle);
#endif


#if !defined (DISABLE_GREENPOWER_BASIC_PROXY) && (ZG_BUILD_RTR_TYPE)
  app_Green_Power_Init(appServiceTaskId, &appServiceTaskEvents, appSemHandle, SAMPLEAPP_PROCESS_GP_DATA_SEND_EVT,
                       SAMPLEAPP_PROCESS_GP_EXPIRE_DUPLICATE_EVT, SAMPLEAPP_PROCESS_GP_TEMP_MASTER_EVT);
#endif

#if defined ( BDB_TL_INITIATOR )
    touchLinkInitiatorApp_Init(appServiceTaskId);
#elif defined ( BDB_TL_TARGET )
    touchLinkTargetApp_Init(appServiceTaskId);
#endif

#ifdef PER_TEST
#ifndef CUI_DISABLE
  PERTest_init( appSemHandle, appServiceTaskId, gCuiHandle );
#else
  PERTest_init( appSemHandle, appServiceTaskId, NULL );
#endif
#endif

}

#ifndef CUI_DISABLE
/*********************************************************************
 * @fn          zclSampleLight_RemoveAppNvmData
 *
 * @brief       Callback when Application performs reset to Factory New Reset.
 *              Application must restore the application to default values
 *
 * @param       none
 *
 * @return      none
 */
static void zclSampleLight_RemoveAppNvmData(void)
{
#ifdef ZCL_GROUPS
    uint8_t numGroups;
    uint16_t groupList[APS_MAX_GROUPS];
    uint8_t i;

    if ( 0 != ( numGroups = aps_FindAllGroupsForEndpoint( SAMPLELIGHT_ENDPOINT, groupList ) ) )
    {
      for ( i = 0; i < numGroups; i++ )
      {
#if defined ( ZCL_SCENES )
        zclGeneral_RemoveAllScenes( SAMPLELIGHT_ENDPOINT, groupList[i] );
#endif
      }
      aps_RemoveAllGroup( SAMPLELIGHT_ENDPOINT );
    }
#endif
}
#endif

static void zclSampleLight_initParameters(void)
{
    zstack_bdbSetAttributesReq_t zstack_bdbSetAttrReq;

    zstack_bdbSetAttrReq.bdbCommissioningGroupID              = BDB_DEFAULT_COMMISSIONING_GROUP_ID;
    zstack_bdbSetAttrReq.bdbPrimaryChannelSet                 = BDB_DEFAULT_PRIMARY_CHANNEL_SET;
    zstack_bdbSetAttrReq.bdbScanDuration                      = BDB_DEFAULT_SCAN_DURATION;
    zstack_bdbSetAttrReq.bdbSecondaryChannelSet               = BDB_DEFAULT_SECONDARY_CHANNEL_SET;
    zstack_bdbSetAttrReq.has_bdbCommissioningGroupID          = TRUE;
    zstack_bdbSetAttrReq.has_bdbPrimaryChannelSet             = TRUE;
    zstack_bdbSetAttrReq.has_bdbScanDuration                  = TRUE;
    zstack_bdbSetAttrReq.has_bdbSecondaryChannelSet           = TRUE;
#if (ZG_BUILD_COORDINATOR_TYPE)
    zstack_bdbSetAttrReq.has_bdbJoinUsesInstallCodeKey        = TRUE;
    zstack_bdbSetAttrReq.has_bdbTrustCenterNodeJoinTimeout    = TRUE;
    zstack_bdbSetAttrReq.has_bdbTrustCenterRequireKeyExchange = TRUE;
    zstack_bdbSetAttrReq.bdbJoinUsesInstallCodeKey            = BDB_DEFAULT_JOIN_USES_INSTALL_CODE_KEY;
    zstack_bdbSetAttrReq.bdbTrustCenterNodeJoinTimeout        = BDB_DEFAULT_TC_NODE_JOIN_TIMEOUT;
    zstack_bdbSetAttrReq.bdbTrustCenterRequireKeyExchange     = BDB_DEFAULT_TC_REQUIRE_KEY_EXCHANGE;
#endif
#if (ZG_BUILD_JOINING_TYPE)
    zstack_bdbSetAttrReq.has_bdbTCLinkKeyExchangeAttemptsMax  = TRUE;
    zstack_bdbSetAttrReq.has_bdbTCLinkKeyExchangeMethod       = TRUE;
    zstack_bdbSetAttrReq.bdbTCLinkKeyExchangeAttemptsMax      = BDB_DEFAULT_TC_LINK_KEY_EXCHANGE_ATTEMPS_MAX;
    zstack_bdbSetAttrReq.bdbTCLinkKeyExchangeMethod           = BDB_DEFAULT_TC_LINK_KEY_EXCHANGE_METHOD;
#endif

    Zstackapi_bdbSetAttributesReq(appServiceTaskId, &zstack_bdbSetAttrReq);
}

/*******************************************************************************
 * @fn      zclSampleLight_initializeClocks
 *
 * @brief   Initialize Clocks
 *
 * @param   none
 *
 * @return  none
 */
static void zclSampleLight_initializeClocks(void)
{
#if ZG_BUILD_ENDDEVICE_TYPE
    // Initialize the timers needed for this application
    EndDeviceRejoinClkHandle = UtilTimer_construct(
    &EndDeviceRejoinClkStruct,
    zclSampleLight_processEndDeviceRejoinTimeoutCallback,
    SAMPLEAPP_END_DEVICE_REJOIN_DELAY,
    0, false, 0);
#endif
#if defined(USE_DMM) && defined(BLE_START)
    // Clock for synchronizing application configuration parameters for BLE
    UtilTimer_construct(
    &SyncAttrClkStruct,
    zclSampleLight_processSyncAttrTimeoutCallback,
    SAMPLEAPP_CONFIG_SYNC_TIMEOUT,
    SAMPLEAPP_CONFIG_SYNC_TIMEOUT, true, 0);
#endif // defined(USE_DMM) && defined(BLE_START)

    // Initialize the timers needed for this application
    DiscoveryClkHandle = UtilTimer_construct(
    &DiscoveryClkStruct,
    zclSampleLight_processDiscoveryTimeoutCallback,
    DISCOVERY_IN_PROGRESS_TIMEOUT,
    0, false, 0);

    // ==============================================================
    // =================== I2C Data Transfer Clock ==================
    SensorDataClkHandle = UtilTimer_construct(
    &SensorDataClkStruct,
    SAEMS_SensorsCallback,
    SENSOR_UPDATE_TIMEOUT,
    0, false, 0);
    // ==============================================================

    // =============================================================
    // =================== Motion Detection Clock ==================
    MotionSensorClkHandle = UtilTimer_construct(
    &MotionSensorClkStruct,
    SAEMS_MotionSensorCB,
    3000,
    0, false, 0); 
    // =============================================================

    // =============================================================
    // ==================== On-Off Ramping Clock ===================
    OnOffClkHandle = UtilTimer_construct(
    &OnOffClkStruct,
    SAEMS_OnOff_Timeout_Callback,
    25,
    0, false, 0); 
    // =============================================================

    // =============================================================
    // ================= Carbon Monoxide Alarm Clock ===============
    ALARM_CLKHANDLE = UtilTimer_construct(
    &ALARM_CLKSTRUCT,
    SAEMS_ALARM_handler,
    30000,  
    0, false, 0);
    // =============================================================    

    // =============================================================
    // ================= Carbon Monoxide Alarm Clock ===============
    CarbonMonoxide_Alarm_ClkHandle = UtilTimer_construct(
    &CarbonMonoxide_Alarm_ClkStruct,
    SAEMS_CarbonMonoxide_Alarm_handler,
    500,
    1000, false, 0);
    // =============================================================

    // =============================================================
    // ====================== Smoke Alarm Clock ====================
    Smoke_Alarm_ClkHandle = UtilTimer_construct(
    &Smoke_Alarm_ClkStruct,
    SAEMS_Smoke_Alarm_handler,
    1500,
    3000, false, 0);
    // =============================================================

}

#if ZG_BUILD_ENDDEVICE_TYPE
/*******************************************************************************
 * @fn      zclSampleLight_processEndDeviceRejoinTimeoutCallback
 *
 * @brief   Timeout handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
static void zclSampleLight_processEndDeviceRejoinTimeoutCallback(UArg a0)
{
    (void)a0; // Parameter is not used

    appServiceTaskEvents |= SAMPLEAPP_END_DEVICE_REJOIN_EVT;

    // Wake up the application thread when it waits for clock event
    Semaphore_post(appSemHandle);
}
#endif


/*******************************************************************************
 * @fn      zclSampleLight_processDiscoveryTimeoutCallback
 *
 * @brief   Timeout handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
static void zclSampleLight_processDiscoveryTimeoutCallback(UArg a0)
{
    (void)a0; // Parameter is not used

    appServiceTaskEvents |= SAMPLEAPP_DISCOVERY_TIMEOUT_EVT;

    // Wake up the application thread when it waits for clock event
    Semaphore_post(appSemHandle);
}

/*******************************************************************************
 * @fn      zclSampleLight_process_loop
 *
 * @brief   Application task processing start.
 *
 * @param   none
 *
 * @return  void
 */
static void zclSampleLight_process_loop(void)
{
    zclSampleLight_BdbCommissioningModes = BDB_COMMISSIONING_MODE_NWK_STEERING | BDB_COMMISSIONING_MODE_FINDING_BINDING;
    // Call BDB initialization. Should be called once from application at startup to restore
    // previous network configuration, if applicable.
    zstack_bdbStartCommissioningReq_t zstack_bdbStartCommissioningReq;
    zstack_bdbStartCommissioningReq.commissioning_mode = zclSampleLight_BdbCommissioningModes;
    Zstackapi_bdbStartCommissioningReq(appServiceTaskId,&zstack_bdbStartCommissioningReq);

    // Start the I2C Data Transfer Clock
    printf("Sensor Data Transfer Clock has been started ... \n");
    UtilTimer_setTimeout(SensorDataClkHandle, SENSOR_UPDATE_TIMEOUT);
    UtilTimer_start(&SensorDataClkStruct);

    printf("Now entering the process loop ... \n");
    /* Forever loop */
    for(;;)
    {
        zstackmsg_genericReq_t *pMsg = NULL;
        bool msgProcessed = FALSE;
        /* Wait for response message */
        if(Semaphore_pend(appSemHandle, BIOS_WAIT_FOREVER ))
        {
            /* Retrieve the response message */
            if((pMsg = (zstackmsg_genericReq_t*) OsalPort_msgReceive( appServiceTaskId )) != NULL){
                zclSampleLight_processZStackMsgs(pMsg);
                // Free any separately allocated memory
                msgProcessed = Zstackapi_freeIndMsg(pMsg);
            }
            if((msgProcessed == FALSE) && (pMsg != NULL)){
                OsalPort_msgDeallocate((uint8_t*)pMsg);
            }

            if( appServiceTaskEvents & SAMPLELIGHT_LEVEL_CTRL_EVT ){
              SAEMS_OnOff_Ramp();
              appServiceTaskEvents &= ~SAMPLELIGHT_LEVEL_CTRL_EVT;
            }

            if( appServiceTaskEvents & SAMPLELIGHT_POLL_CONTROL_TIMEOUT_EVT ){
              SAEMS_getSensorData();
              SAEMS_updateSensorData();
              
              // Reset and restart the timer
              UtilTimer_setTimeout(SensorDataClkHandle, SENSOR_UPDATE_TIMEOUT);
              UtilTimer_start(&SensorDataClkStruct);
              
              appServiceTaskEvents &= ~SAMPLELIGHT_POLL_CONTROL_TIMEOUT_EVT;
            }

            if ( appServiceTaskEvents & SAMPLEAPP_DISCOVERY_TIMEOUT_EVT ){
              discoveryInprogress = FALSE;
              appServiceTaskEvents &= ~SAMPLEAPP_DISCOVERY_TIMEOUT_EVT;
            }

#if defined (BDB_TL_TARGET) || defined (BDB_TL_INITIATOR)
            if(appServiceTaskEvents & TL_BDB_FB_EVT){
                zstack_bdbStartCommissioningReq_t zstack_bdbStartCommissioningReq;
                zstack_bdbStartCommissioningReq.commissioning_mode = BDB_COMMISSIONING_MODE_FINDING_BINDING;
                Zstackapi_bdbStartCommissioningReq(appServiceTaskId, &zstack_bdbStartCommissioningReq);
                appServiceTaskEvents &= ~TL_BDB_FB_EVT;
            }
#endif // defined ( BDB_TL_TARGET ) || (BDB_TL_INITIATOR)


#if ZG_BUILD_ENDDEVICE_TYPE
            if ( appServiceTaskEvents & SAMPLEAPP_END_DEVICE_REJOIN_EVT )
            {
              zstack_bdbZedAttemptRecoverNwkRsp_t zstack_bdbZedAttemptRecoverNwkRsp;

              Zstackapi_bdbZedAttemptRecoverNwkReq(appServiceTaskId,&zstack_bdbZedAttemptRecoverNwkRsp);

              appServiceTaskEvents &= ~SAMPLEAPP_END_DEVICE_REJOIN_EVT;
            }
#endif
        }
    }
}

#if defined (BDB_TL_TARGET) || defined (BDB_TL_INITIATOR)
static void tl_BDBFindingBindingCb(void)
{
  OsalPortTimers_startTimer(appServiceTaskId, TL_BDB_FB_EVT, TL_BDB_FB_DELAY);
}
#endif // defined ( BDB_TL_TARGET ) || defined (BDB_TL_INITIATOR)


/*********************************************************************
 * @fn      zclSampleLight_IdentifyQueryRspCB
 *
 * @brief   Callback from the ZCL General Cluster Library
 *          to process a Indentify Query Rsp.
 *
 * @param   none
 *
 * @return  none
 */
static void zclSampleLight_IdentifyQueryRspCB(zclIdentifyQueryRsp_t *pRsp)
{
    zstack_zdoMatchDescReq_t Req;
    uint16_t  OnOffCluster = ZCL_CLUSTER_ID_GENERAL_ON_OFF;

    if(discoveryInprogress)
    {
        Req.dstAddr = pRsp->srcAddr->addr.shortAddr;
        Req.nwkAddrOfInterest = pRsp->srcAddr->addr.shortAddr;
        Req.profileID = ZCL_HA_PROFILE_ID;

        endPointDiscovered = pRsp->srcAddr->endPoint;

        Req.n_inputClusters = 0;
        Req.n_outputClusters = 1;

        Req.pOutputClusters = &OnOffCluster;

        Zstackapi_ZdoMatchDescReq(appServiceTaskId, &Req);
    }
}


/*******************************************************************************
 * @fn      zclSampleLight_processZStackMsgs
 *
 * @brief   Process event from Stack
 *
 * @param   pMsg - pointer to incoming ZStack message to process
 *
 * @return  void
 */
static void zclSampleLight_processZStackMsgs(zstackmsg_genericReq_t *pMsg)
{
    switch(pMsg->hdr.event)
    {
        case zstackmsg_CmdIDs_BDB_NOTIFICATION:
            {
                zstackmsg_bdbNotificationInd_t *pInd;
                pInd = (zstackmsg_bdbNotificationInd_t*)pMsg;
                zclSampleLight_ProcessCommissioningStatus(&(pInd->Req));
            }
            break;

        case zstackmsg_CmdIDs_BDB_IDENTIFY_TIME_CB:
            {
#ifndef CUI_DISABLE
                zstackmsg_bdbIdentifyTimeoutInd_t *pInd;
                pInd = (zstackmsg_bdbIdentifyTimeoutInd_t*) pMsg;
                uiProcessIdentifyTimeChange(&(pInd->EndPoint));
#endif
            }
            break;

        case zstackmsg_CmdIDs_BDB_BIND_NOTIFICATION_CB:
            {
#ifndef CUI_DISABLE
                zstackmsg_bdbBindNotificationInd_t *pInd;
                pInd = (zstackmsg_bdbBindNotificationInd_t*) pMsg;
                uiProcessBindNotification(&(pInd->Req));
#endif
            }
            break;

        case zstackmsg_CmdIDs_AF_INCOMING_MSG_IND:
            {
                // Process incoming data messages
                zstackmsg_afIncomingMsgInd_t *pInd;
                pInd = (zstackmsg_afIncomingMsgInd_t *)pMsg;
                zclSampleLight_processAfIncomingMsgInd( &(pInd->req) );
            }
            break;

#if (ZG_BUILD_JOINING_TYPE)
        case zstackmsg_CmdIDs_BDB_CBKE_TC_LINK_KEY_EXCHANGE_IND:
        {
          zstack_bdbCBKETCLinkKeyExchangeAttemptReq_t zstack_bdbCBKETCLinkKeyExchangeAttemptReq;
          /* Z3.0 has not defined CBKE yet, so lets attempt default TC Link Key exchange procedure
           * by reporting CBKE failure.
           */

          zstack_bdbCBKETCLinkKeyExchangeAttemptReq.didSuccess = FALSE;

          Zstackapi_bdbCBKETCLinkKeyExchangeAttemptReq(appServiceTaskId,
                                                       &zstack_bdbCBKETCLinkKeyExchangeAttemptReq);
        }
        break;

        case zstackmsg_CmdIDs_BDB_FILTER_NWK_DESCRIPTOR_IND:

         /*   User logic to remove networks that do not want to join
          *   Networks to be removed can be released with Zstackapi_bdbNwkDescFreeReq
          */

          Zstackapi_bdbFilterNwkDescComplete(appServiceTaskId);
        break;

#endif
        case zstackmsg_CmdIDs_DEV_STATE_CHANGE_IND:
        {
#if !defined(CUI_DISABLE) || defined(USE_DMM) && defined(BLE_START)
            // The ZStack Thread is indicating a State change
            zstackmsg_devStateChangeInd_t *pInd =
                (zstackmsg_devStateChangeInd_t *)pMsg;
#endif // !defined(CUI_DISABLE) || defined(USE_DMM) && defined(BLE_START)

#ifndef CUI_DISABLE
            UI_DeviceStateUpdated(&(pInd->req));
#endif

#if defined(USE_DMM) && defined(BLE_START)
            provState = pInd->req.state;

            appServiceTaskEvents |= SAMPLEAPP_POLICY_UPDATE_EVT;

            // Wake up the application thread when it waits for clock event
            Semaphore_post(appSemHandle);
#endif // defined(USE_DMM) && defined(BLE_START)
        }
        break;



        /*
         * These are messages/indications from ZStack that this
         * application doesn't process.  These message can be
         * processed by your application, remove from this list and
         * process them here in this switch statement.
         */

#if !defined (DISABLE_GREENPOWER_BASIC_PROXY) && (ZG_BUILD_RTR_TYPE)
          case zstackmsg_CmdIDs_GP_DATA_IND:
          {
              zstackmsg_gpDataInd_t *pInd;
              pInd = (zstackmsg_gpDataInd_t*)pMsg;
              gp_processDataIndMsg( &(pInd->Req) );
          }
          break;

          case zstackmsg_CmdIDs_GP_SECURITY_REQ:
          {
              zstackmsg_gpSecReq_t *pInd;
              pInd = (zstackmsg_gpSecReq_t*)pMsg;
              gp_processSecRecMsg( &(pInd->Req) );
          }
          break;

          case zstackmsg_CmdIDs_GP_CHECK_ANNCE:
          {
              zstackmsg_gpCheckAnnounce_t *pInd;
              pInd = (zstackmsg_gpCheckAnnounce_t*)pMsg;
              gp_processCheckAnnceMsg( &(pInd->Req) );
          }
          break;

          case zstackmsg_CmdIDs_GP_COMMISSIONING_MODE_IND:
          {
#ifndef CUI_DISABLE
            zstackmsg_gpCommissioningModeInd_t *pInd;
            pInd = (zstackmsg_gpCommissioningModeInd_t*)pMsg;
            UI_SetGPPCommissioningMode( &(pInd->Req) );
#endif
          }
          break;
#endif

#ifdef BDB_TL_TARGET
        case zstackmsg_CmdIDs_BDB_TOUCHLINK_TARGET_ENABLE_IND:
        {
          UI_UpdateBdbStatusLine(NULL);
        }
        break;
#endif

        case zstackmsg_CmdIDs_ZDO_DEVICE_ANNOUNCE:
#if defined (Z_POWER_TEST)
#if defined (POWER_TEST_POLL_DATA)
        {
          zstackmsg_zdoDeviceAnnounceInd_t *pInd;
          pInd = (zstackmsg_zdoDeviceAnnounceInd_t*)pMsg;

          // save the short address of the ZED to send ZCL test data
          powerTestZEDAddr = pInd->req.srcAddr;

          // start periodic timer for sending ZCL data to zed
          OsalPortTimers_startReloadTimer(appServiceTaskId, SAMPLEAPP_POWER_TEST_ZCL_DATA_EVT, Z_POWER_TEST_DATA_TX_INTERVAL);
        }
#endif
#else
        break;
#endif

        case zstackmsg_CmdIDs_ZDO_MATCH_DESC_RSP:
        {
            zstackmsg_zdoMatchDescRspInd_t *pInd =
                    (zstackmsg_zdoMatchDescRspInd_t *)pMsg;
            if((pInd->rsp.status == zstack_ZdpStatus_SUCCESS) && (pInd->rsp.n_matchList))
            {
                zstack_zdoIeeeAddrReq_t Req;

                while((*(pInd->rsp.pMatchList) != endPointDiscovered) && (pInd->rsp.pMatchList != NULL))
                {
                    pInd->rsp.pMatchList++;
                }

                if(pInd->rsp.pMatchList != NULL)
                {
                    Req.startIndex = 0;
                    Req.type = zstack_NwkAddrReqType_SINGLE_DEVICE;
                    Req.nwkAddr = pInd->rsp.nwkAddrOfInterest;

                    Zstackapi_ZdoIeeeAddrReq(appServiceTaskId, &Req);
                }
                else
                {
                    endPointDiscovered = 0;
                }
            }

        }
        break;

        case zstackmsg_CmdIDs_ZDO_IEEE_ADDR_RSP:
        {
            zstackmsg_zdoIeeeAddrRspInd_t *pInd =
                    (zstackmsg_zdoIeeeAddrRspInd_t *)pMsg;

            if(pInd->rsp.status == zstack_ZdpStatus_SUCCESS)
            {
                zstack_zdoBindReq_t Req;
                zstack_sysNwkInfoReadRsp_t  Rsp;

                //Get our IEEE address
                Zstackapi_sysNwkInfoReadReq(appServiceTaskId, &Rsp);


                //fill in bind request to self
                Req.nwkAddr = Rsp.nwkAddr;
                OsalPort_memcpy(Req.bindInfo.srcAddr, Rsp.ieeeAddr, Z_EXTADDR_LEN);
                Req.bindInfo.dstAddr.addrMode = zstack_AFAddrMode_EXT;
                OsalPort_memcpy(Req.bindInfo.dstAddr.addr.extAddr, pInd->rsp.ieeeAddr, Z_EXTADDR_LEN);
                Req.bindInfo.dstAddr.endpoint = endPointDiscovered;
                Req.bindInfo.clusterID = ZCL_CLUSTER_ID_GENERAL_ON_OFF;
                Req.bindInfo.srcEndpoint = SAMPLELIGHT_ENDPOINT;

                //create the bind to that device
                Zstackapi_ZdoBindReq(appServiceTaskId, &Req);
            }

        }
        break;


        case zstackmsg_CmdIDs_DEV_PERMIT_JOIN_IND:
        case zstackmsg_CmdIDs_BDB_TC_LINK_KEY_EXCHANGE_NOTIFICATION_IND:
        case zstackmsg_CmdIDs_AF_DATA_CONFIRM_IND:
        case zstackmsg_CmdIDs_ZDO_NWK_ADDR_RSP:
        case zstackmsg_CmdIDs_ZDO_NODE_DESC_RSP:
        case zstackmsg_CmdIDs_ZDO_POWER_DESC_RSP:
        case zstackmsg_CmdIDs_ZDO_SIMPLE_DESC_RSP:
        case zstackmsg_CmdIDs_ZDO_ACTIVE_EP_RSP:
        case zstackmsg_CmdIDs_ZDO_COMPLEX_DESC_RSP:
        case zstackmsg_CmdIDs_ZDO_USER_DESC_RSP:
        case zstackmsg_CmdIDs_ZDO_USER_DESC_SET_RSP:
        case zstackmsg_CmdIDs_ZDO_SERVER_DISC_RSP:
        case zstackmsg_CmdIDs_ZDO_END_DEVICE_BIND_RSP:
        case zstackmsg_CmdIDs_ZDO_BIND_RSP:
        case zstackmsg_CmdIDs_ZDO_UNBIND_RSP:
        case zstackmsg_CmdIDs_ZDO_MGMT_NWK_DISC_RSP:
        case zstackmsg_CmdIDs_ZDO_MGMT_LQI_RSP:
        case zstackmsg_CmdIDs_ZDO_MGMT_RTG_RSP:
        case zstackmsg_CmdIDs_ZDO_MGMT_BIND_RSP:
        case zstackmsg_CmdIDs_ZDO_MGMT_LEAVE_RSP:
        case zstackmsg_CmdIDs_ZDO_MGMT_DIRECT_JOIN_RSP:
        case zstackmsg_CmdIDs_ZDO_MGMT_PERMIT_JOIN_RSP:
        case zstackmsg_CmdIDs_ZDO_MGMT_NWK_UPDATE_NOTIFY:
        case zstackmsg_CmdIDs_ZDO_SRC_RTG_IND:
        case zstackmsg_CmdIDs_ZDO_CONCENTRATOR_IND:
        case zstackmsg_CmdIDs_ZDO_LEAVE_CNF:
        case zstackmsg_CmdIDs_ZDO_LEAVE_IND:
        case zstackmsg_CmdIDs_SYS_RESET_IND:
        case zstackmsg_CmdIDs_AF_REFLECT_ERROR_IND:
        case zstackmsg_CmdIDs_ZDO_TC_DEVICE_IND:
            break;

        default:
            break;
    }
}



/*******************************************************************************
 *
 * @fn          zclSampleLight_processAfIncomingMsgInd
 *
 * @brief       Process AF Incoming Message Indication message
 *
 * @param       pInMsg - pointer to incoming message
 *
 * @return      none
 *
 */
static void zclSampleLight_processAfIncomingMsgInd(zstack_afIncomingMsgInd_t *pInMsg)
{
    afIncomingMSGPacket_t afMsg;

    /*
     * All incoming messages are passed to the ZCL message processor,
     * first convert to a structure that ZCL can process.
     */
    afMsg.groupId = pInMsg->groupID;
    afMsg.clusterId = pInMsg->clusterId;
    afMsg.srcAddr.endPoint = pInMsg->srcAddr.endpoint;
    afMsg.srcAddr.panId = pInMsg->srcAddr.panID;
    afMsg.srcAddr.addrMode = (afAddrMode_t)pInMsg->srcAddr.addrMode;
    if( (afMsg.srcAddr.addrMode == afAddr16Bit)
        || (afMsg.srcAddr.addrMode == afAddrGroup)
        || (afMsg.srcAddr.addrMode == afAddrBroadcast) )
    {
        afMsg.srcAddr.addr.shortAddr = pInMsg->srcAddr.addr.shortAddr;
    }
    else if(afMsg.srcAddr.addrMode == afAddr64Bit)
    {
        OsalPort_memcpy(afMsg.srcAddr.addr.extAddr, &(pInMsg->srcAddr.addr.extAddr), 8);
    }
    afMsg.macDestAddr = pInMsg->macDestAddr;
    afMsg.endPoint = pInMsg->endpoint;
    afMsg.wasBroadcast = pInMsg->wasBroadcast;
    afMsg.LinkQuality = pInMsg->linkQuality;
    afMsg.correlation = pInMsg->correlation;
    afMsg.rssi = pInMsg->rssi;
    afMsg.SecurityUse = pInMsg->securityUse;
    afMsg.timestamp = pInMsg->timestamp;
    afMsg.nwkSeqNum = pInMsg->nwkSeqNum;
    afMsg.macSrcAddr = pInMsg->macSrcAddr;
    afMsg.radius = pInMsg->radius;
    afMsg.cmd.DataLength = pInMsg->n_payload;
    afMsg.cmd.Data = pInMsg->pPayload;

    zcl_ProcessMessageMSG(&afMsg);

    // Incoming Alarm Signal Handling
    //  check to see if the message was a broadcast message
    if(afMsg.wasBroadcast){
      // If the ClusterId is FF01 - raise the CO Alarm
      if(afMsg.clusterId == SAEMS_CO_ALARM_CLUSTER_ID)
        CO_ALARM = true;
      // If the ClusterIF is FF02 - raise the Smoke Alarm
      else if(afMsg.clusterId == SAEMS_SMOKE_ALARM_CLUSTER_ID)
        SMOKE_ALARM = true;
    }
}


/*********************************************************************
 * @fn      zclSampleLight_ProcessCommissioningStatus
 *
 * @brief   Callback in which the status of the commissioning process are reported
 *
 * @param   bdbCommissioningModeMsg - Context message of the status of a commissioning process
 *
 * @return  none
 */
static void zclSampleLight_ProcessCommissioningStatus(bdbCommissioningModeMsg_t *bdbCommissioningModeMsg)
{
  switch(bdbCommissioningModeMsg->bdbCommissioningMode)
  {
    case BDB_COMMISSIONING_FORMATION:
      if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_SUCCESS)
      {
        //YOUR JOB:
      }
      else
      {
        //Want to try other channels?
        //try with bdb_setChannelAttribute
      }
    break;
    case BDB_COMMISSIONING_NWK_STEERING:
      if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_SUCCESS)
      {
        //YOUR JOB:
        //We are on the nwk, what now?
      }
      else
      {
        //See the possible errors for nwk steering procedure
        //No suitable networks found
        //Want to try other channels?
        //try with bdb_setChannelAttribute
      }
    break;
    case BDB_COMMISSIONING_FINDING_BINDING:
      if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_SUCCESS)
      {
        //YOUR JOB:
      }
      else
      {
        //YOUR JOB:
        //retry?, wait for user interaction?
      }
    break;
    case BDB_COMMISSIONING_INITIALIZATION:
      //Initialization notification can only be successful. Failure on initialization
      //only happens for ZED and is notified as BDB_COMMISSIONING_PARENT_LOST notification

      //YOUR JOB:
      //We are on a network, what now?

    break;
#if ZG_BUILD_ENDDEVICE_TYPE
    case BDB_COMMISSIONING_PARENT_LOST:
      if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_NETWORK_RESTORED)
      {
        //We did recover from losing parent
      }
      else
      {
        //Parent not found, attempt to rejoin again after a fixed delay
        UtilTimer_setTimeout( EndDeviceRejoinClkHandle, SAMPLEAPP_END_DEVICE_REJOIN_DELAY );
        UtilTimer_start(&EndDeviceRejoinClkStruct);
      }
    break;
#endif
  }
#ifndef CUI_DISABLE
  UI_UpdateBdbStatusLine(bdbCommissioningModeMsg);
#endif
}



/*********************************************************************
 * @fn      zclSampleLight_BasicResetCB
 *
 * @brief   Callback from the ZCL General Cluster Library
 *          to set all the Basic Cluster attributes to default values.
 *
 * @param   none
 *
 * @return  none
 */
static void zclSampleLight_BasicResetCB( void )
{
  //Reset every attribute in all supported cluster to their default value.

  zclSampleLight_ResetAttributesToDefaultValues();

#ifndef CUI_DISABLE
  zclSampleLight_UpdateStatusLine();
#endif
}

/******************************************************************************
 *
 *  Functions for processing ZCL Foundation incoming Command/Response messages
 *
 *****************************************************************************/

/*********************************************************************
 * @fn      zclSampleLight_SceneRecallCB
 *
 * @brief   Callback from the ZCL Scenes Cluster Library
 *          to recall a set of attributes from a stored scene.
 *
 * @param   none
 *
 * @return  none
 */
static void zclSampleLight_SceneRecallCB( zclSceneReq_t *pReq )
{
    zclGeneral_Scene_extField_t extField;
    uint8_t *pBuf;
    uint8_t extLen = 0;

    pBuf = pReq->scene->extField;
    extField.AttrLen = pBuf[2];

     while(extLen < ZCL_GENERAL_SCENE_EXT_LEN)
     {
         //Parse ExtField
         extField.ClusterID = BUILD_UINT16(pBuf[0],pBuf[1]);
         extField.AttrLen = pBuf[2];
         extField.AttrBuf = &pBuf[3];

         if(extField.AttrLen == 0xFF || extField.ClusterID == 0xFFFF)
         {
             break;
         }

         //If On/Off then retrieve the attribute
         if(extField.ClusterID == ZCL_CLUSTER_ID_GENERAL_ON_OFF)
         {
             uint8_t tempState = *extField.AttrBuf;
             zclSampleLight_updateOnOffAttribute(tempState);
         }
#ifdef ZCL_LVL_CTRL
         //If Level Control then retrieve the attribute
         else if(extField.ClusterID == ZCL_CLUSTER_ID_GENERAL_LEVEL_CONTROL)
         {
             uint8_t tempState = *extField.AttrBuf;
             zclSampleLight_updateCurrentLevelAttribute(tempState);
         }
#endif
         //Move to the next extension field (increase pointer by clusterId, Attribute len field, and attribute)
         pBuf += sizeof(uint16_t) + sizeof(uint8_t) + extField.AttrLen;
         extLen += sizeof(uint16_t) + sizeof(uint8_t) + extField.AttrLen;  //increase ExtField
     }

    //Update scene attributes
    zclSampleLight_ScenesValid = TRUE;
    zclSampleLight_ScenesCurrentGroup = pReq->scene->groupID;
    zclSampleLight_ScenesCurrentScene = pReq->scene->ID;

}


/*********************************************************************
 * @fn      zclSampleLight_SceneStoreCB
 *
 * @brief   Callback from the ZCL Scenes Cluster Library
 *          to store a set of attributes to a specific scene.
 *
 * @param   none
 *
 * @return  none
 */
static uint8_t zclSampleLight_SceneStoreCB( zclSceneReq_t *pReq )
{
    zclGeneral_Scene_extField_t extField;
    uint8_t *pBuf;
    uint8_t extLen = 0;
    uint8_t sceneChanged = FALSE;

    pBuf = pReq->scene->extField;
    extField.AttrLen = pBuf[2];

    while(extLen < ZCL_GENERAL_SCENE_EXT_LEN)
    {
        //Parse ExtField
        extField.ClusterID = BUILD_UINT16(pBuf[0],pBuf[1]);
        extField.AttrLen = pBuf[2];
        extField.AttrBuf = &pBuf[3];

        if(extField.AttrLen == 0xFF || extField.ClusterID == 0xFFFF)
        {
            break;
        }

        //If On/Off then store attribute
        if(extField.ClusterID == ZCL_CLUSTER_ID_GENERAL_ON_OFF)
        {
            uint8_t tempState = zclSampleLight_getOnOffAttribute();
            if(*extField.AttrBuf != tempState )
            {
                sceneChanged = TRUE;
            }
            *extField.AttrBuf = tempState;
        }
#ifdef ZCL_LVL_CTRL
        //If Level Control then store attribute
        else if(extField.ClusterID == ZCL_CLUSTER_ID_GENERAL_LEVEL_CONTROL)
        {
            uint8_t tempState = zclSampleLight_getCurrentLevelAttribute();
            if(*extField.AttrBuf != tempState )
            {
                sceneChanged = TRUE;
            }
            *extField.AttrBuf = tempState;
        }
#endif
        //Move to the next extension field (increase pointer by clusterId, Attribute len field, and attribute)
        pBuf += sizeof(uint16_t) + sizeof(uint8_t) + extField.AttrLen;
        extLen += sizeof(uint16_t) + sizeof(uint8_t) + extField.AttrLen;  //increase ExtField
    }

    //Update scene attributes
    zclSampleLight_ScenesValid = TRUE;
    zclSampleLight_ScenesCurrentGroup = pReq->scene->groupID;
    zclSampleLight_ScenesCurrentScene = pReq->scene->ID;

    return sceneChanged;
}




/*********************************************************************
 * @fn      zclSampleLight_ReadWriteAttrCB
 *
 * @brief   Handle ATTRID_SCENES_SCENE_COUNT, ATTRID_ON_OFF_ON_OFF and ATTRID_LEVEL_CURRENT_LEVEL.
 *          Only to be called if any of the attributes change.
 *
 * @param   clusterId - cluster that attribute belongs to
 * @param   attrId - attribute to be read or written
 * @param   oper - ZCL_OPER_LEN, ZCL_OPER_READ, or ZCL_OPER_WRITE
 * @param   pValue - pointer to attribute value, OTA endian
 * @param   pLen - length of attribute value read, native endian
 *
 * @return  status
 */
ZStatus_t zclSampleLight_ReadWriteAttrCB( uint16_t clusterId, uint16_t attrId, uint8_t oper,
                                         uint8_t *pValue, uint16_t *pLen )
{
    if(clusterId == ZCL_CLUSTER_ID_GENERAL_SCENES)
    {
        if(attrId == ATTRID_SCENES_SCENE_COUNT)
        {
           return zclGeneral_ReadSceneCountCB(clusterId,attrId,oper,pValue,pLen);
        }
    }
    return ZSuccess;
}





/*********************************************************************
 * @fn      zclSampleLight_ProcessIncomingMsg
 *
 * @brief   Process ZCL Foundation incoming message
 *
 * @param   pInMsg - pointer to the received message
 *
 * @return  uint8_t - TRUE if got handled
 */
static uint8_t zclSampleLight_ProcessIncomingMsg( zclIncoming_t *pInMsg )
{
  uint8_t handled = FALSE;

  switch ( pInMsg->hdr.commandID )
  {
#ifdef ZCL_READ
    case ZCL_CMD_READ_RSP:
      zclSampleLight_ProcessInReadRspCmd( pInMsg );
      handled = TRUE;
      break;
#endif
#ifdef ZCL_WRITE
    case ZCL_CMD_WRITE_RSP:
      zclSampleLight_ProcessInWriteRspCmd( pInMsg );
      handled = TRUE;
      break;
#endif
    case ZCL_CMD_CONFIG_REPORT:
    case ZCL_CMD_CONFIG_REPORT_RSP:
    case ZCL_CMD_READ_REPORT_CFG:
    case ZCL_CMD_READ_REPORT_CFG_RSP:
    case ZCL_CMD_REPORT:
      //bdb_ProcessIncomingReportingMsg( pInMsg );
      break;

    case ZCL_CMD_DEFAULT_RSP:
      zclSampleLight_ProcessInDefaultRspCmd( pInMsg );
      handled = TRUE;
      break;
#ifdef ZCL_DISCOVER
    case ZCL_CMD_DISCOVER_CMDS_RECEIVED_RSP:
      zclSampleLight_ProcessInDiscCmdsRspCmd( pInMsg );
      handled = TRUE;
      break;

    case ZCL_CMD_DISCOVER_CMDS_GEN_RSP:
      zclSampleLight_ProcessInDiscCmdsRspCmd( pInMsg );
      handled = TRUE;
      break;

    case ZCL_CMD_DISCOVER_ATTRS_RSP:
      zclSampleLight_ProcessInDiscAttrsRspCmd( pInMsg );
      handled = TRUE;
      break;

    case ZCL_CMD_DISCOVER_ATTRS_EXT_RSP:
      zclSampleLight_ProcessInDiscAttrsExtRspCmd( pInMsg );
      handled = TRUE;
      break;
#endif
    default:
      break;
  }

  return handled;
}

#ifdef ZCL_READ
/*********************************************************************
 * @fn      zclSampleLight_ProcessInReadRspCmd
 *
 * @brief   Process the "Profile" Read Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8_t zclSampleLight_ProcessInReadRspCmd( zclIncoming_t *pInMsg )
{
  zclReadRspCmd_t *readRspCmd;
  uint8_t i;

  readRspCmd = (zclReadRspCmd_t *)pInMsg->attrCmd;
  for (i = 0; i < readRspCmd->numAttr; i++)
  {
    // Notify the originator of the results of the original read attributes
    // attempt and, for each successfull request, the value of the requested
    // attribute
  }

  return ( TRUE );
}
#endif // ZCL_READ

#ifdef ZCL_WRITE
/*********************************************************************
 * @fn      zclSampleLight_ProcessInWriteRspCmd
 *
 * @brief   Process the "Profile" Write Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8_t zclSampleLight_ProcessInWriteRspCmd( zclIncoming_t *pInMsg )
{
  zclWriteRspCmd_t *writeRspCmd;
  uint8_t i;

  writeRspCmd = (zclWriteRspCmd_t *)pInMsg->attrCmd;
  for ( i = 0; i < writeRspCmd->numAttr; i++ )
  {
    // Notify the device of the results of the its original write attributes
    // command.
  }

  return ( TRUE );
}
#endif // ZCL_WRITE

/*********************************************************************
 * @fn      zclSampleLight_ProcessInDefaultRspCmd
 *
 * @brief   Process the "Profile" Default Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8_t zclSampleLight_ProcessInDefaultRspCmd( zclIncoming_t *pInMsg )
{
  // zclDefaultRspCmd_t *defaultRspCmd = (zclDefaultRspCmd_t *)pInMsg->attrCmd;

  // Device is notified of the Default Response command.
  (void)pInMsg;

  return ( TRUE );
}

#ifdef ZCL_DISCOVER
/*********************************************************************
 * @fn      zclSampleLight_ProcessInDiscCmdsRspCmd
 *
 * @brief   Process the Discover Commands Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8_t zclSampleLight_ProcessInDiscCmdsRspCmd( zclIncoming_t *pInMsg )
{
  zclDiscoverCmdsCmdRsp_t *discoverRspCmd;
  uint8_t i;

  discoverRspCmd = (zclDiscoverCmdsCmdRsp_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numCmd; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return ( TRUE );
}

/*********************************************************************
 * @fn      zclSampleLight_ProcessInDiscAttrsRspCmd
 *
 * @brief   Process the "Profile" Discover Attributes Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8_t zclSampleLight_ProcessInDiscAttrsRspCmd( zclIncoming_t *pInMsg )
{
  zclDiscoverAttrsRspCmd_t *discoverRspCmd;
  uint8_t i;

  discoverRspCmd = (zclDiscoverAttrsRspCmd_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numAttr; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return ( TRUE );
}

/*********************************************************************
 * @fn      zclSampleLight_ProcessInDiscAttrsExtRspCmd
 *
 * @brief   Process the "Profile" Discover Attributes Extended Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8_t zclSampleLight_ProcessInDiscAttrsExtRspCmd( zclIncoming_t *pInMsg )
{
  zclDiscoverAttrsExtRsp_t *discoverRspCmd;
  uint8_t i;

  discoverRspCmd = (zclDiscoverAttrsExtRsp_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numAttr; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return ( TRUE );
}
#endif // ZCL_DISCOVER


void zclSampleLight_UiActionToggleLight(const int32_t _itemEntry)
{
  SAEMS_OnOffCB(COMMAND_ON_OFF_TOGGLE);
}




void zclSampleLight_UiActionSwitchDiscovery(const int32_t _itemEntry)
{

    afAddrType_t destAddr;
    zstack_getZCLFrameCounterRsp_t Rsp;
    discoveryInprogress = TRUE;


    UtilTimer_setTimeout( DiscoveryClkHandle, DISCOVERY_IN_PROGRESS_TIMEOUT );
    UtilTimer_start(&DiscoveryClkStruct);

    destAddr.endPoint = 0xFF;
    destAddr.addrMode = afAddr16Bit;
    destAddr.addr.shortAddr = 0xFFFF;

    Zstackapi_getZCLFrameCounterReq(appServiceTaskId, &Rsp);

    zclGeneral_SendIdentifyQuery(SAMPLELIGHT_ENDPOINT, &destAddr, FALSE, Rsp.zclFrameCounter);

}



/****************************************************************************
****************************************************************************/


#ifndef CUI_DISABLE
/*********************************************************************
 * @fn      zclSampleLight_processKey
 *
 * @brief   Key event handler function
 *
 * @param   key - key to handle action for
 *          buttonEvents - event to handle action for
 *
 * @return  none
 */
static void zclSampleLight_processKey(uint8_t key, Button_EventMask buttonEvents)
{
    if (buttonEvents & Button_EV_CLICKED)
    {
        if(key == CONFIG_BTN_LEFT)
        {
            zstack_bdbStartCommissioningReq_t zstack_bdbStartCommissioningReq;

            zstack_bdbStartCommissioningReq.commissioning_mode = zclSampleLight_BdbCommissioningModes;
            Zstackapi_bdbStartCommissioningReq(appServiceTaskId, &zstack_bdbStartCommissioningReq);
        }
        if(key == CONFIG_BTN_RIGHT)
        {

        }
    }
}

#ifdef DMM_OAD
/*********************************************************************
 * @fn      zclSampleLight_dmmPausePolicyCb
 *
 * @brief   DMM Policy callback to pause the stack
 */
static void zclSampleLight_dmmPausePolicyCb(uint16_t pause)
{
    zstack_pauseResumeDeviceReq_t zstack_pauseResumeDeviceReq;
    zstack_pauseResumeDeviceReq.pause = pause;
    Zstackapi_pauseResumeDeviceReq(appServiceTaskId, &zstack_pauseResumeDeviceReq);
}
#endif

#if defined (ENABLE_GREENPOWER_COMBO_BASIC)

void zclSampleLight_setGPSinkCommissioningMode(const int32_t _itemEntry)
{
  bool zclSampleLight_SetSinkCommissioningMode = 0;

  //Toggle current commissioning flag value and set that value again
  zclSampleLight_SetSinkCommissioningMode = !gp_GetSinkCommissioningMode();
  gp_SetSinkCommissioningMode(zclSampleLight_SetSinkCommissioningMode);

  UI_UpdateGpStatusLine();

}
#endif
#endif

#if defined (ENABLE_GREENPOWER_COMBO_BASIC)
/*********************************************************************
 * @fn      zclSampleLight_GPSink_Identify
 *
 * @brief   Callback to process Identify command from a GPD
 *
 * @param   zclGpNotification
 *
 * @return  none
 */
static void zclSampleLight_GPSink_Identify(zclGpNotification_t *zclGpNotification)
{
  afAddrType_t  dstAddr;

  dstAddr.endPoint = SAMPLELIGHT_ENDPOINT;
  dstAddr.panId = 0;
  dstAddr.addrMode = afAddr16Bit;
  dstAddr.addr.shortAddr = _NIB.nwkDevAddress;

  //Identify is a payloadless command which triggers a 60 seconds identify in the device (doc 14-0563-16 GP spec Table 49)
  zclGeneral_SendIdentify(SAMPLELIGHT_ENDPOINT,&dstAddr,60,TRUE, 1);
}

/*********************************************************************
 * @fn      zclSampleLight_GPSink_Off
 *
 * @brief   Callback to process Off command from a GPD
 *
 * @param   zclGpNotification
 *
 * @return  none
 */
static void zclSampleLight_GPSink_Off(zclGpNotification_t *zclGpNotification)
{
  SAEMS_OnOffCB(COMMAND_ON_OFF_OFF);
}

/*********************************************************************
 * @fn      zclSampleLight_GPSink_On
 *
 * @brief   Callback to process On command from a GPD
 *
 * @param   zclGpNotification
 *
 * @return  none
 */
static void zclSampleLight_GPSink_On(zclGpNotification_t *zclGpNotification)
{
  SAEMS_OnOffCB(COMMAND_ON_OFF_ON);
}

/*********************************************************************
 * @fn      zclSampleLight_GPSink_Toggle
 *
 * @brief   Callback to process Toggle command from a GPD
 *
 * @param   zclGpNotification
 *
 * @return  none
 */
static void zclSampleLight_GPSink_Toggle(zclGpNotification_t *zclGpNotification)
{
  SAEMS_OnOffCB(COMMAND_ON_OFF_TOGGLE);
}
#endif









