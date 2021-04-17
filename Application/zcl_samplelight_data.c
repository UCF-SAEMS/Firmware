/**************************************************************************************************
  Filename:       zcl_samplelight_data.c
  Revised:        $Date: 2014-05-12 13:14:02 -0700 (Mon, 12 May 2014) $
  Revision:       $Revision: 38502 $


  Description:    Zigbee Cluster Library - sample device application.


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
 * INCLUDES
 */
#include <stdio.h>
#include <stdlib.h>
#include "zcomdef.h"
#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"
#include "zcl_poll_control.h"
#include "zcl_electrical_measurement.h"
#include "zcl_meter_identification.h"
#include "zcl_appliance_identification.h"
#include "zcl_appliance_events_alerts.h"
#include "zcl_power_profile.h"
#include "zcl_appliance_control.h"
#include "zcl_appliance_statistics.h"
#include "zcl_hvac.h"
#include "zcl_ms.h"
#include "zcl_lighting.h"

#include "zcl_samplelight.h"
#ifdef BDB_REPORTING
#include "zstackapi.h"
#endif
/*********************************************************************
 * CONSTANTS
 */

#define SAMPLELIGHT_DEVICE_VERSION     1
#define SAMPLELIGHT_FLAGS              0

#define SAMPLELIGHT_HWVERSION          1
#define SAMPLELIGHT_ZCLVERSION         BASIC_ZCL_VERSION

#define DEFAULT_IDENTIFY_TIME 0
#define DEFAULT_ON_OFF_TRANSITION_TIME 20
#define DEFAULT_ON_LEVEL ATTR_LEVEL_ON_LEVEL_NO_EFFECT
#define DEFAULT_ON_TRANSITION_TIME 20
#define DEFAULT_OFF_TRANSITION_TIME 20
#define DEFAULT_MOVE_RATE 0 // as fast as possible

#define DEFAULT_ON_OFF_STATE LIGHT_ON
#define DEFAULT_LEVEL 40

// Used to update sensor structs
#define ZCL_DATA_UPDATE

// >>>> This is where the the CONSTANTS for the required/optional values for the sensors will be added <<<<<
//  Temperature Sensor Cluster
#define SAEMS_TEMPERATURESENSOR_MIN_MEASURED_VALUE  0000
//  Humididty Sensor Cluster
#define SAEMS_HUMIDITYSENSOR_MIN_MEASURED_VALUE 0000
//  Pressure Sensor Cluster
#define SAEMS_PRESSURESENSOR_MIN_MEASURED_VALUE   0000
// Motion Cluster
#define SAEMS_MOTIONSENSOR_OCCUPANCY 0000
// Carbon Monoxide Cluster
#define SAEMS_CARBONMONOXIDE_MIN_MEASURED_VALUE 0000
// Carbon Dioxide Cluster
#define SAEMS_CARBONDIOXIDE_MIN_MEASURED_VALUE 0000
// Smoke
#define SAEMS_SMOKE_MIN_MEASURED_VALUE 0000
// VOC
#define SAEMS_VOC_MIN_MEASURED_VALUE 0000
// Particulates
#define SAEMS_PARTICULATES_MIN_MEASURED_VALUE 0000
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// Color Control Cluster
#define SAEMS_COLORCONTROL_CURRENT_HUE 100
#define SAEMS_COLORCONTROL_CURRENT_SATURATION 1
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// Thresholds to update the sensor data struct
#define TEMPERATURE_UPDATE_THRESHOLD 0001
#define HUMIDITY_UPDATE_THRESHOLD 0001
#define PRESSURE_UPDATE_THRESHOLD 0001
#define OCCUPANCY_UPDATE_THRESHOLD 1
#define CARBONMONOXIDE_UPDATE_THRESHOLD 0001
#define CARBONDIOXIDE_UPDATE_THRESHOLD 0001
#define SMOKE_UPDATE_THRESHOLD 0001
#define VOC_UPDATE_THRESHOLD 0001
#define PARTICULATES_UPDATE_THRESHOLD 0001


/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */


/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern uint8_t  appServiceTaskId;

/*********************************************************************
 * GLOBAL VARIABLES
 */

//global attributes
const uint16_t zclSampleLight_basic_clusterRevision = 0x0002;
const uint16_t zclSampleLight_identify_clusterRevision = 0x0001;
const uint16_t zclSampleLight_groups_clusterRevision = 0x0001;
const uint16_t zclSampleLight_scenes_clusterRevision = 0x0001;
const uint16_t zclSampleLight_onoff_clusterRevision = 0x0001;
const uint16_t zclSampleLight_level_clusterRevision = 0x0001;


// Basic Cluster
const uint8_t zclSampleLight_HWRevision = SAMPLELIGHT_HWVERSION;
const uint8_t zclSampleLight_ZCLVersion = SAMPLELIGHT_ZCLVERSION;
const uint8_t zclSampleLight_ManufacturerName[] = { 16, 'T','e','x','a','s','I','n','s','t','r','u','m','e','n','t','s' };
const uint8_t zclSampleLight_PowerSource = POWER_SOURCE_MAINS_1_PHASE;
uint8_t zclSampleLight_PhysicalEnvironment = PHY_UNSPECIFIED_ENV;

// Identify Cluster
uint16_t zclSampleLight_IdentifyTime;

// Groups Cluster
uint8_t zclSampleLight_GroupsNameSupport = 0;

// On/Off Cluster
static uint8_t  zclSampleLight_OnOff;

// Level Control Cluster
#ifdef ZCL_LEVEL_CTRL
static uint8_t  zclSampleLight_LevelCurrentLevel;
uint16_t zclSampleLight_LevelRemainingTime;
uint16_t zclSampleLight_LevelOnOffTransitionTime;
uint8_t  zclSampleLight_LevelOnLevel;
uint16_t zclSampleLight_LevelOnTransitionTime;
uint16_t zclSampleLight_LevelOffTransitionTime;
uint8_t  zclSampleLight_LevelDefaultMoveRate;
#endif

#ifdef ZCL_LIGHTING
// >>>>>>>>>>>>>>>>>> DEFAULT VALUES AND VARIABLES FOR COLOR CONTROL CLUSTER <<<<<<<<<<<<<<<<<<<<<<
// Color Control Cluster
uint8_t SAEMS_ColorControl_CurrentHue = SAEMS_COLORCONTROL_CURRENT_HUE;
uint8_t SAEMS_ColorControl_CurrentSaturation = SAEMS_COLORCONTROL_CURRENT_SATURATION;
#endif  // ZCL_LIGHTING

#ifdef ZCL_MS
// >>>>>>>>>>>>>>>> DEFAULT VALUES AND VARIABLES FOR EACH SENSOR CLUSTER (to be used with commands/attributes) <<<<<<<<<<<<<<<<<<<<
// Temperature Sensor Variable
int16_t zclSAEMS_Temperature_MeasuredValue = SAEMS_TEMPERATURESENSOR_MIN_MEASURED_VALUE;
// Humididty Sensor Variable
int16_t zclSAEMS_Humidity_MesauredValue = SAEMS_HUMIDITYSENSOR_MIN_MEASURED_VALUE;
// Pressure Sensor Variable
int16_t zclSAEMS_Pressure_MeasuredValue = SAEMS_PRESSURESENSOR_MIN_MEASURED_VALUE;
// Motion Sensor Variable
int16_t zclSAEMS_Motion_Occupancy = SAEMS_MOTIONSENSOR_OCCUPANCY;
// Carbon Monoxide Sensor Variable
int16_t zclSAEMS_CarbonMonoxide_MeasuredValue = SAEMS_CARBONMONOXIDE_MIN_MEASURED_VALUE;
// Carbon Dioxide Sensor Variable
int16_t zclSAEMS_CarbonDioxide_MeasuredValue = SAEMS_CARBONDIOXIDE_MIN_MEASURED_VALUE;
// Smoke Sensor Variable
int16_t zclSAEMS_Smoke_MeasuredValue = SAEMS_SMOKE_MIN_MEASURED_VALUE;
// VOC Sensor Variable
int16_t zclSAEMS_VOC_MeasuredValue = SAEMS_VOC_MIN_MEASURED_VALUE;
// Particulates Sensor Variable
int16_t zclSAEMS_Particulates_MeasuredValue = SAEMS_PARTICULATES_MIN_MEASURED_VALUE;
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#endif // ZCL_MS

// Initialize data structs
SAEMS_SensorData sensorDataCurrent = {0};
SAEMS_SensorData sensorDataNew = {0};

uint8_t  zclSampleLight_ScenesCurrentScene = 0;
uint16_t zclSampleLight_ScenesCurrentGroup = 0;
uint8_t  zclSampleLight_ScenesValid = 0;
uint8_t  zclSampleLight_ScenesNameSupport = 0;

int Particulates2SeqNum = 0;
int Particulates4SeqNum = 0;

// >>>>>>>>>>>>>>>>>>>>>>> IF MORE COMMANDS ARE NEEDED, SHOULD BE ADDED HERE <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

#if ZCL_DISCOVER
CONST zclCommandRec_t zclSampleLight_Cmds[] =
{
// >>>>>>>>>>>>>>> ON/OFF CLUSTER COMMANDS <<<<<<<<<<<<<<<
  {
    ZCL_CLUSTER_ID_GENERAL_BASIC,
    COMMAND_BASIC_RESET_TO_FACTORY_DEFAULTS,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_GENERAL_ON_OFF,
    COMMAND_ON_OFF_OFF,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_GENERAL_ON_OFF,
    COMMAND_ON_OFF_ON,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_GENERAL_ON_OFF,
    COMMAND_ON_OFF_TOGGLE,
    CMD_DIR_SERVER_RECEIVED
  },
// >>>>>>>>>>>>>>> LEVEL CONTROL CLUSTER COMMANDS (SLIDER) <<<<<<<<<<<<<<<
#ifdef ZCL_LEVEL_CONTROL
  ,{
    ZCL_CLUSTER_ID_GENERAL_LEVEL_CONTROL,
    COMMAND_LEVEL_MOVE_TO_LEVEL,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_GENERAL_LEVEL_CONTROL,
    COMMAND_LEVEL_MOVE,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_GENERAL_LEVEL_CONTROL,
    COMMAND_LEVEL_STEP,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_GENERAL_LEVEL_CONTROL,
    COMMAND_LEVEL_STOP,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_GENERAL_LEVEL_CONTROL,
    COMMAND_LEVEL_MOVE_TO_LEVEL_WITH_ON_OFF,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_GENERAL_LEVEL_CONTROL,
    COMMAND_LEVEL_MOVE_WITH_ON_OFF,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_GENERAL_LEVEL_CONTROL,
    COMMAND_LEVEL_STEP_WITH_ON_OFF,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_GENERAL_LEVEL_CONTROL,
    COMMAND_LEVEL_STOP_WITH_ON_OFF,
    CMD_DIR_SERVER_RECEIVED
  },
#endif // ZCL_LEVEL_CONTROL
#ifdef ZCL_LIGHTING
// >>>>>>>>>>>>>>> COLOR CONTROL CLUSTER COMMANDS <<<<<<<<<<<<<<<
  {
    ZCL_CLUSTER_ID_LIGHTING_COLOR_CONTROL,
    COMMAND_COLOR_CONTROL_MOVE_TO_HUE,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_LIGHTING_COLOR_CONTROL,
    COMMAND_COLOR_CONTROL_MOVE_HUE,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_LIGHTING_COLOR_CONTROL,
    COMMAND_COLOR_CONTROL_STEP_HUE,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_LIGHTING_COLOR_CONTROL,
    COMMAND_COLOR_CONTROL_MOVE_TO_SATURATION,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_LIGHTING_COLOR_CONTROL,
    COMMAND_COLOR_CONTROL_MOVE_SATURATION,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_LIGHTING_COLOR_CONTROL,
    COMMAND_COLOR_CONTROL_STEP_SATURATION,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_LIGHTING_COLOR_CONTROL,
    COMMAND_COLOR_CONTROL_MOVE_TO_HUE_AND_SATURATION,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_LIGHTING_COLOR_CONTROL,
    COMMAND_COLOR_CONTROL_STOP_MOVE_STEP,
    CMD_DIR_SERVER_RECEIVED
  }
#endif //ZCL_LIGHTING
// ----------------------------------------------------------------
};

CONST uint8_t zclCmdsArraySize = ( sizeof(zclSampleLight_Cmds) / sizeof(zclSampleLight_Cmds[0]) );
#endif // ZCL_DISCOVER

/*********************************************************************
 * ATTRIBUTE DEFINITIONS - Uses REAL cluster IDs
 */

// NOTE: The attributes listed in the AttrRec must be in ascending order
// per cluster to allow right function of the Foundation discovery commands

CONST zclAttrRec_t zclSampleLight_Attrs[] =
{
  // *** General Basic Cluster Attributes ***
  {
    ZCL_CLUSTER_ID_GENERAL_BASIC,
    { // Attribute record
      ATTRID_BASIC_ZCL_VERSION,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&zclSampleLight_ZCLVersion
    }
  },
  {
    ZCL_CLUSTER_ID_GENERAL_BASIC,             // Cluster IDs - defined in the foundation (ie. zcl.h)
    {  // Attribute record
      ATTRID_BASIC_HW_VERSION,            // Attribute ID - Found in Cluster Library header (ie. zcl_general.h)
      ZCL_DATATYPE_UINT8,                 // Data Type - found in zcl.h
      ACCESS_CONTROL_READ,                // Variable access control - found in zcl.h
      (void *)&zclSampleLight_HWRevision  // Pointer to attribute variable
    }
  },
  {
    ZCL_CLUSTER_ID_GENERAL_BASIC,
    { // Attribute record
      ATTRID_BASIC_MANUFACTURER_NAME,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zclSampleLight_ManufacturerName
    }
  },
  {
    ZCL_CLUSTER_ID_GENERAL_BASIC,
    { // Attribute record
      ATTRID_BASIC_POWER_SOURCE,
      ZCL_DATATYPE_ENUM8,
      ACCESS_CONTROL_READ,
      (void *)&zclSampleLight_PowerSource
    }
  },
  {
    ZCL_CLUSTER_ID_GENERAL_BASIC,
    { // Attribute record
      ATTRID_BASIC_PHYSICAL_ENVIRONMENT,
      ZCL_DATATYPE_ENUM8,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclSampleLight_PhysicalEnvironment
    }
  },
  {
    ZCL_CLUSTER_ID_GENERAL_BASIC,
    {  // Attribute record
      ATTRID_CLUSTER_REVISION,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      (void *)&zclSampleLight_basic_clusterRevision
    }
  },
#ifdef ZCL_IDENTIFY
  // *** Identify Cluster Attribute ***
  {
    ZCL_CLUSTER_ID_GENERAL_IDENTIFY,
    { // Attribute record
      ATTRID_IDENTIFY_IDENTIFY_TIME,
      ZCL_DATATYPE_UINT16,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclSampleLight_IdentifyTime
    }
  },
  {
    ZCL_CLUSTER_ID_GENERAL_IDENTIFY,
    {  // Attribute record
      ATTRID_CLUSTER_REVISION,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      (void *)&zclSampleLight_identify_clusterRevision
    }
  },
#endif

  // *** On/Off Cluster Attributes ***
  {
    ZCL_CLUSTER_ID_GENERAL_ON_OFF,
    { // Attribute record
      ATTRID_ON_OFF_ON_OFF,
      ZCL_DATATYPE_BOOLEAN,
      ACCESS_CONTROL_READ | ACCESS_REPORTABLE,
      (void*)&zclSampleLight_OnOff
    }
  },
  {
    ZCL_CLUSTER_ID_GENERAL_ON_OFF,
    {  // Attribute record
      ATTRID_CLUSTER_REVISION,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      (void *)&zclSampleLight_onoff_clusterRevision
    }
  },

#ifdef ZCL_LEVEL_CTRL
  {
    ZCL_CLUSTER_ID_GENERAL_LEVEL_CONTROL,
    { // Attribute record
      ATTRID_LEVEL_CURRENT_LEVEL,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ | ACCESS_REPORTABLE,
      (void*)&zclSampleLight_LevelCurrentLevel
    }
  },
  {
    ZCL_CLUSTER_ID_GENERAL_LEVEL_CONTROL,
    { // Attribute record
      ATTRID_LEVEL_REMAINING_TIME,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      (void *)&zclSampleLight_LevelRemainingTime
    }
  },
  {
    ZCL_CLUSTER_ID_GENERAL_LEVEL_CONTROL,
    { // Attribute record
      ATTRID_LEVEL_ON_OFF_TRANSITION_TIME,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE,
      (void *)&zclSampleLight_LevelOnOffTransitionTime
    }
  },
  {
    ZCL_CLUSTER_ID_GENERAL_LEVEL_CONTROL,
    { // Attribute record
      ATTRID_LEVEL_ON_LEVEL,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE,
      (void *)&zclSampleLight_LevelOnLevel
    }
  },
  {
    ZCL_CLUSTER_ID_GENERAL_LEVEL_CONTROL,
    { // Attribute record
      ATTRID_LEVEL_ON_TRANSITION_TIME,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE,
      (void *)&zclSampleLight_LevelOnTransitionTime
    }
  },
  {
    ZCL_CLUSTER_ID_GENERAL_LEVEL_CONTROL,
    { // Attribute record
      ATTRID_LEVEL_OFF_TRANSITION_TIME,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE,
      (void *)&zclSampleLight_LevelOffTransitionTime
    }
  },
  {
    ZCL_CLUSTER_ID_GENERAL_LEVEL_CONTROL,
    { // Attribute record
      ATTRID_LEVEL_DEFAULT_MOVE_RATE,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE,
      (void *)&sensorDataCurrent.pollingRate
    }
  },
  {
    ZCL_CLUSTER_ID_GENERAL_LEVEL_CONTROL,
    {  // Attribute record
      ATTRID_CLUSTER_REVISION,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      (void *)&zclSampleLight_level_clusterRevision
    }
  },
#endif
#ifdef ZCL_GROUPS
  {
    ZCL_CLUSTER_ID_GENERAL_GROUPS,
    {
      ATTRID_GROUPS_NAME_SUPPORT,
      ZCL_DATATYPE_BITMAP8,
      ACCESS_CONTROL_READ,
      (void*)&zclSampleLight_GroupsNameSupport
    }
  },

  {
    ZCL_CLUSTER_ID_GENERAL_GROUPS,
    {  // Attribute record
      ATTRID_CLUSTER_REVISION,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      (void *)&zclSampleLight_groups_clusterRevision
    }
  },
#endif
  // *** Scene Cluster Attributes ***
  {
    ZCL_CLUSTER_ID_GENERAL_SCENES,
    { // Attribute record
      ATTRID_SCENES_SCENE_COUNT,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_GENERAL_SCENES,
    { // Attribute record
      ATTRID_SCENES_CURRENT_SCENE,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&zclSampleLight_ScenesCurrentScene
    }
  },
  {
    ZCL_CLUSTER_ID_GENERAL_SCENES,
    { // Attribute record
      ATTRID_SCENES_CURRENT_GROUP,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      (void *)&zclSampleLight_ScenesCurrentGroup
    }
  },
  {
    ZCL_CLUSTER_ID_GENERAL_SCENES,
    { // Attribute record
      ATTRID_SCENES_SCENE_VALID,
      ZCL_DATATYPE_BOOLEAN,
      ACCESS_CONTROL_READ,
      (void *)&zclSampleLight_ScenesValid
    }
  },
  {
    ZCL_CLUSTER_ID_GENERAL_SCENES,
    { // Attribute record
      ATTRID_SCENES_NAME_SUPPORT,
      ZCL_DATATYPE_BITMAP8,
      ACCESS_CONTROL_READ,
      (void *)&zclSampleLight_ScenesNameSupport
    }
  },
  {
    ZCL_CLUSTER_ID_GENERAL_SCENES,
    {  // Attribute record
      ATTRID_CLUSTER_REVISION,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      (void *)&zclSampleLight_scenes_clusterRevision
    }
  },
//--------------------------------------------------------------
// ============================================================================================================
// =========================================== SAEMS SENSOR CLUSTERS ==========================================
// ============================================================================================================
#ifdef ZCL_MS
  {
    ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT,
    { // >>>> TEMPERATURE Attribute <<<<
      ATTRID_TEMPERATURE_MEASUREMENT_MEASURED_VALUE,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ | ACCESS_REPORTABLE,
      (void *)&sensorDataCurrent.temperature
    }
  },
// ------------------------------------------------------------
  {
    ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT,
    { // >>>> PARTICULATES Attribute <<<<
      ATTRID_TEMPERATURE_MEASUREMENT_MIN_MEASURED_VALUE,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ | ACCESS_REPORTABLE,
      (void *)&sensorDataCurrent.particulates
    }
  },
// ------------------------------------------------------------
  {
    ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT,
    { // >>>> OCCUPANCY Attribute <<<<
      ATTRID_TEMPERATURE_MEASUREMENT_TOLERANCE,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ | ACCESS_REPORTABLE,
      (void *)&sensorDataCurrent.occupancy
    }
  },
// ------------------------------------------------------------
  {
    ZCL_CLUSTER_ID_MS_RELATIVE_HUMIDITY,
    { // >>>> HUMIDITY Attribute <<<<
      ATTRID_RELATIVITY_HUMIDITY_MEASURED_VALUE,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ | ACCESS_REPORTABLE,
      (void *)&sensorDataCurrent.humidity
    }
  },
// ------------------------------------------------------------
  {
    ZCL_CLUSTER_ID_MS_RELATIVE_HUMIDITY,
    { // >>>> CARBON MONOXIDE Attribute <<<<
      ATTRID_RELATIVITY_HUMIDITY_TOLERANCE,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ | ACCESS_REPORTABLE,
      (void *)&sensorDataCurrent.carbonmonoxide
    }
  },
// ------------------------------------------------------------
  {
    ZCL_CLUSTER_ID_MS_PRESSURE_MEASUREMENT,
    { // >>>> PRESSURE Attribute <<<<
      ATTRID_PRESSURE_MEASUREMENT_MEASURED_VALUE,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ | ACCESS_REPORTABLE,
      (void *)&sensorDataCurrent.pressure
    }
  },
// ------------------------------------------------------------
  {
    ZCL_CLUSTER_ID_MS_PRESSURE_MEASUREMENT,
    { // >>>> CARBON DIOXIDE Attribute <<<<
      ATTRID_PRESSURE_MEASUREMENT_TOLERANCE,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ | ACCESS_REPORTABLE,
      (void *)&sensorDataCurrent.carbondioxide
    }
  },
// ------------------------------------------------------------
  {
    ZCL_CLUSTER_ID_MS_PRESSURE_MEASUREMENT,
    { // >>>> SMOKE Attribute <<<<
      ATTRID_PRESSURE_MEASUREMENT_SCALED_VALUE,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ | ACCESS_REPORTABLE,
      (void *)&sensorDataCurrent.smoke
    }
  },
// ------------------------------------------------------------
  {
    ZCL_CLUSTER_ID_MS_PRESSURE_MEASUREMENT,
    { // >>>> VOC Attribute <<<<
      ATTRID_PRESSURE_MEASUREMENT_SCALED_TOLERANCE,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ | ACCESS_REPORTABLE,
      (void *)&sensorDataCurrent.voc
    }
  },
// ------------------------------------------------------------
  {
    SAEMS_PARTICULATES_CLUSTER_ID,
    { // >>>> PARTICULATES 1.0 MASS Attribute <<< 
      SAEMS_ATTRID_PARTICULATES_MASS_1,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ | ACCESS_REPORTABLE,
      (void *)&sensorDataCurrent.pm1mass
    }
  },
// ------------------------------------------------------------
  {
    SAEMS_PARTICULATES_CLUSTER_ID,
    { // >>>> PARTICULATES 2.5 MASS Attribute <<< 
      SAEMS_ATTRID_PARTICULATES_MASS_2,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ | ACCESS_REPORTABLE,
      (void *)&sensorDataCurrent.pm2mass
    }
  },
// ------------------------------------------------------------
  {
    SAEMS_PARTICULATES_CLUSTER_ID,
    { // >>>> PARTICULATES 4.0 MASS Attribute <<< 
      SAEMS_ATTRID_PARTICULATES_MASS_4,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ | ACCESS_REPORTABLE,
      (void *)&sensorDataCurrent.pm4mass
    }
  },
// ------------------------------------------------------------
  {
    SAEMS_PARTICULATES_CLUSTER_ID,
    { // >>>> PARTICULATES 10.0 MASS Attribute <<< 
      SAEMS_ATTRID_PARTICULATES_MASS_10,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ | ACCESS_REPORTABLE,
      (void *)&sensorDataCurrent.pm10mass
    }
  },
// ------------------------------------------------------------
  {
    SAEMS_PARTICULATES_CLUSTER_ID,
    { // >>>> PARTICULATES 1.0 NUMBER Attribute <<< 
      SAEMS_ATTRID_PARTICULATES_NUMBER_1,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ | ACCESS_REPORTABLE,
      (void *)&sensorDataCurrent.pm1number
    }
  },
// ------------------------------------------------------------
  {
    SAEMS_PARTICULATES_CLUSTER_ID,
    { // >>>> PARTICULATES 2.5 NUMBER Attribute <<< 
      SAEMS_ATTRID_PARTICULATES_NUMBER_2,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ | ACCESS_REPORTABLE,
      (void *)&sensorDataCurrent.pm2number
    }
  },
// ------------------------------------------------------------
  {
    SAEMS_PARTICULATES_CLUSTER_ID,
    { // >>>> PARTICULATES 4.0 NUMBER Attribute <<< 
      SAEMS_ATTRID_PARTICULATES_NUMBER_4,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ | ACCESS_REPORTABLE,
      (void *)&sensorDataCurrent.pm4number
    }
  },
// ------------------------------------------------------------
  {
    SAEMS_PARTICULATES_CLUSTER_ID,
    { // >>>> PARTICULATES 10.0 NUMBER Attribute <<< 
      SAEMS_ATTRID_PARTICULATES_NUMBER_10,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ | ACCESS_REPORTABLE,
      (void *)&sensorDataCurrent.pm10number
    }
  },
// ------------------------------------------------------------
  {
    SAEMS_PARTICULATES_CLUSTER_ID,
    { // >>>> PARTICULATES TYPICAL SIZE Attribute <<< 
      SAEMS_ATTRID_PARTICULATES_TYPICAL,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ | ACCESS_REPORTABLE,
      (void *)&sensorDataCurrent.typicalparticlesize
    }
  },
// ------------------------------------------------------------

#endif // ZCL_MS
// ===================================================================================================================
// =========================================== SAEMS COLOR CONTROL CLUSTERS ==========================================
// ===================================================================================================================
#ifdef ZCL_LIGHTING
  {
  ZCL_CLUSTER_ID_LIGHTING_COLOR_CONTROL,
    { // CURRENT HUE Attribute
      ATTRID_COLOR_CONTROL_CURRENT_HUE,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ | ACCESS_REPORTABLE,
      (void *)&SAEMS_ColorControl_CurrentHue
    }
  },
  {
    ZCL_CLUSTER_ID_LIGHTING_COLOR_CONTROL,
    { // CURRENT SATURATION Attribute
      ATTRID_COLOR_CONTROL_CURRENT_SATURATION,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ | ACCESS_REPORTABLE,
      (void *)&SAEMS_ColorControl_CurrentSaturation
    }
  }
#endif  // ZCL_LIGHTING
// --------------------------------------------------------------------------------------------------------------------
};

uint8_t CONST zclSampleLight_NumAttributes = ( sizeof(zclSampleLight_Attrs) / sizeof(zclSampleLight_Attrs[0]) );

/*********************************************************************
 * SIMPLE DESCRIPTOR
 */
// This is the Cluster ID List and should be filled with Application
// specific cluster IDs.
const cId_t zclSampleLight_InClusterList[] =
{
  ZCL_CLUSTER_ID_GENERAL_BASIC,
  ZCL_CLUSTER_ID_GENERAL_IDENTIFY,
  ZCL_CLUSTER_ID_GENERAL_GROUPS,
  ZCL_CLUSTER_ID_GENERAL_SCENES,
  ZCL_CLUSTER_ID_GENERAL_ON_OFF
#ifdef ZCL_LEVEL_CTRL
  , ZCL_CLUSTER_ID_GENERAL_LEVEL_CONTROL
#endif
,
// Sensor and App - added Cluster IDs
  ZCL_CLUSTER_ID_LIGHTING_COLOR_CONTROL,
  ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT,
  ZCL_CLUSTER_ID_MS_RELATIVE_HUMIDITY,
  ZCL_CLUSTER_ID_MS_PRESSURE_MEASUREMENT,
  SAEMS_PARTICULATES_CLUSTER_ID
};

#define ZCLSAMPLELIGHT_MAX_INCLUSTERS   (sizeof(zclSampleLight_InClusterList) / sizeof(zclSampleLight_InClusterList[0]))

SimpleDescriptionFormat_t zclSampleLight_SimpleDesc =
{
  SAMPLELIGHT_ENDPOINT,                  //  int Endpoint;
  ZCL_HA_PROFILE_ID,                     //  uint16_t AppProfId;
  ZCL_DEVICEID_SIMPLE_SENSOR,            //  uint16_t AppDeviceId;
  SAMPLELIGHT_DEVICE_VERSION,            //  int   AppDevVer:4;
  SAMPLELIGHT_FLAGS,                     //  int   AppFlags:4;
  ZCLSAMPLELIGHT_MAX_INCLUSTERS,         //  byte  AppNumInClusters;
  (cId_t *)zclSampleLight_InClusterList, //  byte *pAppInClusterList;
  0,        //  byte  AppNumInClusters;
  NULL //  byte *pAppInClusterList;
};

// Added to include Touchlink Target functionality
#if defined ( BDB_TL_INITIATOR ) || defined ( BDB_TL_TARGET )
bdbTLDeviceInfo_t tlSampleLight_DeviceInfo =
{
  ZCL_HA_PROFILE_ID,                        //uint16_t profileID;
#ifdef ZCL_LEVEL_CTRL
      ZCL_DEVICEID_DIMMABLE_LIGHT,        //  uint16_t AppDeviceId;
#else
      ZCL_DEVICEID_ON_OFF_LIGHT,          //  uint16_t AppDeviceId;
#endif
  SAMPLELIGHT_ENDPOINT,                  //uint8_t endpoint;
  SAMPLELIGHT_DEVICE_VERSION,                    //uint8_t version;
  SAMPLELIGHT_NUM_GRPS                   //uint8_t grpIdCnt;
};
#endif
/*********************************************************************
 * GLOBAL FUNCTIONS
 */

/*********************************************************************
 * @fn      zclSampleLight_updateOnOffAttribute
 *
 * @brief   Function that updates the OnOff attribute, this will also update scene attribute.
 *
 * @param   OnOff state
 *
 * @return  none
 */
void zclSampleLight_updateOnOffAttribute(uint8_t OnOff)
{
    if(zclSampleLight_OnOff != OnOff)
    {
        zclSampleLight_OnOff = OnOff;
        zclSampleLight_ScenesValid = FALSE;

#ifdef BDB_REPORTING
        zstack_bdbRepChangedAttrValueReq_t Req;

        Req.attrID = ATTRID_ON_OFF_ON_OFF;
        Req.cluster = ZCL_CLUSTER_ID_GENERAL_ON_OFF;
        Req.endpoint = SAMPLELIGHT_ENDPOINT;

      Zstackapi_bdbRepChangedAttrValueReq(appServiceTaskId,&Req);
#endif
#ifdef ZCL_MEASUREMENT_TESTING
#ifdef BDB_REPORTING

        Req.attrID = ATTRID_PRESSURE_MEASUREMENT_SCALED_VALUE;
        Req.cluster = ZCL_CLUSTER_ID_MS_PRESSURE_MEASUREMENT;
        Req.endpoint = SAMPLELIGHT_ENDPOINT;

      Zstackapi_bdbRepChangedAttrValueReq(appServiceTaskId,&Req);
#endif
#endif
    }
}

/*********************************************************************
 * @fn      zclSampleLight_getOnOffAttribute
 *
 * @brief   Function to get the state of the OnOff attribute.
 *
 * @param   none
 *
 * @return  OnOff attribute state (True if on, False if Off)
 */
uint8_t zclSampleLight_getOnOffAttribute(void)
{
    return zclSampleLight_OnOff;
}

#ifdef ZCL_LEVEL_CTRL
/*********************************************************************
 * @fn      zclSampleLight_updateCurrentLevelAttribute
 *
 * @brief   Function that updates the OnOff attribute, this will also update scene attribute.
 *
 * @param   Current Level state
 *
 * @return  none
 */
void zclSampleLight_updateCurrentLevelAttribute(uint8_t CurrentLevel)
{
    if(zclSampleLight_LevelCurrentLevel != CurrentLevel)
    {
        zclSampleLight_LevelCurrentLevel = CurrentLevel;
        zclSampleLight_ScenesValid = FALSE;

#ifdef BDB_REPORTING
        zstack_bdbRepChangedAttrValueReq_t Req;

        Req.attrID = ATTRID_LEVEL_CURRENT_LEVEL;
        Req.cluster = ZCL_CLUSTER_ID_GENERAL_LEVEL_CONTROL;
        Req.endpoint = SAMPLELIGHT_ENDPOINT;

      Zstackapi_bdbRepChangedAttrValueReq(appServiceTaskId,&Req);
#endif
    }
}

/*********************************************************************
 * @fn      zclSampleLight_getCurrentLevelAttribute
 *
 * @brief   Function to get the state of the OnOff attribute.
 *
 * @param   none
 *
 * @return  Current Level attribute state
 */
uint8_t zclSampleLight_getCurrentLevelAttribute(void)
{
    return zclSampleLight_LevelCurrentLevel;
}
#endif

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      zclSampleLight_ResetAttributesToDefaultValues
 *
 * @brief   Reset all writable attributes to their default values.
 *
 * @param   none
 *
 * @return  none
 */
void zclSampleLight_ResetAttributesToDefaultValues(void)
{
  zclSampleLight_PhysicalEnvironment = PHY_UNSPECIFIED_ENV;

#ifdef ZCL_IDENTIFY
  zclSampleLight_IdentifyTime = DEFAULT_IDENTIFY_TIME;
#endif
#ifdef ZCL_LEVEL_CTRL
  zclSampleLight_LevelRemainingTime = 0;
  zclSampleLight_LevelOnOffTransitionTime = DEFAULT_ON_OFF_TRANSITION_TIME;
  zclSampleLight_LevelOnLevel = DEFAULT_ON_LEVEL;
  zclSampleLight_LevelOnTransitionTime = DEFAULT_ON_TRANSITION_TIME;
  zclSampleLight_LevelOffTransitionTime = DEFAULT_OFF_TRANSITION_TIME;
  zclSampleLight_LevelDefaultMoveRate = DEFAULT_MOVE_RATE;

  zclSampleLight_updateCurrentLevelAttribute(DEFAULT_LEVEL);
#endif

  zclSampleLight_updateOnOffAttribute(DEFAULT_ON_OFF_STATE);

  zclSampleLight_IdentifyTime = 0;
}
// ====================================================================================================================
// ====================================================================================================================

#ifdef ZCL_DATA_UPDATE
  void SAEMS_updateSensorData(void){

    zstack_bdbRepChangedAttrValueReq_t Req;
    
    Req.endpoint = SAMPLELIGHT_ENDPOINT;

    printf("***************************************\n");
    printf("Updating current sensor data to new sensor data and sending to hub\n");

    // Update temperature value
    if (abs(sensorDataCurrent.temperature - sensorDataNew.temperature) > TEMPERATURE_UPDATE_THRESHOLD) {
        sensorDataCurrent.temperature = sensorDataNew.temperature;
        printf("Temperature updated to %u\n", (unsigned int)sensorDataCurrent.temperature);

        Req.attrID = ATTRID_TEMPERATURE_MEASUREMENT_MEASURED_VALUE;
        Req.cluster = ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT;
        Zstackapi_bdbRepChangedAttrValueReq(appServiceTaskId,&Req);
    }

    // Update humidity value
    if (abs(sensorDataCurrent.humidity - sensorDataNew.humidity) > HUMIDITY_UPDATE_THRESHOLD) {
        sensorDataCurrent.humidity = sensorDataNew.humidity;
        printf("Humidity updated to %u\n", (unsigned int)sensorDataCurrent.humidity);

        Req.attrID = ATTRID_RELATIVITY_HUMIDITY_MEASURED_VALUE;
        Req.cluster = ZCL_CLUSTER_ID_MS_RELATIVE_HUMIDITY;
        Zstackapi_bdbRepChangedAttrValueReq(appServiceTaskId,&Req);
    }

    // Update pressure value
    if (abs(sensorDataCurrent.pressure - sensorDataNew.pressure) > PRESSURE_UPDATE_THRESHOLD) {
        sensorDataCurrent.pressure = sensorDataNew.pressure;
        printf("Pressure updated to %u\n", (unsigned int)sensorDataCurrent.pressure);

        Req.attrID = ATTRID_PRESSURE_MEASUREMENT_MEASURED_VALUE;
        Req.cluster = ZCL_CLUSTER_ID_MS_PRESSURE_MEASUREMENT;
        Zstackapi_bdbRepChangedAttrValueReq(appServiceTaskId,&Req);
    }

    // Update occupancy value
    if (abs(sensorDataCurrent.occupancy - sensorDataNew.occupancy) >= OCCUPANCY_UPDATE_THRESHOLD) {
        sensorDataCurrent.occupancy = sensorDataNew.occupancy;
        printf("Occupancy updated to %u\n", (unsigned int)sensorDataCurrent.occupancy);

        Req.attrID = ATTRID_TEMPERATURE_MEASUREMENT_TOLERANCE;
        Req.cluster = ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT;
        Zstackapi_bdbRepChangedAttrValueReq(appServiceTaskId,&Req);
    }

    // Update carbonmonoxide value
    if (abs(sensorDataCurrent.carbonmonoxide - sensorDataNew.carbonmonoxide) >= CARBONMONOXIDE_UPDATE_THRESHOLD) {
        sensorDataCurrent.carbonmonoxide = sensorDataNew.carbonmonoxide;
        printf("Carbonmonoxide updated to %u\n", (unsigned int)sensorDataCurrent.carbonmonoxide);

        Req.attrID = ATTRID_RELATIVITY_HUMIDITY_TOLERANCE;
        Req.cluster = ZCL_CLUSTER_ID_MS_RELATIVE_HUMIDITY;
        Zstackapi_bdbRepChangedAttrValueReq(appServiceTaskId,&Req);
    }

    // Update carbondioxide value
    if (abs(sensorDataCurrent.carbondioxide - sensorDataNew.carbondioxide) > CARBONDIOXIDE_UPDATE_THRESHOLD) {
        sensorDataCurrent.carbondioxide = sensorDataNew.carbondioxide;
        printf("Carbondioxide updated to %u\n", (unsigned int)sensorDataCurrent.carbondioxide);

        Req.attrID = ATTRID_PRESSURE_MEASUREMENT_TOLERANCE;
        Req.cluster = ZCL_CLUSTER_ID_MS_PRESSURE_MEASUREMENT;
        Zstackapi_bdbRepChangedAttrValueReq(appServiceTaskId,&Req);
    }

    // Update smoke value
    if (abs(sensorDataCurrent.smoke - sensorDataNew.smoke) > SMOKE_UPDATE_THRESHOLD) {
        sensorDataCurrent.smoke = sensorDataNew.smoke;
        printf("Smoke updated to %u\n", (unsigned int)sensorDataCurrent.smoke);

        Req.attrID = ATTRID_PRESSURE_MEASUREMENT_SCALED_VALUE;
        Req.cluster = ZCL_CLUSTER_ID_MS_PRESSURE_MEASUREMENT;
        Zstackapi_bdbRepChangedAttrValueReq(appServiceTaskId,&Req);
    }

    // Update voc value
    if (abs(sensorDataCurrent.voc - sensorDataNew.voc) > VOC_UPDATE_THRESHOLD) {
        sensorDataCurrent.voc = sensorDataNew.voc;
        printf("Voc updated to %u\n", (unsigned int)sensorDataCurrent.voc);

        Req.attrID = ATTRID_PRESSURE_MEASUREMENT_SCALED_TOLERANCE;
        Req.cluster = ZCL_CLUSTER_ID_MS_PRESSURE_MEASUREMENT;
        Zstackapi_bdbRepChangedAttrValueReq(appServiceTaskId,&Req);
    }

    // Update particulates 2.5 mass and number value
    if (abs(sensorDataCurrent.pm2mass - sensorDataNew.pm2mass) > PARTICULATES_UPDATE_THRESHOLD) {
        sensorDataCurrent.pm1mass = sensorDataNew.pm2mass;
        sensorDataCurrent.pm2mass = sensorDataNew.pm2mass;
        sensorDataCurrent.pm4mass = sensorDataNew.pm4mass;
        sensorDataCurrent.pm10mass = sensorDataNew.pm10mass;
        sensorDataCurrent.pm1number = sensorDataNew.pm1number;
        sensorDataCurrent.pm2number = sensorDataNew.pm2number;
        sensorDataCurrent.pm4number = sensorDataNew.pm4number;
        sensorDataCurrent.pm10number = sensorDataNew.pm10number;
        sensorDataCurrent.typicalparticlesize = sensorDataNew.typicalparticlesize;
        printf("Particulates_2.5 updated to %u\n", (unsigned int)sensorDataCurrent.pm2mass);

        Req.attrID = SAEMS_ATTRID_PARTICULATES_MASS_2;
        Req.cluster = SAEMS_PARTICULATES_CLUSTER_ID;
        Zstackapi_bdbRepChangedAttrValueReq(appServiceTaskId, &Req);
    }

        printf("\nCurrent Motion Polling Time: %u\n",(unsigned int)sensorDataCurrent.pollingRate);


    printf("***************************************\n");
  }
#endif

/****************************************************************************
****************************************************************************/
