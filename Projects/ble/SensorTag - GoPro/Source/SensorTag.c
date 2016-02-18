/**************************************************************************************************
  Filename:       sensorTag.c
  Revised:        $Date: 2013-08-23 11:45:31 -0700 (Fri, 23 Aug 2013) $
  Revision:       $Revision: 35100 $

  Description:    This file contains the Sensor Tag sample application
                  for use with the TI Bluetooth Low Energy Protocol Stack.

  Copyright 2012-2013  Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_i2c.h"

#include "gatt.h"
#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"

#if defined ( PLUS_BROADCASTER )
  #include "peripheralBroadcaster.h"
#else
  #include "peripheral.h"
#endif

#include "gapbondmgr.h"

#if defined FEATURE_OAD
  #include "oad.h"
  #include "oad_target.h"
#endif


// Services
//#include "st_util.h"
#include "devinfoservice-st.h"
//#include "irtempservice.h"
//#include "accelerometerservice.h"
//#include "humidityservice.h"
//#include "magnetometerservice.h"
//#include "barometerservice.h"
//#include "gyroservice.h"
//#include "testservice.h"
#include "goproservice.h"
#include "ccservice.h"

// Sensor drivers
#include "sensorTag.h"
#include "hal_sensor.h"

//#include "hal_irtemp.h"
//#include "hal_acc.h"
//#include "hal_humi.h"
//#include "hal_mag.h"
//#include "hal_bar.h"
//#include "hal_gyro.h"


#include "stdio.h"
#include "string.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define GOPRO_PACKET_RATE_MS    30     // how often we will send metadata packets
#define GOPRO_PACKET_COUNT       3     // number of packets to send

// How often to perform sensor reads (milliseconds)
#define TEMP_DEFAULT_PERIOD                   1000
#define HUM_DEFAULT_PERIOD                    1000
#define BAR_DEFAULT_PERIOD                    1000
#define MAG_DEFAULT_PERIOD                    2000
#define ACC_DEFAULT_PERIOD                    1000
#define GYRO_DEFAULT_PERIOD                   1000

// Constants for two-stage reading
#define TEMP_MEAS_DELAY                       275   // Conversion time 250 ms
#define BAR_FSM_PERIOD                        80
#define ACC_FSM_PERIOD                        20
#define HUM_FSM_PERIOD                        20
#define GYRO_STARTUP_TIME                     60    // Start-up time max. 50 ms

// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_LIMITED

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         8

// Company Identifier: Texas Instruments Inc. (13)
#define TI_COMPANY_ID                         0x000D

#define INVALID_CONNHANDLE                    0xFFFF

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

#if defined ( PLUS_BROADCASTER )
  #define ADV_IN_CONN_WAIT                    500 // delay 500 ms
#endif

// Side key bit
#define SK_KEY_SIDE                           0x04

// Test mode bit
#define TEST_MODE_ENABLE                      0x80

// Common values for turning a sensor on and off + config/status
#define ST_CFG_SENSOR_DISABLE                 0x00
#define ST_CFG_SENSOR_ENABLE                  0x01
#define ST_CFG_CALIBRATE                      0x02
#define ST_CFG_ERROR                          0xFF

// System reset
#define ST_SYS_RESET_DELAY                    3000

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
   
   
   
   

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8 sensorTag_TaskID;   // Task ID for internal task/event processing

static gaprole_States_t gapProfileState = GAPROLE_INIT;

static attHandleValueNoti_t gpCmdNotification;
static attHandleValueNoti_t gpMetaNotification;
static attHandleValueNoti_t gpNotification;

static uint8 shutterValue = 0;
static uint32 hrm_counter = 0;
static uint32 sticky_counter = 20;  
static uint8 sysResetRequest = FALSE;

static uint8 status_encoding=0;
static uint8 status_mode=0;

static uint8 failedCmd=0;
static uint8 modeValue = 0;



// GAP - SCAN RSP data (max size = 31 bytes)

/*
static uint8 scanRspData[] =
{
  // complete name
  0x0A,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  0x53,   // 'S'
  0x65,   // 'e'
  0x6E,   // 'n'
  0x73,   // 's'
  0x6F,   // 'o'
  0x72,   // 'r'
  '-',   // 'T'
  '0',   // 'a'
  '0',   // 'g'
  
  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
  HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
  LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),
  HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0    // 0dBm
    
};
*/

static uint8 scanRspData[] =
{
  // complete name
  0x0A,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  0x53,   // 'S'
  0x65,   // 'e'
  0x6E,   // 'n'
  0x73,   // 's'
  0x6F,   // 'o'
  0x72,   // 'r'
  '-',   // 'T'
  '0',   // 'a'
  '0',   // 'g'
  
  // MFG SPECIFIC DATA
  0x13,   // length of this data
  GAP_ADTYPE_MANUFACTURER_SPECIFIC,
  0x15,
  0x16,
  0x17,
  0x18,
  0x19,
  0x1a,
  0x1b,
  0x1c,
  0x1d,
  0x1e,
  0x1f,
  0x20,
  0x21,
  0x22,
  0x23,
  0x24,
  0x25,
  0x26  
  
    
};


// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)


/*
static uint8 advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
   
  0x03,   // length of this data 
  GAP_ADTYPE_16BIT_MORE,
  0xFE,
  0xA5,
  // complete name
  0x0A,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  0x53,   // 'S'
  0x65,   // 'e'
  0x6E,   // 'n'
  0x73,   // 's'
  0x6F,   // 'o'
  0x72,   // 'r'
  '-',   // 'T'
  '0',   // 'a'
  '0'   // 'g'
};

*/


////////////////////////
//  TEST SMARTY- GREG
////////////////////////

static uint8 advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
   
  0x05,   // length of this data 
  GAP_ADTYPE_16BIT_MORE,
  0xA5,
  0xFE,
  0x0A,
  0x18,  
  
  0x15,   //21 bytes
  GAP_ADTYPE_SERVICE_DATA,  //MFG Specific
  0x01,
  0x02,
  0x03,
  0x04,
  0x05,
  0x06,
  0x07,
  0x08,
  0x09,
  0x0a,
  0x0b,
  0x0c,
  0x0d,
  0x0e,
  0x0f,
  0x10,
  0x11,
  0x12,
  0x13,
  0x14  
};










// GAP GATT Attributes
static uint8 attDeviceName[] = "TI BLE Sensor Tag";



static bool   testMode = FALSE;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void sensorTag_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );
static bStatus_t sendShutter(uint8 shutterValue);
static void gpSdkCB(uint8 event, uint8 value);


// METADATA
static void sendMeta_DeviceName();
static void sendMeta_Register4CC();
static void sendMeta_RegisterSticky4CC();

//QUERY
static bStatus_t sendQuery_RegisterForStatus();



static bStatus_t sendMeta_Data();
static bStatus_t sendMode(uint8 modeValue);
static bStatus_t sendMeta_DataSticky();

static void gapRolesParamUpdateCB( uint16 connInterval, uint16 connSlaveLatency,uint16 connTimeout );

static void resetSensorSetup( void );
static void sensorTag_HandleKeys( uint8 shift, uint8 keys );
//static void resetCharacteristicValue( uint16 servID, uint8 paramID, uint8 value, uint8 paramLen );
//static void resetCharacteristicValues( void );

static char *bdAddr2Str ( uint8 *pAddr );

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t sensorTag_PeripheralCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller (not used by application)
};

// GAP Bond Manager Callbacks
static gapBondCBs_t sensorTag_BondMgrCBs =
{
  NULL,                     // Passcode callback (not used by application)
  NULL                      // Pairing / Bonding state Callback (not used by application)
};



static gapRolesParamUpdateCB_t paramUpdateCB =
{
  gapRolesParamUpdateCB,
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SensorTag_Init
 *
 * @brief   Initialization function for the Simple BLE Peripheral App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */

#define B_ADDR_STR_LEN                        15

static uint8 metadata_state = 0;
static uint8 register_state = 0;

void SensorTag_Init( uint8 task_id )
{
  sensorTag_TaskID = task_id;
  metadata_state = 0;
  register_state = 0;
        
  // Setup the GAP
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );

  //static uint8 bdAddress[6] = {0x04,0x04,0x04,0x04,0x04,0x06};
  
  //static uint8 bdAddress[6] = {0x05,0x05,0x05,0x05,0x05,0x05};
  //HCI_EXT_SetBDADDRCmd(bdAddress);
  
  
  // Setup the GAP Peripheral Role Profile
  {
    // Device starts advertising upon initialization
    uint8 initial_advertising_enable = FALSE;

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 0;
    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );

    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );

    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }

  // Set the GAP Characteristics
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, sizeof(attDeviceName), attDeviceName );

  // Set advertising interval
  {
    uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
  }

  // Setup the GAP Bond Manager
  {
    uint32 passkey = 0; // passkey "000000"
    uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8 mitm = FALSE;
    uint8 ioCap = GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT;
    uint8 bonding = TRUE;

    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
  }


  // Add services
  GGS_AddService( GATT_ALL_SERVICES );            // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
  DevInfo_AddService();                           // Device Information Service
  
  SK_AddService( GATT_ALL_SERVICES );             // Simple Keys Profile
 
  CcService_AddService( GATT_ALL_SERVICES );      // Connection Control Service

#if defined FEATURE_OAD
  VOID OADTarget_AddService();                    // OAD Profile
#endif

  // Setup the Seensor Profile Characteristic Values
  //resetCharacteristicValues();

  // Register for all key events - This app will handle all key events
  RegisterForKeys( sensorTag_TaskID );

  // makes sure LEDs are off
  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );

  
  
  // Initialise sensor drivers
 
 
  //HALIRTempInit();
  //HalHumiInit();
  //HalMagInit();
  //HalAccInit();
  //HalBarInit();
  //HalGyroInit();
/*
  
  // Register callbacks with profile
  VOID IRTemp_RegisterAppCBs( &sensorTag_IrTempCBs );
  VOID Magnetometer_RegisterAppCBs( &sensorTag_MagnetometerCBs );
  VOID Accel_RegisterAppCBs( &sensorTag_AccelCBs );
  VOID Humidity_RegisterAppCBs( &sensorTag_HumidCBs );
  VOID Barometer_RegisterAppCBs( &sensorTag_BarometerCBs );
  VOID Gyro_RegisterAppCBs( &sensorTag_GyroCBs );
  VOID Test_RegisterAppCBs( &sensorTag_TestCBs );
  VOID CcService_RegisterAppCBs( &sensorTag_ccCBs );
*/  
  
  // Register for gopro sdk service callback
  GpSdk_Register( gpSdkCB );
  
  VOID GAPRole_RegisterAppCBs( &paramUpdateCB );

  // Enable clock divide on halt
  // This reduces active current while radio is active and CC254x MCU
  // is halted
  HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );

  
  // Setup a delayed profile startup
  osal_set_event( sensorTag_TaskID, ST_START_DEVICE_EVT );
}
/*********************************************************************
 * @fn      gpSdkCB
 *
 * @brief   Callback function for gpSdk service.
 *
 * @param   event - service event
 *
 * @return  none
 */
static void gpSdkCB(uint8 event, uint8 value)
{

  switch(event)
      {
      case GPSDK_ENCODING:
            status_encoding = value;
            break;
       case GPSDK_MODE:
            status_mode = value;
            modeValue = status_mode;
            break;
      default:
          break;
      }
}


/*********************************************************************
 * @fn      SensorTag_ProcessEvent
 *
 * @brief   Simple BLE Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include s, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */




uint16 SensorTag_ProcessEvent( uint8 task_id, uint16 events )
{
  VOID task_id; // OSAL required parameter that isn't used in this function

  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( sensorTag_TaskID )) != NULL )
    {
      sensorTag_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Handle system reset (long press on side key)
  /*
  if ( events & ST_SYS_RESET_EVT )
  {
    if (sysResetRequest)
    {
      HAL_SYSTEM_RESET();
    }
    return ( events ^ ST_SYS_RESET_EVT );
  }
*/

  if ( events & ST_SYS_RESET_EVT )
  {
    if (sysResetRequest)
    {
      HAL_SYSTEM_RESET();
    }
    return ( events ^ ST_SYS_RESET_EVT );
  }


/////////////////////////////////////
// Command Timer
//
//  Retry commands if they fail
//  
//////////////////////////////////////  
if ( events & ST_COMMAND_TIMER_EVT )
{
  
  switch (failedCmd)
  {
    case 0x01:
      if(sendShutter(shutterValue) == SUCCESS)
      {
        failedCmd = 0;
      }
      else
      {
        failedCmd = 0x01;
      }
        
      break;
  case 0x02:
      if(sendMode(modeValue)==SUCCESS)
      {
        failedCmd = 0;
      }
      else
      {
        failedCmd = 0x02;
      }
      break;
    default:
      break;
  }
  
  // keep trying
  if(failedCmd > 0)
    osal_start_timerEx( sensorTag_TaskID, ST_COMMAND_TIMER_EVT, 30 ); 
  
  return ( events ^ ST_COMMAND_TIMER_EVT );  
}

  
/////////////////////////////////////
// Metadata Timer
//
//  if success increment state machine until we are jsut sending metadata
//  
//////////////////////////////////////  
if ( events & ST_HRM_TIMER_EVT )
  {
      switch(metadata_state)
      {
      case 0:
        sendMeta_DeviceName();  
        break;
      
      case 1:
         sendMeta_Register4CC();
         break;
      
      case 2:
         sendMeta_RegisterSticky4CC();
         break;

      case 3:
        sendMeta_DataSticky();
        break;  
         
      case 4:
  
        
//#define GOPRO_PACKET_RATE_MS  1000     // how often we will send metadata packets
//#define GOPRO_PACKET_COUNT       1     // number of packets to send    
        
        //only send metadata if there are no pending commands  
        if(failedCmd == 0)
        { 
          for(int i=0; i < GOPRO_PACKET_COUNT;i++)
          {
            if(sendMeta_Data()==SUCCESS)
                hrm_counter++;
          }
        }
        break;
        
      default:
        break;
        
    }   
    
    if(hrm_counter >= 0xFFFF)
    {
        hrm_counter = 0;
    }
    
    //send data every x ms
    osal_start_timerEx( sensorTag_TaskID, ST_HRM_TIMER_EVT, GOPRO_PACKET_RATE_MS ); 
    
    return ( events ^ ST_HRM_TIMER_EVT );
  }

  
/////////////////////////////////////
// REGISTER TIMER
//
//  - register for client char configs
//  - register for query status
//     - encoding
//     - mode
//////////////////////////////////////  
if ( events & ST_REGISTER_TIMER_EVT )
  {
      switch(register_state)
      {
      case 0:
        if (sendQuery_RegisterForStatus() == SUCCESS)
        {
          register_state++;
        }
        osal_start_timerEx( sensorTag_TaskID, ST_REGISTER_TIMER_EVT, 100 );
        break;
      
      case 1:
         
         break;
      
      case 2:

         break;

      case 3:

        break;  
         
      case 4:
        
        break;
        
      default:
        break;
    }   
    
    return ( events ^ ST_REGISTER_TIMER_EVT );
  }

  
  
  
  if ( events & ST_START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPRole_StartDevice( &sensorTag_PeripheralCBs );

    // Start Bond Manager
    VOID GAPBondMgr_Register( &sensorTag_BondMgrCBs );

    return ( events ^ ST_START_DEVICE_EVT );
  }

  
  
  
#if defined ( PLUS_BROADCASTER )
  if ( events & ST_ADV_IN_CONNECTION_EVT )
  {
    uint8 turnOnAdv = TRUE;
    // Turn on advertising while in a connection
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &turnOnAdv );

    return (events ^ ST_ADV_IN_CONNECTION_EVT);
  }
#endif // PLUS_BROADCASTER

  // Discard unknown events
  return 0;
}



/*********************************************************************
* Private functions
*/


/*********************************************************************
 * @fn      sensorTag_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void sensorTag_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
    case KEY_CHANGE:
      sensorTag_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;

    default:
      // do nothing
      break;
  }
}

/*********************************************************************
 * @fn      sensorTag_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */

static void sensorTag_HandleKeys( uint8 shift, uint8 keys )
{
  uint8 SK_Keys = 0;
  VOID shift;  // Intentionally unreferenced parameter

  
  
  if (keys & HAL_KEY_SW_1)
  {
    // Reset the system if side key is pressed for more than 3 seconds
   // sysResetRequest = TRUE;
    osal_start_timerEx( sensorTag_TaskID, ST_SYS_RESET_EVT, ST_SYS_RESET_DELAY );

    if (!testMode ) // Side key
    {
      // If device is not in a connection, pressing the side key should toggle
      //  advertising on and off
      if ( gapProfileState != GAPROLE_CONNECTED )
      {
        uint8 current_adv_enabled_status;
        uint8 new_adv_enabled_status;

        // Find the current GAP advertising status
        GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &current_adv_enabled_status );

        if( current_adv_enabled_status == FALSE )
        {
          new_adv_enabled_status = TRUE;
        }
        else
        {
          new_adv_enabled_status = FALSE;
        }

        // Change the GAP advertisement status to opposite of current status
        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );
      }

      
      /////////////////////////////////////////////////////
      //  SIDE BUTTON - trigger metadata if in connection
      ////////////////////////////////////////////////////
      if ( gapProfileState == GAPROLE_CONNECTED )
      {
        //uint8 adv_enabled = TRUE;
        // Disconnect
        //GAPRole_TerminateConnection();
        // Start advertising
        //GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &adv_enabled );
        
        //if we haven't started metadata counter 
        if(  hrm_counter == 0)
        {
          //start HR time
          osal_start_timerEx( sensorTag_TaskID, ST_HRM_TIMER_EVT, 500 );
          failedCmd=0x0;
        }
        else
        {
          failedCmd=0x03; 
         
          //start HR time
          osal_stop_timerEx( sensorTag_TaskID, ST_HRM_TIMER_EVT);
 
          //reset counter
          hrm_counter = 0;
         }
        
      }
    }
    else
    {
      // Test mode
      if ( keys & HAL_KEY_SW_1 ) // Side key
      {
        SK_Keys |= SK_KEY_SIDE;
      }
    }
  }

  
  ///////////////////////////////
  //  Right button - SHUTTER
  ////////////////////////////////
  if ( keys & 2 )  
  {
    //if video mode and encoding on
    if(status_mode == 0 && status_encoding == 1)
    {
      shutterValue = 0;
    }
    else
    {
       shutterValue = 1;
    }
    
    if(sendShutter(shutterValue) != SUCCESS )
    {
      failedCmd = 0x01;
      osal_start_timerEx( sensorTag_TaskID, ST_COMMAND_TIMER_EVT, 30 ); 
    }
  }
  
  ///////////////////////////////
  // Left Button - MODE CYCLE
  ////////////////////////////////
  if ( keys & 4 ) 
  {
      if(modeValue++==2)
      {
       modeValue=0;
      }
    
      //send mode
      if( sendMode(modeValue) != SUCCESS)
      {
        failedCmd = 0x02;
        osal_start_timerEx( sensorTag_TaskID, ST_COMMAND_TIMER_EVT, 30 ); 
      }
        
  }
  
  if (!(keys & HAL_KEY_SW_1))
  {
    // Cancel system reset request
    //sysResetRequest = FALSE;
  }

  // Set the value of the keys state to the Simple Keys Profile;
  // This will send out a notification of the keys state if enabled
  //SK_SetParameter( SK_KEY_ATTR, sizeof ( uint8 ), &SK_Keys );
}


static bStatus_t sendShutter(uint8 shutterValue)
{
   uint8 *p = gpCmdNotification.value;
  
   uint8 len =  0x03;    // shutter
   uint8 cmd =  0x01;    // shutter
   uint8 plen = 0x01;    // param len 
 
   *p++ = len;  
   *p++ = cmd;
   *p++ = plen;
   *p++ = shutterValue;
   
    gpCmdNotification.len = (uint8) (p - gpCmdNotification.value);
  
    return gp_CmdNotify( 0, &gpCmdNotification );
}

//////////////////////////////////////
// MODE
/////////////////////////////////////
static bStatus_t sendMode(uint8 modeValue)
{
   uint8 *p = gpCmdNotification.value;
  
   uint8 len =  0x03;    // shutter
   uint8 cmd =  0x02;    // mode
   uint8 plen = 0x01;    // param len 
 
   *p++ = len;  
   *p++ = cmd;
   *p++ = plen;
   *p++ = modeValue;
   
    gpCmdNotification.len = (uint8) (p - gpCmdNotification.value);
  
    return (gp_CmdNotify( 0, &gpCmdNotification ));
  
}




//////////////////////////////////////
// META DATA - DEVICE NAME
/////////////////////////////////////
static void sendMeta_DeviceName()
{
  
   uint8 *p = gpMetaNotification.value;
  
   *p++ = 0x0B;  //length
   *p++ = 0x02;
   *p++ = 0x02;
   *p++ = 0x53;  //S
   *p++ = 0x45;  //E
   *p++ = 0x4E;  //N       
   *p++ = 0x53;  //S
   *p++ = 0x4F;  //O
   *p++ = 0x52;  //R
   *p++ = scanRspData[9];  // From bdAddr    
   *p++ = scanRspData[10];  // From bdAddr   
   *p++ = 0x00;  //0
   
    gpMetaNotification.len = (uint8) (p - gpMetaNotification.value);
  
    if(gp_MetaNotify( 0, &gpMetaNotification )==0)
    {
        metadata_state++;
    }
      
}


//////////////////////////////////////
// META DATA - REGISTER 4CC
/////////////////////////////////////
static void sendMeta_Register4CC()
{
  
   uint8 *p = gpMetaNotification.value;
  
   *p++ = 0x0B;  // Len
   *p++ = 0x03;  // CMD = Register 4CC
   *p++ = 0x61;  // A
   *p++ = 0x63;  // C
   *p++ = 0x63;  // C
   *p++ = 0x31;  // 1       
   *p++ = 0x01;  // id
   *p++ = 0x00;  // stream
   *p++ = 0x62;  // B byte  
   *p++ = 0x02;  // len 2
   *p++ = 0x02;  // qty 2
   *p++ = 0x00;  // non sticky  
  
    gpMetaNotification.len = (uint8) (p - gpMetaNotification.value);
  
    if( gp_MetaNotify( 0, &gpMetaNotification )==0)
    {
      metadata_state++;
    }
}


//////////////////////////////////////
// META DATA - REGISTER 4CC - STICKY
/////////////////////////////////////
static void sendMeta_RegisterSticky4CC()
{
  
   uint8 *p = gpMetaNotification.value;
  
   *p++ = 0x0B;  // Len
   *p++ = 0x03;  // CMD = Register 4CC
   *p++ = 0x4D;  // B
   *p++ = 0x46;  // C
   *p++ = 0x47;  // C
   *p++ = 0x49;  // 1       
   *p++ = 0x02;  // id   ***** make sure this is unique
   *p++ = 0x00;  // stream
   *p++ = 0x62;  // B byte  
   *p++ = 0x02;  // len 2
   *p++ = 0x02;  // qty 2
   *p++ = 0x01;  // sticky  
  
    gpMetaNotification.len = (uint8) (p - gpMetaNotification.value);
  
    // if fails, we will just not incrment state, which will result in retry
    if( gp_MetaNotify( 0, &gpMetaNotification )==0)
    {
      metadata_state++;
    }
}

//////////////////////////////////////
// META DATA - DATA
/////////////////////////////////////
static bStatus_t sendMeta_DataSticky()
{
   // *p++ = hrm_counter;
   uint8 *p = gpMetaNotification.value;
  
   *p++ = 0x06;
   *p++ = 0x01;
   *p++ = 0x02; //4CC Index
   
   char buf[5] = {0x0,0x0,0x0,0x0,0x0};
   
   // convert to ascii for display
   sprintf(buf,"%04lu",sticky_counter);
 
   *p++ = buf[0];
   *p++ = buf[1];
   *p++ = buf[2];
   *p++ = buf[3];
   
    gpMetaNotification.len = (uint8) (p - gpMetaNotification.value);
  
    // if fails, we will just not incrment state, which will result in retry
    if( gp_MetaNotify( 0, &gpMetaNotification )==0)
    {
      metadata_state++;
    }
   
    return 0;
    
}


//////////////////////////////////////
// META DATA - DATA
/////////////////////////////////////
static bStatus_t sendMeta_Data()
{
   // *p++ = hrm_counter;
   uint8 *p = gpMetaNotification.value;
  
   *p++ = 0x06;
   *p++ = 0x01;
   *p++ = 0x01;
   
   char buf[5] = {0x0,0x0,0x0,0x0,0x0};
   
   // convert to ascii for display
   sprintf(buf,"%04lu",hrm_counter);
 
   *p++ = buf[0];
   *p++ = buf[1];
   *p++ = buf[2];
   *p++ = buf[3];
   
  //*p++= hrm_counter;
   
    gpMetaNotification.len = (uint8) (p - gpMetaNotification.value);
  
    return gp_MetaNotify( 0, &gpMetaNotification );
  
}


//////////////////////////////////////
// QUERY - Register for Status
/////////////////////////////////////
static bStatus_t sendQuery_RegisterForStatus()
{
   uint8 *p = gpNotification.value;
  
   *p++ = 0x03; // length
   *p++ = 0x53; // register for status
   *p++ = 0x0A; // encode status
   *p++ = 0x2B; // mode
   
    gpNotification.len = (uint8) (p - gpNotification.value);
  
    return gp_QueryNotify( 0, &gpNotification );
}


/*********************************************************************
 * @fn      resetSensorSetup
 *
 * @brief   Turn off all sensors that are on
 *
 * @param   none
 *
 * @return  none
 */
static void resetSensorSetup (void)
{
  /*
  if (HalIRTempStatus()!=TMP006_OFF || irTempEnabled)
  {
    HalIRTempTurnOff();
    irTempEnabled = FALSE;
  }

  if (accConfig != ST_CFG_SENSOR_DISABLE)
  {
    accConfig = ST_CFG_SENSOR_DISABLE;
  }

  if (HalMagStatus()!=MAG3110_OFF || magEnabled)
  {
    HalMagTurnOff();
    magEnabled = FALSE;
  }

  if (gyroEnabled)
  {
    HalGyroTurnOff();
    gyroEnabled = FALSE;
  }

  if (barEnabled)
  {
    HalBarInit();
    barEnabled = FALSE;
  }

  if (humiEnabled)
  {
    HalHumiInit();
    humiEnabled = FALSE;
  }

  // Reset internal states
  sensorGyroAxes = 0;
  sensorGyroUpdateAxes = FALSE;
  testMode = FALSE;

  // Reset all characteristics values
  resetCharacteristicValues();
  
  */
  
}


/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
  HalLedSet(HAL_LED_1, HAL_LED_MODE_OFF );
  HalLedSet(HAL_LED_2, HAL_LED_MODE_OFF );
   
  switch ( newState )
  {
   
    case GAPROLE_STARTED:
    {
      uint8 ownAddress[B_ADDR_LEN];
      uint8 systemId[DEVINFO_SYSTEM_ID_LEN];

      GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

      // use 6 bytes of device address for 8 bytes of system ID value
      systemId[0] = ownAddress[0];
      systemId[1] = ownAddress[1];
      systemId[2] = ownAddress[2];

      // set middle bytes to zero
      systemId[4] = 0x00;
      systemId[3] = 0x00;

      // shift three bytes up
      systemId[7] = ownAddress[5];
      systemId[6] = ownAddress[4];
      systemId[5] = ownAddress[3];

      DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);
      
      char sAddress[B_ADDR_STR_LEN] = {0};

        //uint8 ownAddress[B_ADDR_LEN];
    
        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);
        
        memcpy(sAddress, bdAddr2Str( ownAddress ),B_ADDR_STR_LEN );
   
        scanRspData[9] = sAddress[12];
        scanRspData[10] = sAddress[13];

        GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof( scanRspData ), scanRspData );         
      
      
    }
    break;

    case GAPROLE_ADVERTISING:
	//HalLedSet(HAL_LED_1, HAL_LED_MODE_ON );
            
       HalLedBlink (HAL_LED_1, 0, 2, 250);
       
       osal_stop_timerEx( sensorTag_TaskID, ST_HRM_TIMER_EVT );
       osal_stop_timerEx( sensorTag_TaskID, ST_REGISTER_TIMER_EVT );
       osal_stop_timerEx( sensorTag_TaskID, ST_COMMAND_TIMER_EVT );
       
      
      //HalLedSet(HAL_LED_2, HAL_LED_MODE_FLASH );
	    break;

    case GAPROLE_CONNECTED:
      
      //HalLedBlink (HAL_LED_1, 0, 2, 1000);
      HalLedSet(HAL_LED_1, HAL_LED_MODE_ON );
      
      //HalLedSet(HAL_LED_2, HAL_LED_MODE_ON );
      
      //reset counter
      hrm_counter = 0;
      
      //reset metadata state
      metadata_state=0;
      status_encoding=0;
      status_mode=0;
      modeValue = 0;
      failedCmd = 0;

      // Register for status on connection ( allow time for camera  to enable CCC)
      register_state = 0;
      osal_start_timerEx( sensorTag_TaskID, ST_REGISTER_TIMER_EVT, 5000 ); 
      
      
      break;

    case GAPROLE_WAITING:
      // Link terminated intentionally: reset all sensors
      resetSensorSetup();
      break;

	  default:
	    break;
  }

  gapProfileState = newState;
}


/*********************************************************************
 * @fn      gapRolesParamUpdateCB
 *
 * @brief   Called when connection parameters are updates
 *
 * @param   connInterval - new connection interval
 *
 * @param   connSlaveLatency - new slave latency
 *
 * @param   connTimeout - new connection timeout
 *
 * @return  none
*/
static void gapRolesParamUpdateCB( uint16 connInterval, uint16 connSlaveLatency,uint16 connTimeout )
{
  uint8 buf[CCSERVICE_CHAR1_LEN];

  buf[0] = LO_UINT16(connInterval);
  buf[1] = HI_UINT16(connInterval);
  buf[2] = LO_UINT16(connSlaveLatency);
  buf[3] = HI_UINT16(connSlaveLatency);
  buf[4] = LO_UINT16(connTimeout);
  buf[5] = HI_UINT16(connTimeout);
  CcService_SetParameter(CCSERVICE_CHAR1,sizeof(buf),buf);
}


/*********************************************************************
 * @fn      resetCharacteristicValue
 *
 * @brief   Initialize a characteristic value to zero
 *
 * @param   servID - service ID (UUID)
 *
 * @param   paramID - parameter ID of the value is to be cleared
 *
 * @param   vakue - value to initialise with
 *
 * @param   paramLen - length of the parameter
 *
 * @return  none
 */
/*
static void resetCharacteristicValue(uint16 servUuid, uint8 paramID, uint8 value, uint8 paramLen)
{
  uint8* pData = osal_mem_alloc(paramLen);

  if (pData == NULL)
  {
    return;
  }

  osal_memset(pData,value,paramLen);

 

  osal_mem_free(pData);
}
*/

                 
                 
                 




/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string. Only needed when
 *          LCD display is used.
 *
 * @return  none
 */
char *bdAddr2Str( uint8 *pAddr )
{
  uint8       i;
  char        hex[] = "0123456789ABCDEF";
  static char str[B_ADDR_STR_LEN];
  char        *pStr = str;

  *pStr++ = '0';
  *pStr++ = 'x';

  // Start from end of addr
  pAddr += B_ADDR_LEN;

  for ( i = B_ADDR_LEN; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }

  *pStr = 0;

  return str;
}
