/**************************************************************************************************
  Filename:       simplekey.c
  Revised:        $Date: 2013-05-06 13:33:47 -0700 (Mon, 06 May 2013) $
  Revision:       $Revision: 34153 $

  Description:    Simple Keys Profile


  Copyright 2010 - 2013 Texas Instruments Incorporated. All rights reserved.

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
  LEGAL EQUI THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
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
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"

#include "goproservice.h"
#include "hal_led.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define SERVAPP_NUM_ATTR_SUPPORTED        21



// Position  in attribute array
#define GP_CMD_VALUE_POS        2
#define GP_CMD_CONFIG_POS       3
#define GP_SETTING_VALUE_POS    7
#define GP_SETTING_CONFIG_POS   8
#define GP_QUERY_VALUE_POS      12
#define GP_QUERY_CONFIG_POS     13
#define GP_META_VALUE_POS       17 
#define GP_META_CONFIG_POS      18 


/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */


// SK Service UUID: 0x1800
CONST uint8 skServUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(SK_SERV_UUID), HI_UINT16(SK_SERV_UUID)
};


/*
////////////////////////////////////////////////////////////////////////////////
// 16 bit UUID
////////////////////////////////////////////////////////////////////////////////
// CMD
CONST uint8 gpCmdUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(GOPRO_CMD_UUID), HI_UINT16(GOPRO_CMD_UUID)
};

// CMD_RSP
CONST uint8 gpCmdRspUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(GOPRO_CMD_RSP_UUID), HI_UINT16(GOPRO_CMD_RSP_UUID)
};

//SETTINGS
CONST uint8 gpSettingsUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(GOPRO_SETTINGS_UUID), HI_UINT16(GOPRO_SETTINGS_UUID)
};

//SETTINGS_RSP
CONST uint8 gpSettingsRspUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(GOPRO_SETTINGS_RSP_UUID), HI_UINT16(GOPRO_SETTINGS_RSP_UUID)
};

//STATUS
CONST uint8 gpStatusUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(GOPRO_STATUS_UUID), HI_UINT16(GOPRO_STATUS_UUID)
};

//META
CONST uint8 gpMetaUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(GOPRO_META_UUID), HI_UINT16(GOPRO_META_UUID)
};

*/


///////////////////////////////////////////////////////////////////////////////
//  128
///////////////////////////////////////////////////////////////////////////////

// CMD
static CONST uint8 gpCmdUUID[TI_UUID_SIZE] =
{
  GOPRO_UUID(GOPRO_CMD_UUID),
};


// CMD_RSP
CONST uint8 gpCmdRspUUID[TI_UUID_SIZE] =
{ 
  GOPRO_UUID(GOPRO_CMD_RSP_UUID)
};

//SETTINGS
CONST uint8 gpSettingsUUID[TI_UUID_SIZE] =
{ 
  GOPRO_UUID(GOPRO_SETTINGS_UUID)
};

//SETTINGS_RSP
CONST uint8 gpSettingsRspUUID[TI_UUID_SIZE] =
{ 
  GOPRO_UUID(GOPRO_SETTINGS_RSP_UUID)
};

//QUERY
CONST uint8 gpQueryUUID[TI_UUID_SIZE] =
{ 
 GOPRO_UUID(GOPRO_QUERY_UUID)
};

//QUERY_RSP
CONST uint8 gpQueryRspUUID[TI_UUID_SIZE] =
{ 
  GOPRO_UUID(GOPRO_QUERY_RSP_UUID)
};

//META
CONST uint8 gpMetaUUID[TI_UUID_SIZE] =
{ 
 GOPRO_UUID(GOPRO_META_UUID)
};

//QUERY_RSP
CONST uint8 gpMetaRspUUID[TI_UUID_SIZE] =
{ 
  GOPRO_UUID(GOPRO_META_RSP_UUID)
};



/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static gpSdkServiceCB_t gpSdkServiceCB;

/*********************************************************************
 * Profile Attributes - variables
 */

// SK Service attribute
static CONST gattAttrType_t skService = { ATT_BT_UUID_SIZE, skServUUID };





//CMD
static uint8 gpCmdProps = GATT_PROP_NOTIFY;

//CMD_RSP
static uint8 gpCmdRspProps = GATT_PROP_WRITE;

//SETTINGS
static uint8 gpSettingsProps = GATT_PROP_NOTIFY;

//SETTINGS_RSP
static uint8 gpSettingsRspProps = GATT_PROP_WRITE;

//QUERY
static uint8 gpQueryProps = GATT_PROP_NOTIFY;

//QUERY_RSP
static uint8 gpQueryRspProps = GATT_PROP_WRITE;

//META
static uint8 gpMetaProps = GATT_PROP_NOTIFY;

//META_RSP
static uint8 gpMetaRspProps = GATT_PROP_WRITE;




// Key Pressed State Characteristic
static uint8 skKeyPressed = 0;

// Key Pressed Characteristic Configs
static gattCharCfg_t gpCmdCfg[GATT_MAX_NUM_CONN];
static gattCharCfg_t gpSettingsCfg[GATT_MAX_NUM_CONN];
static gattCharCfg_t gpQueryCfg[GATT_MAX_NUM_CONN];
static gattCharCfg_t gpMetaCfg[GATT_MAX_NUM_CONN];



//values
static uint8 gpCmdValue = 0;
static uint8 gpCmdRspValue = 0;
static uint8 gpSettingsValue = 0;
static uint8 gpSettingsRspValue = 0;
static uint8 gpQueryValue = 0;
static uint8 gpQueryRspValue = 0;
static uint8 gpMetaValue = 0;
static uint8 gpMetaRspValue = 0;


/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t simplekeysAttrTbl[SERVAPP_NUM_ATTR_SUPPORTED] = 
{
  //////////////////////////////////////////////////////////////////////
  // GOPRO SERVICE
  //////////////////////////////////////////////////////////////////////
  { 
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&skService                       /* pValue */
  },


///////////////////////////////////////////
// COMMANDS
///////////////////////////////////////////
    // 1. Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ, 
        0,
        &gpCmdProps 
      },
     
      // 2. Characteristic Value
      { 
        { TI_UUID_SIZE, gpCmdUUID },
        0, 
        0, 
        &gpCmdValue 
      },      
      // 3. Characteristic configuration
      { 
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8 *)&gpCmdCfg 
      },
///////////////////////////////////////////     
// CMD_RSP
///////////////////////////////////////////
      // 4. Characteristic Declaration 
      { 
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ, 
        0,
        &gpCmdRspProps 
      },

      // 5. Characteristic Value
      { 
        { TI_UUID_SIZE, gpCmdRspUUID },
        GATT_PERMIT_WRITE, 
        0, 
        &gpCmdRspValue 
      },
 
///////////////////////////////////////////     
// SETTINGS
/////////////////////////////////////////// 
      // 6.Characteristic Declaration 
      { 
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ, 
        0,
        &gpSettingsProps 
      },

      // 7.Characteristic Value
      { 
        { TI_UUID_SIZE, gpSettingsUUID },
        0, 
        0, 
        &gpSettingsValue 
      },

      // 8.Characteristic configuration
      { 
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8 *)&gpSettingsCfg 
      },

///////////////////////////////////////////     
// SETTINGS_RSP
/////////////////////////////////////////// 
      // 9.Characteristic Declaration 
      { 
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ, 
        0,
        &gpSettingsRspProps 
      },

      // 10.Characteristic Value- Settings RSP
      { 
        { TI_UUID_SIZE, gpSettingsRspUUID },
        GATT_PERMIT_WRITE, 
        0, 
        &gpSettingsRspValue 
      },
      
 ///////////////////////////////////////////////////
 // QUERY
 ////////////////////////////////////////////////// 
      // 11. Query Declaration
      { 
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ, 
        0,
        &gpQueryProps 
      },

      // 12. Query Value
      { 
        { TI_UUID_SIZE, gpQueryUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        &gpQueryValue   //write no response
      },
      
      // 13. Query CCC
      { 
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8 *)&gpQueryCfg 
      },      

///////////////////////////////////////////     
// QUERY_RSP
/////////////////////////////////////////// 
    // 14.Characteristic Declaration 
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &gpQueryRspProps 
    },

      // 15.Characteristic Value- Settings RSP
      { 
        { TI_UUID_SIZE, gpQueryRspUUID },
        GATT_PERMIT_WRITE, 
        0, 
        &gpQueryRspValue 
      },


///////////////////////////////////////////     
// META
///////////////////////////////////////////  
    // 16.Characteristic Declaration for Keys
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &gpMetaProps 
    },

      // 17.Characteristic Value- Meta
      { 
        { TI_UUID_SIZE, gpMetaUUID },
        0, 
        0, 
        &gpMetaValue 
      },

      // 18.Characteristic configuration
      { 
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8 *)&gpMetaCfg 
      },

///////////////////////////////////////////     
// META_RSP
/////////////////////////////////////////// 
    // 19.Characteristic Declaration 
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &gpMetaRspProps 
    },

      // 20.Characteristic Value- Settings RSP
      { 
        { TI_UUID_SIZE, gpMetaRspUUID },
         GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        &gpMetaRspValue 
      },
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 sk_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );
static bStatus_t sk_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset );

static void sk_HandleConnStatusCB( uint16 connHandle, uint8 changeType );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// SK Service Callbacks
CONST gattServiceCBs_t skCBs =
{
  sk_ReadAttrCB,  // Read callback function pointer
  sk_WriteAttrCB, // Write callback function pointer
  NULL            // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SK_AddService
 *
 * @brief   Initializes the Simple Key service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t SK_AddService( uint32 services )
{
  uint8 status = SUCCESS;

  // Initialize Client Characteristic Configuration attributes
  //GATTServApp_InitCharCfg( INVALID_CONNHANDLE, skConfig );
  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, gpCmdCfg );
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, gpSettingsCfg );
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, gpMetaCfg );
  
  

  // Register with Link DB to receive link status change callback
  VOID linkDB_Register( sk_HandleConnStatusCB );  
  
  
  if ( services & SK_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( simplekeysAttrTbl, 
                                          GATT_NUM_ATTRS( simplekeysAttrTbl ),
                                          &skCBs );
  }

  return ( status );
}


/*********************************************************************
 * @fn      GpSdk_Register
 *
 * @brief   Register a callback function with the GoPro SDK Service.
 *
 * @param   pfnServiceCB - Callback function.
 *
 * @return  None.
 */
extern void GpSdk_Register( gpSdkServiceCB_t pfnServiceCB )
{
  gpSdkServiceCB = pfnServiceCB;
}





/*********************************************************************
 * @fn      SK_SetParameter
 *
 * @brief   Set a Simple Key Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to right
 * @param   pValue - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t SK_SetParameter( uint8 param, uint8 len, void *pValue )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case SK_KEY_ATTR:
      if ( len == sizeof ( uint8 ) ) 
      {
        skKeyPressed = *((uint8*)pValue);
        
        // See if Notification/Indication has been enabled
 /*       GATTServApp_ProcessCharCfg( skConfig, &skKeyPressed, FALSE, 
                                    simplekeysAttrTbl, GATT_NUM_ATTRS( simplekeysAttrTbl ),
                                    INVALID_TASK_ID );
 */
 
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
      
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn      SK_GetParameter
 *
 * @brief   Get a Simple Key Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   pValue - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t SK_GetParameter( uint8 param, void *pValue )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case SK_KEY_ATTR:
      *((uint8*)pValue) = skKeyPressed;
      break;
      
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn          sk_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 *
 * @return      Success or Failure
 */
static uint8 sk_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen )
{
  bStatus_t status = SUCCESS;
 
  // Make sure it's not a blob operation (no attributes in the profile are long
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }
 
  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
      // gattserverapp handles this type for reads

      // simple keys characteristic does not have read permissions, but because it
      //   can be sent as a notification, it must be included here
      case GOPRO_CMD_UUID:
        *pLen = 1;
        pValue[0] = *pAttr->pValue;
        break;

      default:
        // Should never get here!
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    // 128-bit UUID
    *pLen = 0;
    status = ATT_ERR_INVALID_HANDLE;
  }

  return ( status );
}

/*********************************************************************
 * @fn      sk_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 *
 * @return  Success or Failure
 */
static bStatus_t sk_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset )
{
  bStatus_t status = SUCCESS;

  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);

    switch ( uuid )
    {
     /* case GATT_CLIENT_CHAR_CFG_UUID:
        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_NOTIFY );
        break;
      */
      
    case GATT_CLIENT_CHAR_CFG_UUID:
        if ( pAttr->handle == simplekeysAttrTbl[GP_CMD_CONFIG_POS].handle )
        {
          // Commands
          status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                   offset, GATT_CLIENT_CFG_NOTIFY );

        }
        else if ( pAttr->handle == simplekeysAttrTbl[GP_SETTING_CONFIG_POS].handle )
        {
          // Settings
          status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                   offset, GATT_CLIENT_CFG_NOTIFY );
        }
        else if ( pAttr->handle == simplekeysAttrTbl[GP_QUERY_CONFIG_POS].handle )
        {
          // Query
          status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                   offset, GATT_CLIENT_CFG_NOTIFY );
        }
        else if ( pAttr->handle == simplekeysAttrTbl[GP_META_CONFIG_POS].handle )
        {
          // Metadata
          status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                   offset, GATT_CLIENT_CFG_NOTIFY );
        }        
        else
        {
          status = ATT_ERR_INVALID_HANDLE;
        }
        break;         
        
        
    case GOPRO_CMD_RSP_UUID:
      
        
        HalLedBlink(HAL_LED_2, 0, 2, 2000);
       // HalLedBlink(HAL_LED_2, 0, 50, 2000);
       // HalLedSet(HAL_LED_2, HAL_LED_MODE_ON );
       // HalLedSet(HAL_LED_2, HAL_LED_MODE_OFF );
        break;
          
      default:
        // Should never get here!
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
    
  }
  else
  {
    // 128-bit UUID
    uint8 uuid128[ATT_UUID_SIZE]; 
    osal_memcpy(uuid128, pAttr->type.uuid, ATT_UUID_SIZE);
    
    ///////////////////////////
    // Query reponse uuid 0xA6
    ///////////////////////////
    if ( osal_memcmp(uuid128, gpQueryRspUUID, ATT_UUID_SIZE))
    {
      
         ///////////////////////////////////////////////////////////////
         // query notification response - assume only one change at a time
         //   todo - add parser with array parsing
         /////////////////////////////////////////////////////////
         if(pValue[1] == 0x93) //notification 
         {
           // status ok
           if( pValue[2] == 0x0)
           {
            // encoding id
            if( pValue[3] == 0x0A)  
            {
              //encoding value
              (*gpSdkServiceCB)( GPSDK_ENCODING,pValue[5]);
            }
            // mode id
            else if(pValue[3] == 0x2B)
            {
                (*gpSdkServiceCB)(GPSDK_MODE,pValue[5]);
            }
           }
         }
         ///////////////////////////////////////////
         // query register response - initial values
         ///////////////////////////////////////////
         else if(pValue[1] == 0x53) 
         {
           //mode id pos 3
           if(pValue[3]==0x2B)
                (*gpSdkServiceCB)(GPSDK_MODE,pValue[5]);
           //mode id pos 6
           if(pValue[6]==0x2B)
                (*gpSdkServiceCB)(GPSDK_MODE,pValue[8]);
 
           //encoding id pos 3
           if(pValue[3]==0x0A)
                (*gpSdkServiceCB)(GPSDK_ENCODING,pValue[5]);
           //encoding id pos 6
           if(pValue[6]==0x0A)
                (*gpSdkServiceCB)(GPSDK_ENCODING,pValue[8]);
         }// reg response
          
    } //end query
    
     // status = ATT_ERR_INVALID_HANDLE;
  } // 128

  return ( status );
}

/*********************************************************************
 * @fn          sk_HandleConnStatusCB
 *
 * @brief       Simple Keys Profile link status change handler function.
 *
 * @param       connHandle - connection handle
 * @param       changeType - type of change
 *
 * @return      none
 */
static void sk_HandleConnStatusCB( uint16 connHandle, uint8 changeType )
{ 
   // Make sure this is not loopback connection
  if ( connHandle != LOOPBACK_CONNHANDLE )
  {
    // Reset Client Char Config if connection has dropped
    if ( ( changeType == LINKDB_STATUS_UPDATE_REMOVED )      ||
         ( ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS ) && 
           ( !linkDB_Up( connHandle ) ) ) )
    { 
      GATTServApp_InitCharCfg( connHandle, gpCmdCfg );
      GATTServApp_InitCharCfg( connHandle, gpMetaCfg );
    }
  }
}

bStatus_t gp_CmdNotify( uint16 connHandle, attHandleValueNoti_t *pNoti )
{
  uint16 value = GATTServApp_ReadCharCfg( connHandle,gpCmdCfg );

  // If notifications enabled
  if ( value & GATT_CLIENT_CFG_NOTIFY )
  {
    // Set the handle
    pNoti->handle = simplekeysAttrTbl[GP_CMD_VALUE_POS].handle;
  
    // Send the Indication
    return GATT_Notification( connHandle, pNoti, FALSE);
   
  }
  return bleIncorrectMode;
  
}

////////////////////////////////////////////
//  META NOTIFY 
////////////////////////////////////////////
bStatus_t gp_MetaNotify( uint16 connHandle, attHandleValueNoti_t *pNoti )
{
  uint16 value = GATTServApp_ReadCharCfg( connHandle,gpMetaCfg );
 
  // If notifications enabled
  if ( value & GATT_CLIENT_CFG_NOTIFY )
  {
    // Set the handle
    pNoti->handle = simplekeysAttrTbl[GP_META_VALUE_POS].handle;
  
    // Send the Indication
/*
    SUCCESS: Notification was sent successfully.<BR>
    INVALIDPARAMETER: Invalid connection handle or request field.<BR>
    MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
    bleNotConnected: Connection is down.<BR>
    bleMemAllocError: Memory allocation error occurred.<BR>
    bleTimeout: Previous transaction timed out.<BR>    
*/
    
      return GATT_Notification( connHandle, pNoti, FALSE);
  }
  return bleIncorrectMode;
  
}


/////////////////////////////////////////////
//  QUERY NOTIFY 
////////////////////////////////////////////
bStatus_t gp_QueryNotify( uint16 connHandle, attHandleValueNoti_t *pNoti )
{
  uint16 value = GATTServApp_ReadCharCfg( connHandle,gpQueryCfg );
 
  if ( value & GATT_CLIENT_CFG_NOTIFY )
  {
    // Set the handle
    pNoti->handle = simplekeysAttrTbl[GP_QUERY_VALUE_POS].handle;
    return GATT_Notification( connHandle, pNoti, FALSE);
  }
  
  return bleIncorrectMode;
}




/*********************************************************************
*********************************************************************/
