/**************************************************************************************************
  Filename:       simplekeys.h
  Revised:        $Date: 2010-10-01 14:14:58 -0700 (Fri, 01 Oct 2010) $
  Revision:       $Revision: 23960 $

  Description:    This file contains the Simple Keys Profile header file.


  Copyright 2010 Texas Instruments Incorporated. All rights reserved.

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

#ifndef SIMPLEKEYS_H
#define SIMPLEKEYS_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
  
  #include "st_util.h"

/*********************************************************************
 * CONSTANTS
 */

  
 // TI Base 128-bit UUID: F000XXXX-0451-4000-B000-000000000000
#define GOPRO_BASE_UUID_128( uuid )  0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x46, 0x90, 0xe3, 0x11, 0x8D, 0xAA, LO_UINT16( uuid ), HI_UINT16( uuid ), 0xF9, 0xB5
  
#define GOPRO_UUID(uuid)       GOPRO_BASE_UUID_128(uuid) 
  
  
// Profile Parameters
#define SK_KEY_ATTR                   0  // RW uint8 - Profile Attribute value
 
// GOPRO Service UUID
#define SK_SERV_UUID              0xFEA5
    
// Cmd
#define GOPRO_CMD_UUID            0x00A1

// Cmd Response
#define GOPRO_CMD_RSP_UUID        0x00A2

// Settings
#define GOPRO_SETTINGS_UUID       0x00A3
  
// Settings Response
#define GOPRO_SETTINGS_RSP_UUID   0x00A4  
 
// Query
#define GOPRO_QUERY_UUID         0x00A5 
 
//Query Response
#define GOPRO_QUERY_RSP_UUID     0x00A6

// Query
#define GOPRO_META_UUID          0x00A7 
 
//Query Response
#define GOPRO_META_RSP_UUID      0x00A8   
   
  

// Key Values
#define SK_KEY_LEFT                   0x01
#define SK_KEY_RIGHT                  0x02

// Simple Keys Profile Services bit fields
#define SK_SERVICE                    0x00000001

  
 // Callback Ids
#define GPSDK_ENCODING            0x0A
#define GPSDK_MODE                0x2B
  
  
/*********************************************************************
 * TYPEDEFS
 */
  
// Heart Rate Service callback function
typedef void (*gpSdkServiceCB_t)(uint8 id, uint8 value);
  
  
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */


/*********************************************************************
 * API FUNCTIONS 
 */

/*
 * SK_AddService- Initializes the Simple Key service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */

extern bStatus_t SK_AddService( uint32 services );
  
/*
 * SK_SetParameter - Set a Simple Key Profile parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    pValue - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t SK_SetParameter( uint8 param, uint8 len, void *pValue );
  
/*
 * SK_GetParameter - Get a Simple Key Profile parameter.
 *
 *    param - Profile parameter ID
 *    pValue - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t SK_GetParameter( uint8 param, void *pValue );


bStatus_t gp_MetaNotify( uint16 connHandle, attHandleValueNoti_t *pNoti );
bStatus_t gp_CmdNotify( uint16 connHandle, attHandleValueNoti_t *pNoti );
bStatus_t gp_QueryNotify( uint16 connHandle, attHandleValueNoti_t *pNoti );



extern void GpSdk_Register( gpSdkServiceCB_t pfnServiceCB );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SIMPLEKEYS_H */
