
/******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  *
  * Licensed under ST MYLIBERTY SOFTWARE LICENSE AGREEMENT (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/myliberty
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied,
  * AND SPECIFICALLY DISCLAIMING THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, AND NON-INFRINGEMENT.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
******************************************************************************/

/*
 *      PROJECT:   ST25R391x firmware
 *      $Revision: $
 *      LANGUAGE:  ISO C99
 */

/*! \file rfal_dynamicPower.h
 *
 *  \author Martin Zechleitner
 *
 *  \brief Dynamic Power adjustment
 *  
 *  This module provides an interface to perform the power adjustment dynamically 
 *  
 *  
 * @addtogroup RFAL
 * @{
 *
 * @addtogroup RFAL-HAL
 * @brief RFAL Hardware Abstraction Layer
 * @{
 *
 * @addtogroup DynamicPower
 * @brief RFAL Dynamic Power Module
 * @{
 * 
 */


#ifndef RFAL_DYNAMICPOWER_H
#define RFAL_DYNAMICPOWER_H

/*
 ******************************************************************************
 * INCLUDES
 ******************************************************************************
 */
#include "platform.h"
#include "st_errno.h"

/*
 ******************************************************************************
 * GLOBAL DEFINES
 ******************************************************************************
 */

#define RFAL_DYNAMIC_POWER_TABLE_SIZE_MAX 15
#define RFAL_DYNAMIC_POWER_TABLE_PAPAMETER 3

/*
******************************************************************************
* GLOBAL TYPES
******************************************************************************
*/

typedef struct {
    uint8_t rfoRes; /*!< Setting for the resistance level of the RFO */
    uint8_t inc;    /*!< Threshold for incrementing the output power */ 
    uint8_t dec;    /*!< Threshold for decrementing the output power */
}rfalDynamicPowerEntry;

/*
******************************************************************************
* GLOBAL FUNCTION PROTOTYPES
******************************************************************************
*/


/*! 
 *****************************************************************************
 * \brief  Initialize dynamic power table
 *  
 *  This function sets the internal dynamic power table to the default 
 *  values stored in rfal_dynamicPowerTbl.h
 *  
 *****************************************************************************
 */
void rfalDynamicPowerInitialize( void );


/*! 
 *****************************************************************************
 * \brief  Dynamic power table Load
 *  
 * Load the dynamic power table  
 *
 * \param[in]  powerTbl:     location of power Table to be loaded
 * \param[in]  powerTblEntries: number of entries of the power Table to be loaded
 * 
 * \return ERR_NONE    : No error
 * \return ERR_PARAM   : if configTbl is invalid
 * \return ERR_NOMEM   : if the given Table is bigger exceeds the max size
 *****************************************************************************
 */
ReturnCode rfalDynamicPowerTableLoad( rfalDynamicPowerEntry* powerTbl, uint8_t powerTblEntries );

/*! 
 *****************************************************************************
 * \brief  Dynamic power table Read
 *  
 * Read the dynamic power table  
 *
 * \param[out]   tblBuf: location to the rfalDynamicPowerEntry[] to place the Table 
 * \param[in]    tblBufEntries: number of entries available in tblBuf to place the power Table
 * \param[out]   tableEntries: returned number of entries actually written into tblBuf
 * 
 * \return ERR_NONE    : No error
 * \return ERR_PARAM   : if configTbl is invalid or parameters are invalid
 *****************************************************************************
 */
ReturnCode rfalDynamicPowerTableRead( rfalDynamicPowerEntry* tblBuf, uint8_t tblBufEntries, uint8_t* tableEntries );

/*! 
 *****************************************************************************
 * \brief  Dynamic power adjust
 *  
 * It measures the current output and adjusts the power accordingly to 
 * the dynamic power table  
 * 
 *****************************************************************************
 */
void rfalDynamicPowerAdjust( void );

/*! 
 *****************************************************************************
 * \brief  Dynamic power Enable
 *  
 * Enables the Dynamic power adjustment 
 * 
 *****************************************************************************
 */
void rfalDynamicPowerEnable( void );

/*! 
 *****************************************************************************
 * \brief  Dynamic power Disable
 *  
 * Disables the Dynamic power adjustment 
 * 
 *****************************************************************************
 */
void rfalDynamicPowerDisable( void );


#endif /* RFAL_DYNAMICPOWER_H */

/**
  * @}
  *
  * @}
  *
  * @}
  */
