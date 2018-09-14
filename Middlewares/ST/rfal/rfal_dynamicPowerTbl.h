
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


#ifndef ST25R3911_DYNAMICPOWER_H
#define ST25R3911_DYNAMICPOWER_H

/*
 ******************************************************************************
 * INCLUDES
 ******************************************************************************
 */
#include "rfal_dynamicPower.h"


/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

const uint8_t rfalDynamicPowerDefaultSettings [] = {
                0x00, 255, 200,
                0x01, 190, 170,
                0x02, 160, 140,
                0x03, 130, 110,
                0x0E,  80, 0
};

#endif /* ST25R3911_DYNAMICPOWER_H */
