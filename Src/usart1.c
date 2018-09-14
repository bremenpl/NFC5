/**
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  * 1. Redistributions of source code must retain the above copyright notice,
  * this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  * this list of conditions and the following disclaimer in the documentation
  * and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of its contributors
  * may be used to endorse or promote products derived from this software
  * without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
*/

/*! \file
 *
 *  \author 
 *
 *  \brief UART communication handling implementation.
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "usart1.h"
#include "st_errno.h"

#define USART_TIMEOUT          1000

UART_HandleTypeDef *pUsart = 0;

/**
  * @brief  This function initalize the UART handle.
	* @param	husart : already initalized handle to USART HW
  * @retval none :
  */
void UsartInit(UART_HandleTypeDef *husart)
{
    pUsart = husart;
}

/**
  * @brief  This function Transmit one data byte via USART
	* @param	data : data to be transmitted
  * @retval ERR_INVALID_HANDLE : in case the SPI HW is not initalized yet
  * @retval others : HAL status
  */
uint8_t UsartTxByte(uint8_t data)
{
  if(pUsart == 0)
    return ERR_INVALID_HANDLE;

  return HAL_UART_Transmit(pUsart, &data, 1, USART_TIMEOUT);
}

/**
  * @brief  This function Transmit data via USART
	* @param	data : data to be transmitted
	* @param	dataLen : length of data to be transmitted
  * @retval ERR_INVALID_HANDLE : in case the SPI HW is not initalized yet
  * @retval others : HAL status
  */
uint8_t UsartTx(uint8_t *data, uint16_t dataLen)
{
  if(pUsart == 0)
    return ERR_INVALID_HANDLE;

  return HAL_UART_Transmit(pUsart, data, dataLen, USART_TIMEOUT);
}

/**
  * @brief  This function Receive data via USART
	* @param	data : data where received data shall be stored
	* @param	dataLen : length of received data
  * @retval ERR_INVALID_HANDLE : in case the SPI HW is not initalized yet
  * @retval others : HAL status
  */
uint8_t UsartRx(uint8_t *data, uint16_t *dataLen)
{
  uint8_t err = ERR_NONE;
  
  if(pUsart == 0)
    return ERR_INVALID_HANDLE;

  for(uint8_t i = 0; i < *dataLen; i++) {
    err |= HAL_UART_Receive(pUsart, &data[i], 1, USART_TIMEOUT);
    if(data[i] == 0) {
      *dataLen = i;
    }
  }
  return err;
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
