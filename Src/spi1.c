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
 *  \brief SPI communication handling implementation.
 *
 */
 
/* Includes ------------------------------------------------------------------*/

#include "spi1.h"
#include "st_errno.h"
#include "utils.h"

#define SPI_TIMEOUT   1000

SPI_HandleTypeDef *pSpi = 0;

/**
  * @brief  This function initalize the SPI handle.
	* @param	hspi : already initalized handle to SPI HW
  * @retval none :
  */
void SpiInit(SPI_HandleTypeDef *hspi)
{
    pSpi = hspi;
}

/**
  * @brief  This function Transmit and Reveice data via SPI
	* @param	txData : pointer to data that shall be transmitted
	* @param	rxData : pointer to data holding the buffer where received data shall be copied to.
	* @param	length : length of data to transmit
  * @retval ERR_INVALID_HANDLE : in case the SPI HW is not initalized yet
  * @retval others : see HAL error codes
  */
uint8_t SpiTxRx(const uint8_t *txData, uint8_t *rxData, uint8_t length)
{  
  if(pSpi == 0)
    return ERR_INVALID_HANDLE;

  uint8_t   tx[256];
  uint8_t   rx[256];
  if(txData != NULL){
    ST_MEMCPY(tx, txData, length);
  }
  return HAL_SPI_TransmitReceive(pSpi, tx, (rxData != NULL) ? rxData : rx, length, SPI_TIMEOUT);
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
