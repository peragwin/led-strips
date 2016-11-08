/**
  ******************************************************************************
  * File Name          : SPI.c
  * Description        : This file provides code for the configuration
  *                      of the SPI instances.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
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

/* Includes ------------------------------------------------------------------*/
#include "spi.h"

#include "gpio.h"
#include "dma.h"

#include "ControlRegisters.h"

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

uint8_t spiTxBuffer[SPI_BUFFSIZE];
uint8_t spiRxBuffer[SPI_BUFFSIZE];

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI2)
  {

    #ifdef PRINT_DEBUG
      printf("SPI got: ");
      for (int i = 0; i<SPI_BUFFSIZE; i++)
      {
        printf("%2x ", spiRxBuffer[i]);
        spiTxBuffer[i] = spiRxBuffer[i];
      }
      printf("\r\n");
    #endif

    int isDummy, i;
    uint16_t spiAddress;
    uint32_t spiValue;
    RawRegReadValue rv;
    HandleRegAccessFlag fv;

    #define SPI_CRC     0
    #define SPI_COMMAND 1
    #define SPI_FLAGS   1  // command and flags share field depending on rx/tx
    #define SPI_ADDRESS 2
    #define SPI_VALUE   4

    #define SPI_CRC_VALID 0 // no crc, just want this to be 0 for now

    // for FLAGS, msb is valid / invalid flag, the rest are types
    #define SPI_ISVALID       0     // no flags should be issued
    #define SPI_RX_INVALID    0x81  // rx (crc?) was invalid
    #define SPI_READ_VALID    0x04  // read command nominal
    #define SPI_READ_INVALID  0x84  // read command threw invalidAccess
    #define SPI_WRITE_VALID   0x08  // write command nominal
    #define SPI_WRITE_INVALID 0x88  // write command threw invalidAccess
    #define SPI_WRITE_READONLY 0x8c // write command threw readOnlyAccess

    #define SPI_READ  0
    #define SPI_WRITE 1

    isDummy = 1;
    for (i = 0; i < SPI_BUFFSIZE; i++)
      if (spiRxBuffer[i] != 0) { isDummy = 0; break; }

    if (isDummy) // no action
      for (i = 0; i < SPI_BUFFSIZE; i++) spiTxBuffer[i] = 0;

    else if (spiRxBuffer[SPI_CRC] == SPI_CRC_VALID) // todo crc?
    {
      spiAddress = (*(spiRxBuffer+SPI_ADDRESS) << 8) | *(spiRxBuffer+SPI_ADDRESS+1);
      switch (spiRxBuffer[SPI_COMMAND])
      {
        case (SPI_READ):
          RawRegisterRead(spiAddress, &rv);
          if (rv.flag != validAccess)
            spiTxBuffer[SPI_FLAGS] = SPI_READ_INVALID;
          else
          {
            spiTxBuffer[SPI_FLAGS] = SPI_READ_VALID;
            spiValue = rv.reg;
            for (i = 0; i < 4; i++)
              *(spiTxBuffer+SPI_VALUE+i) = (spiValue >> (8*(3-i))) % 256;
          }
          break;
      
        case (SPI_WRITE):
          spiValue = 0;
          for (i = 0; i < 4; i++) spiValue |= (*(spiRxBuffer+SPI_VALUE+i) << (8*(3-i)));
          fv = RawRegisterWrite(spiAddress, spiValue);
          if (fv == validAccess)
            spiTxBuffer[SPI_FLAGS] = SPI_WRITE_VALID;
          else if (fv == invalidAccess)
            spiTxBuffer[SPI_FLAGS] = SPI_WRITE_INVALID;
          else if (fv == readOnlyAccess)
            spiTxBuffer[SPI_FLAGS] = SPI_WRITE_READONLY;
          else
            printf("assert spi.c:116\r\n");
          break;

        default:
          spiTxBuffer[SPI_FLAGS] = SPI_RX_INVALID;
      }
    }
    else spiTxBuffer[SPI_FLAGS] = SPI_RX_INVALID;

    HAL_SPI_TransmitReceive_DMA(hspi,
      (uint8_t*)spiTxBuffer,
      (uint8_t*)spiRxBuffer,
      SPI_BUFFSIZE);
  }
}
 

/* SPI2 init function */
void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_SLAVE;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspInit 0 */

  /* USER CODE END SPI2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();
  
    /**SPI2 GPIO Configuration    
    PB10     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral DMA init*/
  
    hdma_spi2_rx.Instance = DMA1_Stream3;
    hdma_spi2_rx.Init.Channel = DMA_CHANNEL_0;
    hdma_spi2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi2_rx.Init.Mode = DMA_NORMAL;
    hdma_spi2_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_spi2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi2_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(spiHandle,hdmarx,hdma_spi2_rx);
    __HAL_DMA_ENABLE_IT(&hdma_spi2_rx, DMA_IT_TC);

    hdma_spi2_tx.Instance = DMA1_Stream4;
    hdma_spi2_tx.Init.Channel = DMA_CHANNEL_0;
    hdma_spi2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi2_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi2_tx.Init.Mode = DMA_NORMAL;
    hdma_spi2_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_spi2_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi2_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(spiHandle,hdmatx,hdma_spi2_tx);
    __HAL_DMA_ENABLE_IT(&hdma_spi2_tx, DMA_IT_TC);




  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspDeInit 0 */

  /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();
  
    /**SPI2 GPIO Configuration    
    PB10     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_14|GPIO_PIN_15);

    /* Peripheral DMA DeInit*/
    HAL_DMA_DeInit(spiHandle->hdmarx);
    HAL_DMA_DeInit(spiHandle->hdmatx);
  }
  /* USER CODE BEGIN SPI2_MspDeInit 1 */

  /* USER CODE END SPI2_MspDeInit 1 */
} 

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
