/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   This file provides code for the configuration
  *          of the FDCAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "fdcan.h"

/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan1;

/* FDCAN1 init function */
void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = ENABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 20;
  hfdcan1.Init.NominalTimeSeg1 = 59;
  hfdcan1.Init.NominalTimeSeg2 = 20;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 4;
  hfdcan1.Init.DataTimeSeg1 = 11;
  hfdcan1.Init.DataTimeSeg2 = 4;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 64;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 32;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_20;
  hfdcan1.Init.RxFifo1ElmtsNbr = 8;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_20;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 32;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_20;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspInit 0 */

  /* USER CODE END FDCAN1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInitStruct.PLL2.PLL2M = 25;
    PeriphClkInitStruct.PLL2.PLL2N = 160;
    PeriphClkInitStruct.PLL2.PLL2P = 2;
    PeriphClkInitStruct.PLL2.PLL2Q = 2;
    PeriphClkInitStruct.PLL2.PLL2R = 2;
    PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;
    PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
    PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
    PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN1 clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration
    PD0     ------> FDCAN1_RX
    PD1     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* FDCAN1 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    HAL_NVIC_SetPriority(FDCAN1_IT1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT1_IRQn);
  /* USER CODE BEGIN FDCAN1_MspInit 1 */

  /* USER CODE END FDCAN1_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

  /* USER CODE END FDCAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_FDCAN_CLK_DISABLE();

    /**FDCAN1 GPIO Configuration
    PD0     ------> FDCAN1_RX
    PD1     ------> FDCAN1_TX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

    /* FDCAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
    HAL_NVIC_DisableIRQ(FDCAN1_IT1_IRQn);
  /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

  /* USER CODE END FDCAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
FDCAN_FilterTypeDef sFilterConfig;

void FDCAN_Filter_Config(void)
{
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x00;
    sFilterConfig.FilterID2 = 0x00;
    
    HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);
 
 
 
		//*******************下面为其他配置**************************
	 
		//拒绝接收匹配不成功的标准ID和扩展ID,不接受远程帧
		HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,FDCAN_REJECT,FDCAN_REJECT,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE);
		 
		HAL_FDCAN_ConfigFifoWatermark(&hfdcan1,FDCAN_CFG_RX_FIFO0,0);
		HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
		HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_BUS_OFF, 0);
//		HAL_FDCAN_ConfigFifoWatermark(&hfdcan1,FDCAN_CFG_RX_FIFO1,1);
//		HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_COMPLETE, FDCAN_TX_BUFFER0); 这行代码是开启发送中断的，功能上不需要，屏蔽
		 
//		HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_WATERMARK, 0);
//		HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_WATERMARK, 0);
//		HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_BUFFER_NEW_MESSAGE, 0);  这行代码是开启RX_BUFFER管理接收的数据，RX_BUFFER和FIFO二选一，这里选择FIFO
		 

		HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, hfdcan1.Init.DataPrescaler * hfdcan1.Init.DataTimeSeg1, 0);
		HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1);
		
		if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)//启动FDCAN
		{
				Error_Handler();
		}
		/***********************************************************************/
	 
 
}

uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{	
    FDCAN_TxHeaderTypeDef pTxHeader;
    pTxHeader.Identifier=(uint32_t)((id << 7) | 0x05);
    pTxHeader.IdType=FDCAN_STANDARD_ID;
    pTxHeader.TxFrameType=FDCAN_DATA_FRAME;
	
	if(len<=8)
		pTxHeader.DataLength = len;
	if(len==12)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_12;
	if(len==16)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_16;
	if(len==20)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_20;
	if(len==24)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_24;
	if(len==32)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_32;
	if(len==48)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_48;
	if(len==64)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_64;
	
    pTxHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
    pTxHeader.BitRateSwitch=FDCAN_BRS_ON;
    pTxHeader.FDFormat=FDCAN_FD_CAN;
    pTxHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
    pTxHeader.MessageMarker=0;
 
	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, data)!=HAL_OK) 
		return 1;//发送
	return 0;	
}

int flag;
void FDCAN_Send_Packet(uint8_t id, uint8_t* data, uint8_t length)
{
    FDCAN_TxHeaderTypeDef pTxHeader = {0};
    
    uint32_t Identifier = (uint32_t)((id << 7) | 0x14);
    pTxHeader.Identifier = Identifier;
    pTxHeader.IdType = FDCAN_STANDARD_ID;
    pTxHeader.TxFrameType = FDCAN_DATA_FRAME;

    // 根据数据长度设置 DataLength
    switch (length)
    {
        case 0: pTxHeader.DataLength = FDCAN_DLC_BYTES_0; break;
        case 1: pTxHeader.DataLength = FDCAN_DLC_BYTES_1; break;
        case 2: pTxHeader.DataLength = FDCAN_DLC_BYTES_2; break;
        case 3: pTxHeader.DataLength = FDCAN_DLC_BYTES_3; break;
        case 4: pTxHeader.DataLength = FDCAN_DLC_BYTES_4; break;
        case 5: pTxHeader.DataLength = FDCAN_DLC_BYTES_5; break;
        case 6: pTxHeader.DataLength = FDCAN_DLC_BYTES_6; break;
        case 7: pTxHeader.DataLength = FDCAN_DLC_BYTES_7; break;
        case 8: pTxHeader.DataLength = FDCAN_DLC_BYTES_8; break;
        case 12: pTxHeader.DataLength = FDCAN_DLC_BYTES_12; break;
        case 16: pTxHeader.DataLength = FDCAN_DLC_BYTES_16; break;
        case 20: pTxHeader.DataLength = FDCAN_DLC_BYTES_20; break;
        case 24: pTxHeader.DataLength = FDCAN_DLC_BYTES_24; break;
        case 32: pTxHeader.DataLength = FDCAN_DLC_BYTES_32; break;
        case 48: pTxHeader.DataLength = FDCAN_DLC_BYTES_48; break;
        case 64: pTxHeader.DataLength = FDCAN_DLC_BYTES_64; break;
        default: return; // 非法数据长度
    }

    pTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    pTxHeader.BitRateSwitch = FDCAN_BRS_ON;
    pTxHeader.FDFormat = FDCAN_FD_CAN;
    pTxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    pTxHeader.MessageMarker = 0;

		if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &pTxHeader, data)!=HAL_OK) 
		{
			flag = 1;
		}
}

void FDCAN_Send_MOOD(uint8_t id, uint8_t* data)
{
    FDCAN_TxHeaderTypeDef pTxHeader = {0};
    
    uint32_t Identifier = (uint32_t)((id << 7) | 0x13);
    pTxHeader.Identifier = Identifier;
    pTxHeader.IdType = FDCAN_STANDARD_ID;
    pTxHeader.TxFrameType = FDCAN_DATA_FRAME;
		pTxHeader.DataLength = FDCAN_DLC_BYTES_1;
    pTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    pTxHeader.BitRateSwitch = FDCAN_BRS_ON;
    pTxHeader.FDFormat = FDCAN_FD_CAN;
    pTxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    pTxHeader.MessageMarker = 0;

		if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &pTxHeader, data)!=HAL_OK) 
		{
			flag = 1;
		}
}


// 协议发送函数
void FDCAN_Send_TargetSpeed(uint8_t nodeId, uint8_t echo, uint8_t sync, float targetSpeed)
{
    FDCAN_TxHeaderTypeDef pTxHeader = {0};
    uint8_t data[6];

    // 设置消息 ID
    uint32_t Identifier = (uint32_t)((nodeId << 7) | 0x22);
    pTxHeader.Identifier = Identifier;
    pTxHeader.IdType = FDCAN_STANDARD_ID;
    pTxHeader.TxFrameType = FDCAN_DATA_FRAME;
    pTxHeader.DataLength = FDCAN_DLC_BYTES_6;
    pTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    pTxHeader.BitRateSwitch = FDCAN_BRS_ON;
    pTxHeader.FDFormat = FDCAN_FD_CAN;
    pTxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    pTxHeader.MessageMarker = 0;

    // 设置数据字段
    data[0] = echo;
    data[1] = sync;

    // 将 float32 转换为小端模式字节数组
    uint8_t* speedBytes = (uint8_t*)&targetSpeed;
    data[2] = speedBytes[0];
    data[3] = speedBytes[1];
    data[4] = speedBytes[2];
    data[5] = speedBytes[3];

		
    // 发送消息
		if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &pTxHeader, data)!=HAL_OK) 
		{
			flag = 1;
		}
}



/* USER CODE END 1 */
