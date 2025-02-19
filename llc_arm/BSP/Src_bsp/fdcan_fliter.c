#include "fdcan_fliter.h"

/**
 * @brief  FDCAN滤波器配置函数
 * 
 * 此函数负责配置fdcan接收过滤器，目前不过滤任何id，全部接收
 * 
 * @param  NULL
 */

FDCAN_FilterTypeDef sFilterConfig;
void FDCAN_Filter_Config(void)
{
    sFilterConfig.IdType = FDCAN_STANDARD_ID;            //采用标准帧去接收
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;//使用fifi0去接收
    sFilterConfig.FilterID1 = 0x00;                      //全部接收不过滤
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

/**
 * @brief  FDCAN接收函数
 * 
 * 此函数负责接收数据
 * 
 * @param  *rec_id：接收到的电机id号 *buf：接收到的数据
 *
 * @retval 返回值为数据长度
 */
uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint16_t *rec_id, uint8_t *buf)
{        
		FDCAN_RxHeaderTypeDef pRxHeader;
		uint8_t len;
		
		if(HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0, &pRxHeader, buf)==HAL_OK)
		{
				*rec_id = pRxHeader.Identifier;
				if(pRxHeader.DataLength<=FDCAN_DLC_BYTES_8)
								len = pRxHeader.DataLength;
				else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_12)
								len = 12;
				else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_16)
								len = 16;
				else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_20)
								len = 20;
				else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_24)
								len = 24;
				else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_32)
								len = 32;
				else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_48)
								len = 48;
				else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_64)
								len = 64;
				
				return len;//接收数据的长度
		}
		return 0;        
}

/**
 * @brief  FDCAN接收回调
 * 
 * 此函数负责接收电机回传消息
 * 
 * @param  NULL
 */
FDCAN_RxHeaderTypeDef RxHeader;
Motor_parameters_HandleTypeDef Motor[6];

float test;
uint8_t date_len = 0;//date_len较为特殊，需要定义为全局变量，不然编译会报未使用变量警告
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    // 检查 FIFO0 中是否有新消息
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)    /* FIFO1新数据中断 */
    {
			//只有有数据才创建变量，节省内存，同时可以在使用完成后，自动销毁
			uint8_t read_data[30] = {0};
			uint16_t rec_id;
			uint8_t id;
			uint8_t func;
			//调用接收函数
			date_len = fdcanx_receive(&hfdcan1, &rec_id, read_data);
      
			//提取功能码与id信息
			id = rec_id >> 7;
			func = rec_id & 0x007F;//rec_id为16位
			
			Motor[id].function_code = func;
			
			//根据不同功能码提取数据到指定Motor结构体中
			switch(Motor[id].function_code)
			{
				case 0x32://读取实际位置
				{
					uint8_t Positionbyte[] = {read_data[0], read_data[1], read_data[2], read_data[3]};
					Motor[id].actual_position = *((float*)Positionbyte);
					break;
				}
				case 0x33://读取实际速度
				{
					break;
				}
				case 0x34://读取实际转矩
				{
					break;
				}
				case 0x35://读取输出功率
				{
					break;
				}
				case 0x36://读取跟随误差
				{
					break;
				}
				case 0x37://读取总线电压
				{
					break;
				}
				case 0x38://读取电池电压
				{
					break;
				}
			}
			
//        // 接收消息
//        if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
//        {

//						// 复制 RxData 的数据到 Rx_data 
//					  memcpy(Rx_data, RxData, sizeof(RxData)); 
//					 // 清空 RxData 数组 
//					  memset(RxData, 0, sizeof(RxData));
//            // 检查消息ID和DLC
//            if ((RxHeader.Identifier == ((1 << 7) | 0x05)) && (RxHeader.DataLength == FDCAN_DLC_BYTES_1))
//            {
//                // 处理执行结果
//                switch (RxData[0])
//                {
//                    case 0x00:
//												printf("SUCCESS!");
//                        // 正常执行
//                        break;
//                    case 0x01:
//												printf("FILED!");
//                        // 异常执行
//                        break;
//                    default:
//                        // 未知结果
//                        break;
//                }
//            }
//        }
      }

}

