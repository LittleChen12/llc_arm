#include "fdcan_fliter.h"

/**
 * @brief  FDCAN�˲������ú���
 * 
 * �˺�����������fdcan���չ�������Ŀǰ�������κ�id��ȫ������
 * 
 * @param  NULL
 */

FDCAN_FilterTypeDef sFilterConfig;
void FDCAN_Filter_Config(void)
{
    sFilterConfig.IdType = FDCAN_STANDARD_ID;            //���ñ�׼֡ȥ����
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;//ʹ��fifi0ȥ����
    sFilterConfig.FilterID1 = 0x00;                      //ȫ�����ղ�����
    sFilterConfig.FilterID2 = 0x00;												
    
    HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);
 
 
 
		//*******************����Ϊ��������**************************
	 
		//�ܾ�����ƥ�䲻�ɹ��ı�׼ID����չID,������Զ��֡
		HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,FDCAN_REJECT,FDCAN_REJECT,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE);
		 
		HAL_FDCAN_ConfigFifoWatermark(&hfdcan1,FDCAN_CFG_RX_FIFO0,0);
		HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
		HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_BUS_OFF, 0);
//		HAL_FDCAN_ConfigFifoWatermark(&hfdcan1,FDCAN_CFG_RX_FIFO1,1);
//		HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_COMPLETE, FDCAN_TX_BUFFER0); ���д����ǿ��������жϵģ������ϲ���Ҫ������
		 
//		HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_WATERMARK, 0);
//		HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_WATERMARK, 0);
//		HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_BUFFER_NEW_MESSAGE, 0);  ���д����ǿ���RX_BUFFER������յ����ݣ�RX_BUFFER��FIFO��ѡһ������ѡ��FIFO
		 

		HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, hfdcan1.Init.DataPrescaler * hfdcan1.Init.DataTimeSeg1, 0);
		HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1);
		
		if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)//����FDCAN
		{
				Error_Handler();
		}
		/***********************************************************************/
}

/**
 * @brief  FDCAN���պ���
 * 
 * �˺��������������
 * 
 * @param  *rec_id�����յ��ĵ��id�� *buf�����յ�������
 *
 * @retval ����ֵΪ���ݳ���
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
				
				return len;//�������ݵĳ���
		}
		return 0;        
}

/**
 * @brief  FDCAN���ջص�
 * 
 * �˺���������յ���ش���Ϣ
 * 
 * @param  NULL
 */
FDCAN_RxHeaderTypeDef RxHeader;
Motor_parameters_HandleTypeDef Motor[6];
uint8_t Read_datatest[32] = {0};
uint8_t read_data[32] = {0};
uint8_t date_len = 0; // date_len��Ϊ���⣬��Ҫ����Ϊȫ�ֱ�������Ȼ����ᱨδʹ�ñ�������
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    // ��� FIFO0 ���Ƿ�������Ϣ
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) // FIFO0�������ж�
    {
//				uint8_t read_data[29] = {0};
        // �������ݺͽ���ID
        uint16_t rec_id;
        uint8_t id, func;
        float* target_data;//�����ָ��ָ��ṹ���еľ������
				
        // ���ý��պ���
        date_len = fdcanx_receive(&hfdcan1, &rec_id, read_data);
				memcpy(Read_datatest,read_data,29);
        // ��ȡ��������id��Ϣ
        id = rec_id >> 7;
        func = rec_id & 0x007F; // rec_idΪ16λ

        Motor[id-motor_offset].function_code = func;
				
				// ���ݹ�������ȡ���ݵ�ָ��Motor�ṹ����
        switch (func)
        {
					  //ֻ��
            case 0x32: target_data = &Motor[id-motor_offset].actual_position; break;//ʵ��λ��
            case 0x33: target_data = &Motor[id-motor_offset].actual_speed; break;//ʵ���ٶ�
            case 0x34: target_data = &Motor[id-motor_offset].actual_torque; break;//ʵ��Ť��
            case 0x35: target_data = &Motor[id-motor_offset].output_power; break;//�������
            case 0x36: target_data = &Motor[id-motor_offset].follow_error; break;//�������
            case 0x37: target_data = &Motor[id-motor_offset].bus_voltage; break;//���ߵ�ѹ
            case 0x38: target_data = &Motor[id-motor_offset].battery_voltage; break;//��ص�ѹ
					  case 0x31: 
						{
							//С�˶�ȡҪ��λ
							Motor[id-motor_offset].state[0] = read_data[1];
							Motor[id-motor_offset].state[1] = read_data[0];
							Motor[id-motor_offset].state[2] = read_data[3];
							Motor[id-motor_offset].state[3] = read_data[2];	
							break;
						}
					  
					  //���������ͬʱҲҪ������
            case 0x2A://���ø���λ�ú������
						case 0x28://���ø����ٶȺ������
						case 0x25://��������λ�ú������
						case 0x23://���������ٶȺ������
            {
                uint8_t echo = read_data[0];
								uint8_t index = 1; // ָ��Ҫ��ȡ������
								for (int i = 0; i <= 6; i++) // ��0λ��ʼ����
								{
										if (echo & (1 << i))
										{
												switch (i)
												{
														case 6: 
															memcpy(&Motor[id-motor_offset].bus_voltage, read_data + index, 4);
															index += 4; // ��ȡһ��ƫ��4�ֽ�
															break;														
														case 5: 
															memcpy(&Motor[id-motor_offset].follow_error, read_data + index, 4);
															index += 4; // ��ȡһ��ƫ��4�ֽ�
															break;
														case 4: 
															memcpy(&Motor[id-motor_offset].output_power, read_data + index, 4);
															index += 4; // ��ȡһ��ƫ��4�ֽ�
															break;
														case 3: 
															memcpy(&Motor[id-motor_offset].actual_torque, read_data + index, 4);
														index += 4; // ��ȡһ��ƫ��4�ֽ�
															break;
														case 2: 
															memcpy(&Motor[id-motor_offset].actual_speed, read_data + index, 4);
															index += 4; // ��ȡһ��ƫ��4�ֽ�
															break;
														case 1: 
															memcpy(&Motor[id-motor_offset].actual_position, read_data + index, 4);
															index += 4; // ��ȡһ��ƫ��4�ֽ�
															break;
													  case 0: 
															Motor[id-motor_offset].state[0] = read_data[2];
															Motor[id-motor_offset].state[1] = read_data[1];
															Motor[id-motor_offset].state[2] = read_data[4];
															Motor[id-motor_offset].state[3] = read_data[3];		
															index += 4; // ��ȡһ��ƫ��4�ֽ�														
															break;
												}
										}
								}
                break;
            }
						
            default:
                // ����δ֪����������
                return;
        }
        
				//0x32-0x38������ִ��
				if (func >= 0x32 && func <= 0x38)
				{
						memcpy(target_data, read_data, 4);
				}
    }
}


 
 


