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

float test;
uint8_t date_len = 0;//date_len��Ϊ���⣬��Ҫ����Ϊȫ�ֱ�������Ȼ����ᱨδʹ�ñ�������
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    // ��� FIFO0 ���Ƿ�������Ϣ
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)    /* FIFO1�������ж� */
    {
			//ֻ�������ݲŴ�����������ʡ�ڴ棬ͬʱ������ʹ����ɺ��Զ�����
			uint8_t read_data[30] = {0};
			uint16_t rec_id;
			uint8_t id;
			uint8_t func;
			//���ý��պ���
			date_len = fdcanx_receive(&hfdcan1, &rec_id, read_data);
      
			//��ȡ��������id��Ϣ
			id = rec_id >> 7;
			func = rec_id & 0x007F;//rec_idΪ16λ
			
			Motor[id].function_code = func;
			
			//���ݲ�ͬ��������ȡ���ݵ�ָ��Motor�ṹ����
			switch(Motor[id].function_code)
			{
				case 0x32://��ȡʵ��λ��
				{
					uint8_t Positionbyte[] = {read_data[0], read_data[1], read_data[2], read_data[3]};
					Motor[id].actual_position = *((float*)Positionbyte);
					break;
				}
				case 0x33://��ȡʵ���ٶ�
				{
					break;
				}
				case 0x34://��ȡʵ��ת��
				{
					break;
				}
				case 0x35://��ȡ�������
				{
					break;
				}
				case 0x36://��ȡ�������
				{
					break;
				}
				case 0x37://��ȡ���ߵ�ѹ
				{
					break;
				}
				case 0x38://��ȡ��ص�ѹ
				{
					break;
				}
			}
			
//        // ������Ϣ
//        if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
//        {

//						// ���� RxData �����ݵ� Rx_data 
//					  memcpy(Rx_data, RxData, sizeof(RxData)); 
//					 // ��� RxData ���� 
//					  memset(RxData, 0, sizeof(RxData));
//            // �����ϢID��DLC
//            if ((RxHeader.Identifier == ((1 << 7) | 0x05)) && (RxHeader.DataLength == FDCAN_DLC_BYTES_1))
//            {
//                // ����ִ�н��
//                switch (RxData[0])
//                {
//                    case 0x00:
//												printf("SUCCESS!");
//                        // ����ִ��
//                        break;
//                    case 0x01:
//												printf("FILED!");
//                        // �쳣ִ��
//                        break;
//                    default:
//                        // δ֪���
//                        break;
//                }
//            }
//        }
      }

}

