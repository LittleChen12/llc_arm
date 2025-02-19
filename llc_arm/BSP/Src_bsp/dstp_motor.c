#include "dstp_motor.h"

/**
 * @brief  FDCAN���ͺ���
 * 
 * �˺��������Ϳ�����Ϣ
 * 
 * @param  id�����id��  *data����������ָ��  len�����ݳ���
 */
void fdcanx_send_data(uint16_t id, uint8_t *data, uint32_t len, uint8_t func_code)
{	
    FDCAN_TxHeaderTypeDef pTxHeader;
    pTxHeader.Identifier=(uint32_t)((id << 7) | func_code);
    pTxHeader.IdType=FDCAN_STANDARD_ID;
    pTxHeader.TxFrameType=FDCAN_DATA_FRAME;
    switch (len)
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
        default: 
					break; // �Ƿ����ݳ���
    }
    pTxHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
    pTxHeader.BitRateSwitch=FDCAN_BRS_ON;
    pTxHeader.FDFormat=FDCAN_FD_CAN;
    pTxHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
    pTxHeader.MessageMarker=0;
 
		if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &pTxHeader, data)!=HAL_OK) 
		{
			Error_Handler();
		}
}



/**
 * @brief  ���ԭ���趨
 * 
 * �˺��������趨�����ԭ�� ���������ֽڳ��ȣ�0 �����룺0x05
 * 
 * @param  id�����id��
 */
void Motor_origin_setting(uint16_t id)
{
		fdcanx_send_data(id,NULL,0,0x05);
}

/**
 * @brief  �������ģʽѡ��
 * 
 * �˺��������趨����Ŀ���ģʽ ���������ֽڳ��ȣ�0 �����룺0x13
 * 
 * @param  id�����id�� mode��0x01:����ת�أ�0x02:�����ٶȣ�0x03:����λ�ã�0x04:ת�ظ��棬0x05:�ٶȸ��棬0x06:λ�ø���
 */
void Motor_mode_setting(uint16_t id, uint8_t mode)
{
		uint8_t date[1]; 
		date[0] = mode;
		fdcanx_send_data(id,date,1,0x13);
}

/**
 * @brief  �������
 * 
 * �˺�������������� ���������ֽڳ��ȣ�0 �����룺0x14
 * 
 * @param  id�����id��
 */
void Motor_start(uint16_t id)
{
		fdcanx_send_data(id,NULL,0,0x14);
}

/**
 * @brief  ����ر�
 * 
 * �˺�������������� ���������ֽڳ��ȣ�0 �����룺0x12
 * 
 * @param  id�����id��
 */
void Motor_stop(uint16_t id)
{
		fdcanx_send_data(id,NULL,0,0x12);
}

/**
 * @brief  ����λ��ģʽ�������������ڲ����������ٶȺ��ٶ���Ϣ��ȥ���滮��
 * 
 * �˺�����������λ���˶� ���������ֽڳ��ȣ�6 �����룺0x25
 * 
 * @param  id�����id��   *data:����ָ��
 */
 
void Contour_Position_Mode(uint16_t id,uint8_t echo, uint8_t sync, float targetPosition)
{
    uint8_t data[6];
    // ���������ֶ�
    data[0] = echo;
    data[1] = sync;

    // �� float32 ת��ΪС��ģʽ�ֽ�����
    uint8_t* PositionBytes = (uint8_t*)&targetPosition;
    data[2] = PositionBytes[0];
    data[3] = PositionBytes[1];
    data[4] = PositionBytes[2];
    data[5] = PositionBytes[3];		
		
		//���ݷ���
		fdcanx_send_data(id,data,6,0x25);
}


/**
 * @brief  �ɱ������λ��ģʽ
 * 
 * �˺�����������ָ��λ���˶� ���������ֽڳ��ȣ�20 �����룺0x26
 * 
 * @param  id�����id�� echo:���ݽ��� sync�����ͬ������ 
 */
void Variable_Contour_Position_Mode(uint16_t id,uint8_t echo, uint8_t sync, float targetPosition, float speed, float acc, float deceleration)
{
    uint8_t data[20];
    // ���������ֶ�
    data[0] = echo;
    data[1] = sync;

    // �� float32 ת��ΪС��ģʽ�ֽ�����
    uint8_t* PositionBytes = (uint8_t*)&targetPosition;
    data[2] = PositionBytes[0];
    data[3] = PositionBytes[1];
    data[4] = PositionBytes[2];
    data[5] = PositionBytes[3];		
	
    uint8_t* Speed_Bytes = (uint8_t*)&speed;
    data[6] = Speed_Bytes[0];
    data[7] = Speed_Bytes[1];
    data[8] = Speed_Bytes[2];
    data[9] = Speed_Bytes[3];	
	
    uint8_t* Acc_Bytes = (uint8_t*)&acc;
    data[10] = Acc_Bytes[0];
    data[11] = Acc_Bytes[1];
    data[12] = Acc_Bytes[2];
    data[13] = Acc_Bytes[3];	

    uint8_t* Deceleration_Bytes = (uint8_t*)&deceleration;
    data[14] = Deceleration_Bytes[0];
    data[15] = Deceleration_Bytes[1];
    data[16] = Deceleration_Bytes[2];
    data[17] = Deceleration_Bytes[3];	
	
		fdcanx_send_data(id,data,20,0x26);
}

/**
 * @brief  ����λ��ģʽ
 * 
 * �˺���������������λ���˶� ���������ֽڳ��ȣ�6 �����룺0x2a
 * 
 * @param  id�����id�� echo:���ݽ��� sync�����ͬ������ 
 */
void Follow_Position_Mode(uint16_t id,uint8_t echo, uint8_t sync, float Follow_Position)
{
    uint8_t data[6];
    // ���������ֶ�
    data[0] = echo;
    data[1] = sync;

    // �� float32 ת��ΪС��ģʽ�ֽ�����
    uint8_t* PositionBytes = (uint8_t*)&Follow_Position;
    data[2] = PositionBytes[0];
    data[3] = PositionBytes[1];
    data[4] = PositionBytes[2];
    data[5] = PositionBytes[3];		
		
		//���ݷ���
		fdcanx_send_data(id,data,6,0x2a);
}

/**
 * @brief  �������(�Ѿ���֤��
 * 
 * �˺�������������� ���������ֽڳ��ȣ�0 �����룺 0x06
 * 
 * @param  id�����id�� 
 */
void Clear_Error(uint16_t id)
{
		fdcanx_send_data(id,NULL,0,0x06);
}

/**
 * @brief  ����ֹͣ���Ѿ���֤��
 * 
 * �˺����������ֹͣ��� ���������ֽڳ��ȣ�0 �����룺 0x11
 * 
 * @param  id�����id�� 
 */
void Quick_Stop_Motor(uint16_t id)
{
		fdcanx_send_data(id,NULL,0,0x11);
}

/**
 * @brief  ����ͬ��
 * 
 * �˺���������������Ķ���ͬ�� ���������ֽڳ��ȣ�0 �����룺 0x15
 * 
 * @param  ��
 */
void Multi_Axis_Synchronization(void)
{
		fdcanx_send_data(0,NULL,0,0x15);
}

/**
 * @brief  ��������ת�أ��Ѿ���֤��
 * 
 * �˺����������õ��ת�� ���������ֽڳ��ȣ�6 �����룺 0x21
 * 
 * @param  id�����id�� echo:���ݽ��� sync�����ͬ������ Targe_Torque��Ŀ��ת��
 */
void Set_Contour_Torque(uint16_t id,uint8_t echo, uint8_t sync, float Targe_Torque)
{
	  uint8_t data[6];
    // ���������ֶ�
    data[0] = echo;
    data[1] = sync;

    // �� float32 ת��ΪС��ģʽ�ֽ�����
    uint8_t* TorqueBytes = (uint8_t*)&Targe_Torque;
    data[2] = TorqueBytes[0];
    data[3] = TorqueBytes[1];
    data[4] = TorqueBytes[2];
    data[5] = TorqueBytes[3];		
		
		//���ݷ���
		fdcanx_send_data(id,data,6,0x21);
}

/**
 * @brief  ��������ת���Զ��壨�Ѿ���֤��
 * 
 * �˺����������õ��ת�غ�ת�ص������ٶ� ���������ֽڳ��ȣ�12 �����룺 0x22
 * 
 * @param  id�����id�� echo:���ݽ��� sync�����ͬ������ Targe_Torque��Ŀ��ת�� Targe_Speed��Ŀ�������ٶ�
 */
void Set_Custom_Contour_Torque(uint16_t id,uint8_t echo, uint8_t sync, float Targe_Torque, float Targe_Speed)
{
	  uint8_t data[12];
    // ���������ֶ�
    data[0] = echo;
    data[1] = sync;

    // �� float32 ת��ΪС��ģʽ�ֽ����� Ŀ��ת��
    uint8_t* TorqueBytes = (uint8_t*)&Targe_Torque;
    data[2] = TorqueBytes[0];
    data[3] = TorqueBytes[1];
    data[4] = TorqueBytes[2];
    data[5] = TorqueBytes[3];		
	  
	  // �� float32 ת��ΪС��ģʽ�ֽ����� Ŀ�������ٶ�
	  uint8_t* SpeedBytes = (uint8_t*)&Targe_Speed;
		data[6] = SpeedBytes[0];
	  data[7] = SpeedBytes[1];
		data[8] = SpeedBytes[2];
		data[9] = SpeedBytes[3];
	  
		// ���뱣��λ�����CAN-FD���ݳ���
		data[10] = 0;
		data[11] = 0;
		
		//���ݷ���
		fdcanx_send_data(id,data,12,0x22);
}

/**
 * @brief  ���ø���ת��
 * 
 * �˺��������Զ�������ת������ ���������ֽڳ��ȣ�6 �����룺 0x27
 * 
 * @param  id�����id�� echo:���ݽ��� sync�����ͬ������ Targe_Torque��Ŀ��ת�� 
 */
void Set_Follow_Torque(uint16_t id,uint8_t echo,uint8_t sync,float Targe_Torque)
{
	  uint8_t data[6];
    // ���������ֶ�
    data[0] = echo;
    data[1] = sync;

    // �� float32 ת��ΪС��ģʽ�ֽ�����
    uint8_t* TorqueBytes = (uint8_t*)&Targe_Torque;
    data[2] = TorqueBytes[0];
    data[3] = TorqueBytes[1];
    data[4] = TorqueBytes[2];
    data[5] = TorqueBytes[3];		
		
		//���ݷ���
		fdcanx_send_data(id,data,6,0x27);
}

/**
 * @brief  ��ȡʵ��λ��
 * 
 * �˺��������ȡ���ʵ��λ�� ���������ֽڳ��ȣ�0 �����룺 0x32
 * 
 * @param  id�����id��  
 */
void Read_Actual_Position(uint16_t id)
{
		fdcanx_send_data(id,NULL,0,0x32);
}
	


