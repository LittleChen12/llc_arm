#include "dstp_motor.h"

/**
 * @brief  FDCAN发送函数
 * 
 * 此函数负责发送控制信息
 * 
 * @param  id：电机id号  *data：发送数据指针  len：数据长度
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
					break; // 非法数据长度
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
 * @brief  电机原点设定
 * 
 * 此函数负责设定电机的原点 控制数据字节长度：0 功能码：0x05
 * 
 * @param  id：电机id号
 */
void Motor_origin_setting(uint16_t id)
{
		fdcanx_send_data(id,NULL,0,0x05);
}

/**
 * @brief  电机控制模式选择
 * 
 * 此函数负责设定电机的控制模式 控制数据字节长度：0 功能码：0x13
 * 
 * @param  id：电机id号 mode：0x01:轮廓转矩，0x02:轮廓速度，0x03:轮廓位置，0x04:转矩跟随，0x05:速度跟随，0x06:位置跟随
 */
void Motor_mode_setting(uint16_t id, uint8_t mode)
{
		uint8_t date[1]; 
		date[0] = mode;
		fdcanx_send_data(id,date,1,0x13);
}

/**
 * @brief  电机启动
 * 
 * 此函数负责启动电机 控制数据字节长度：0 功能码：0x14
 * 
 * @param  id：电机id号
 */
void Motor_start(uint16_t id)
{
		fdcanx_send_data(id,NULL,0,0x14);
}

/**
 * @brief  电机关闭
 * 
 * 此函数负责启动电机 控制数据字节长度：0 功能码：0x12
 * 
 * @param  id：电机id号
 */
void Motor_stop(uint16_t id)
{
		fdcanx_send_data(id,NULL,0,0x12);
}

/**
 * @brief  轮廓位置模式（采用驱动器内部的轮廓加速度和速度信息，去做规划）
 * 
 * 此函数负责电机的位置运动 控制数据字节长度：6 功能码：0x25
 * 
 * @param  id：电机id号   *data:数据指针
 */
 
void Contour_Position_Mode(uint16_t id,uint8_t echo, uint8_t sync, float targetPosition)
{
    uint8_t data[6];
    // 设置数据字段
    data[0] = echo;
    data[1] = sync;

    // 将 float32 转换为小端模式字节数组
    uint8_t* PositionBytes = (uint8_t*)&targetPosition;
    data[2] = PositionBytes[0];
    data[3] = PositionBytes[1];
    data[4] = PositionBytes[2];
    data[5] = PositionBytes[3];		
		
		//数据发送
		fdcanx_send_data(id,data,6,0x25);
}


/**
 * @brief  可变的轮廓位置模式
 * 
 * 此函数负责电机的指定位置运动 控制数据字节长度：20 功能码：0x26
 * 
 * @param  id：电机id号 echo:数据接收 sync：电机同步功能 
 */
void Variable_Contour_Position_Mode(uint16_t id,uint8_t echo, uint8_t sync, float targetPosition, float speed, float acc, float deceleration)
{
    uint8_t data[20];
    // 设置数据字段
    data[0] = echo;
    data[1] = sync;

    // 将 float32 转换为小端模式字节数组
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
 * @brief  跟随位置模式
 * 
 * 此函数负责电机的周期位置运动 控制数据字节长度：6 功能码：0x2a
 * 
 * @param  id：电机id号 echo:数据接收 sync：电机同步功能 
 */
void Follow_Position_Mode(uint16_t id,uint8_t echo, uint8_t sync, float Follow_Position)
{
    uint8_t data[6];
    // 设置数据字段
    data[0] = echo;
    data[1] = sync;

    // 将 float32 转换为小端模式字节数组
    uint8_t* PositionBytes = (uint8_t*)&Follow_Position;
    data[2] = PositionBytes[0];
    data[3] = PositionBytes[1];
    data[4] = PositionBytes[2];
    data[5] = PositionBytes[3];		
		
		//数据发送
		fdcanx_send_data(id,data,6,0x2a);
}

/**
 * @brief  清除错误(已经验证）
 * 
 * 此函数负责清楚错误 控制数据字节长度：0 功能码： 0x06
 * 
 * @param  id：电机id号 
 */
void Clear_Error(uint16_t id)
{
		fdcanx_send_data(id,NULL,0,0x06);
}

/**
 * @brief  快速停止（已经验证）
 * 
 * 此函数负责快速停止电机 控制数据字节长度：0 功能码： 0x11
 * 
 * @param  id：电机id号 
 */
void Quick_Stop_Motor(uint16_t id)
{
		fdcanx_send_data(id,NULL,0,0x11);
}

/**
 * @brief  多轴同步
 * 
 * 此函数负责启动电机的多轴同步 控制数据字节长度：0 功能码： 0x15
 * 
 * @param  无
 */
void Multi_Axis_Synchronization(void)
{
		fdcanx_send_data(0,NULL,0,0x15);
}

/**
 * @brief  设置轮廓转矩（已经验证）
 * 
 * 此函数负责设置电机转矩 控制数据字节长度：6 功能码： 0x21
 * 
 * @param  id：电机id号 echo:数据接收 sync：电机同步功能 Targe_Torque：目标转矩
 */
void Set_Contour_Torque(uint16_t id,uint8_t echo, uint8_t sync, float Targe_Torque)
{
	  uint8_t data[6];
    // 设置数据字段
    data[0] = echo;
    data[1] = sync;

    // 将 float32 转换为小端模式字节数组
    uint8_t* TorqueBytes = (uint8_t*)&Targe_Torque;
    data[2] = TorqueBytes[0];
    data[3] = TorqueBytes[1];
    data[4] = TorqueBytes[2];
    data[5] = TorqueBytes[3];		
		
		//数据发送
		fdcanx_send_data(id,data,6,0x21);
}

/**
 * @brief  设置轮廓转矩自定义（已经验证）
 * 
 * 此函数负责设置电机转矩和转矩的爬升速度 控制数据字节长度：12 功能码： 0x22
 * 
 * @param  id：电机id号 echo:数据接收 sync：电机同步功能 Targe_Torque：目标转矩 Targe_Speed：目标爬升速度
 */
void Set_Custom_Contour_Torque(uint16_t id,uint8_t echo, uint8_t sync, float Targe_Torque, float Targe_Speed)
{
	  uint8_t data[12];
    // 设置数据字段
    data[0] = echo;
    data[1] = sync;

    // 将 float32 转换为小端模式字节数组 目标转矩
    uint8_t* TorqueBytes = (uint8_t*)&Targe_Torque;
    data[2] = TorqueBytes[0];
    data[3] = TorqueBytes[1];
    data[4] = TorqueBytes[2];
    data[5] = TorqueBytes[3];		
	  
	  // 将 float32 转换为小端模式字节数组 目标爬升速度
	  uint8_t* SpeedBytes = (uint8_t*)&Targe_Speed;
		data[6] = SpeedBytes[0];
	  data[7] = SpeedBytes[1];
		data[8] = SpeedBytes[2];
		data[9] = SpeedBytes[3];
	  
		// 填入保留位，填充CAN-FD数据长度
		data[10] = 0;
		data[11] = 0;
		
		//数据发送
		fdcanx_send_data(id,data,12,0x22);
}

/**
 * @brief  设置跟随转矩
 * 
 * 此函数负责自定义电机的转矩曲线 控制数据字节长度：6 功能码： 0x27
 * 
 * @param  id：电机id号 echo:数据接收 sync：电机同步功能 Targe_Torque：目标转矩 
 */
void Set_Follow_Torque(uint16_t id,uint8_t echo,uint8_t sync,float Targe_Torque)
{
	  uint8_t data[6];
    // 设置数据字段
    data[0] = echo;
    data[1] = sync;

    // 将 float32 转换为小端模式字节数组
    uint8_t* TorqueBytes = (uint8_t*)&Targe_Torque;
    data[2] = TorqueBytes[0];
    data[3] = TorqueBytes[1];
    data[4] = TorqueBytes[2];
    data[5] = TorqueBytes[3];		
		
		//数据发送
		fdcanx_send_data(id,data,6,0x27);
}

/**
 * @brief  读取实际位置
 * 
 * 此函数负责读取电机实际位置 控制数据字节长度：0 功能码： 0x32
 * 
 * @param  id：电机id号  
 */
void Read_Actual_Position(uint16_t id)
{
		fdcanx_send_data(id,NULL,0,0x32);
}
	


