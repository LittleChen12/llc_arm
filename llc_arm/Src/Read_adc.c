#include "Read_adc.h"

float Read_ADC_Data(void)
{
    volatile uint16_t portB_val;
    volatile uint16_t portD_val;

		portB_val = GPIOB->IDR;  // 16 位数据，对应 PB15...PB0
		portD_val = GPIOD->IDR;  // 16 位数据，对应 PD15...PD0，其中 PD8 是我们需要的

		uint16_t adc_value = 0;
		float Draw_data;
		adc_value |= (((portD_val >> 8) & 0x01) << 13); // PD8 → Bit13
		adc_value |= (((portB_val >> 15) & 0x01) << 12); // PB15 → Bit12
		adc_value |= (((portB_val >> 14) & 0x01) << 11); // PB14 → Bit11
		adc_value |= (((portB_val >> 10) & 0x01) << 10); // PB10 → Bit10
		adc_value |= (((portB_val >> 9)  & 0x01) << 9);  // PB9  → Bit9
		adc_value |= (((portB_val >> 8)  & 0x01) << 8);  // PB8  → Bit8
		adc_value |= (((portB_val >> 7)  & 0x01) << 7);  // PB7  → Bit7
		adc_value |= (((portB_val >> 6)  & 0x01) << 6);  // PB6  → Bit6
		adc_value |= (((portB_val >> 5)  & 0x01) << 5);  // PB5  → Bit5
		adc_value |= (((portB_val >> 4)  & 0x01) << 4);  // PB4  → Bit4
		adc_value |= (((portB_val >> 3)  & 0x01) << 3);  // PB3  → Bit3
		adc_value |= (((portB_val >> 2)  & 0x01) << 2);  // PB2  → Bit2
		adc_value |= (((portB_val >> 1)  & 0x01) << 1);  // PB1  → Bit1
		adc_value |= (((portB_val >> 0)  & 0x01) << 0);  // PB0  → Bit0
		Draw_data = (double)((((16384.0f - adc_value) /16384.0f)*10000.0f) - 5000.0f);
		return Draw_data;
}
