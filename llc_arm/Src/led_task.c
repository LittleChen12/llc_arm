#include "led_task.h"

/**
 * @brief  LED ����������
 * 
 * ��������ͨ���л� LED ���ŵĵ�ƽ��ʵ�� LED ����˸Ч����
 * ����������ѭ���ķ�ʽ���У�ÿ���л� LED ״̬����ӳ� 500 ���롣
 * 
 * @param  argument: �������Ĳ���ָ�롣�˲����ڸ�������δʹ�á�
 */

void Led_start(void *argument)
{
  for(;;)
  {
		HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
    osDelay(500);
  }
}



