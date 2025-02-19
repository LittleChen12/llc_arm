#include "led_task.h"

/**
 * @brief  LED 控制任务函数
 * 
 * 此任务函数通过切换 LED 引脚的电平来实现 LED 的闪烁效果。
 * 任务以无限循环的方式运行，每次切换 LED 状态后会延迟 500 毫秒。
 * 
 * @param  argument: 任务函数的参数指针。此参数在该任务中未使用。
 */

void Led_start(void *argument)
{
  for(;;)
  {
		HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
    osDelay(500);
  }
}



