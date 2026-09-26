#include "rc_task.h"

RC_t rc; // 定义遥控器数据结构体

void rc_task(void *argument)
{
    /* USER CODE BEGIN rc_task */
    RC_Init(); // 初始化遥控器接收
    /* Infinite loop */
    for (;;)
    {
        rc = RC_GetInfo(); // 获取遥控器数据结构体的快照
        vTaskDelay(50);         // 延时50ms
    }
    /* USER CODE END rc_task */
}
