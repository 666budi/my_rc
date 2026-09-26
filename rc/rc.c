/**
 * @file rc.c
 * @author 小佛
 * @brief  16通道SBUS/DBUS遥控器数据解析(目前DBUS解析适配DT7)
 * @version 1.0
 * @date    2026-07-5
 * @note    ASCII 流程图:
 * @code
 * @endcode
 */
#include "rc.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

static UART_HandleTypeDef *RC_UART_Ptr = &huart1; // 定义RC_UART的指针
uint8_t RC_buffer[DMA_MAX_LEN] = {0};             // 定义DBUS数据缓存区，初始化为0
RC_t RC_data[2] = {0};                            // 定义遥控器数据结构体

/**
 * @brief  获取DMA当前接收的数据长度
 * @param  dma_stream: 指向DMA_Stream_TypeDef结构体的指针，表示要查询的DMA流
 * @retval 当前接收的数据长度
 */
static HAL_StatusTypeDef uart_receive_dma_no_it(UART_HandleTypeDef *huart, uint8_t *pData, uint32_t Size)
{
    // 检查传入的参数是否有效，包括UART句柄、数据指针、数据长度以及DMA句柄是否为空
    if ((huart == NULL) || (pData == NULL) || (Size == 0U) || (huart->hdmarx == NULL))
    {
        return HAL_ERROR;
    }
    // 判断UART是否处于就绪状态
    if (huart->RxState == HAL_UART_STATE_READY)
    {
        // 设置UART的接收缓冲区指针、接收数据长度和接收计数器
        huart->pRxBuffPtr = pData;
        huart->RxXferSize = Size;
        huart->RxXferCount = Size;
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        huart->RxState = HAL_UART_STATE_BUSY_RX;

        // 在配置 DMA 前先关闭 DMAR，可以避免 DMA 配置过程中被 UART 数据打断。
        CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);

        // 启动DMA接收，源地址为UART数据寄存器，目标地址为接收缓冲区
        if (HAL_DMA_Start(huart->hdmarx,
                          (uint32_t)&huart->Instance->DR,
                          (uint32_t)pData,
                          Size) != HAL_OK)
        {
            huart->RxState = HAL_UART_STATE_READY;
            return HAL_ERROR;
        }

        // 使能UART的DMA接收功能
        SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
        return HAL_OK;
    }
    else
    {
        return HAL_BUSY; // UART忙，无法启动DMA接收
    }
}

/**
 * @brief  初始化RC UART接收功能
 * @note   该函数用于初始化RC UART接收功能，主要包括清除UART的空闲标志、使能空闲中断以及启动DMA接收。
 */
void RC_Init(void)
{
    __HAL_UART_CLEAR_IDLEFLAG(RC_UART_Ptr); // 用于清除UART的空闲标志，空闲标志指示UART在接收数据时处于空闲状态，通常在接收完成后设置
    // 清除这个标志是为了确保后续的接收操作能够正确检测到新的空闲状态
    __HAL_UART_ENABLE_IT(RC_UART_Ptr, UART_IT_IDLE); // 使能UART的空闲中断，当UART处于空闲状态并且接收缓冲区没有数据时，会触发这个中断
    // 使能这个中断后，可以在中断服务例程中处理空闲状态
    uart_receive_dma_no_it(RC_UART_Ptr, RC_buffer, DMA_MAX_LEN); // 调用之前的函数，用DMA来接收串口数据
}

/**
 * @brief  RC UART接收回调处理函数
 * @note   该函数用于处理RC UART接收到的数据，将接收到的数据解析为各个通道的值，并进行异常数据检测。
 * @param  rc_info: 指向RC结构体的指针，用于存储解析后的数据
 * @param  RC_buffer: 指向RC数据缓冲区的指针，用于存储接收到的数据
 * @retval 返回1表示处理成功，返回0表示参数无效
 */
uint8_t RC_callback_handler(RC_t *rc_info, uint8_t *RC_buffer)
{
    if ((rc_info == NULL) || (RC_buffer == NULL))
    {
        return 0;
    }
    
    uint32_t primask;
    /*
     * PRIMASK 是 Cortex-M 内核的中断屏蔽寄存器。
     * primask = 0：当前全局中断打开。
     * primask = 1：当前全局中断关闭。
     */
    primask = __get_PRIMASK();

    /*
     * 关闭全局中断，进入临界区。
     */
    __disable_irq();

    /*数据解析额，关闭中断是为了防止在解析数据的过程中被中断打断，确保数据的一致性和完整性。*/
    if (RC_buffer[18] == 0x00 && RC_buffer[19] == 0x00 && RC_buffer[20] == 0x00 && 
        RC_buffer[21] ==0x00 && RC_buffer[22] == 0x00 && RC_buffer[23] == 0x00)
    {
        // 将buff[0]和buff[1]的值组合为ch0通道的值，并将其限制在11位（通过与0x07FF按位与）
        rc_info->RC_DBUS_t.ch0 = (RC_buffer[0] | RC_buffer[1] << 8) & 0x07FF;
        rc_info->RC_DBUS_t.ch0 -= 1024; // 由于数据在364到1684，将解码后的数据减去1024，使其中心值为0
        rc_info->RC_DBUS_t.ch1 = (RC_buffer[1] >> 3 | RC_buffer[2] << 5) & 0x07FF;
        rc_info->RC_DBUS_t.ch1 -= 1024;
        rc_info->RC_DBUS_t.ch2 = (RC_buffer[2] >> 6 | RC_buffer[3] << 2 | RC_buffer[4] << 10) & 0x07FF;
        rc_info->RC_DBUS_t.ch2 -= 1024;
        rc_info->RC_DBUS_t.ch3 = (RC_buffer[4] >> 1 | RC_buffer[5] << 7) & 0x07FF;
        rc_info->RC_DBUS_t.ch3 -= 1024;
        rc_info->RC_DBUS_t.roll = (RC_buffer[16] | (RC_buffer[17] << 8)) & 0x07FF; // 左上角滚轮
        rc_info->RC_DBUS_t.roll -= 1024;

        rc_info->RC_DBUS_t.sw1 = ((RC_buffer[5] >> 4) & 0x000C) >> 2;
        rc_info->RC_DBUS_t.sw2 = (RC_buffer[5] >> 4) & 0x0003; // switch_left和switch_right的值分别由相应的位计算得出

        memcpy(&RC_data[RC_LAST], &RC_data[RC_TEMP], sizeof(RC_t));

        if (primask == 0U)
        {
            __enable_irq();
        }
        return 1;
    }
    else if (RC_buffer[0] == 0x0F && RC_buffer[24] == 0x00)
    {
        rc_info->RC_SBUS_t.ch0 = (uint16_t)((RC_buffer[1] | RC_buffer[2] << 8) & 0x07FF);
        rc_info->RC_SBUS_t.ch1 = (uint16_t)((RC_buffer[2] >> 3 | RC_buffer[3] << 5) & 0x07FF);
        rc_info->RC_SBUS_t.ch2 = (uint16_t)((RC_buffer[3] >> 6 | RC_buffer[4] << 2 | RC_buffer[5] << 10) & 0x07FF);
        rc_info->RC_SBUS_t.ch3 = (uint16_t)((RC_buffer[5] >> 1 | RC_buffer[6] << 7) & 0x07FF);
        rc_info->RC_SBUS_t.ch4 = (uint16_t)((RC_buffer[6] >> 4 | RC_buffer[7] << 4) & 0x07FF);
        rc_info->RC_SBUS_t.ch5 = (uint16_t)((RC_buffer[7] >> 7 | RC_buffer[8] << 1 | RC_buffer[9] << 9) & 0x07FF);
        rc_info->RC_SBUS_t.ch6 = (uint16_t)((RC_buffer[9] >> 2 | RC_buffer[10] << 6) & 0x07FF);
        rc_info->RC_SBUS_t.ch7 = (uint16_t)((RC_buffer[10] >> 5 | RC_buffer[11] << 3) & 0x07FF);
        rc_info->RC_SBUS_t.ch8 = (uint16_t)((RC_buffer[12] | RC_buffer[13] << 8) & 0x07FF);
        rc_info->RC_SBUS_t.ch9 = (uint16_t)((RC_buffer[13] >> 3 | RC_buffer[14] << 5) & 0x07FF);
        rc_info->RC_SBUS_t.ch10 = (uint16_t)((RC_buffer[14] >> 6 | RC_buffer[15] << 2 | RC_buffer[16] << 10) & 0x07FF);
        rc_info->RC_SBUS_t.ch11 = (uint16_t)((RC_buffer[16] >> 1 | RC_buffer[17] << 7) & 0x07FF);
        rc_info->RC_SBUS_t.ch12 = (uint16_t)((RC_buffer[17] >> 4 | RC_buffer[18] << 4) & 0x07FF);
        rc_info->RC_SBUS_t.ch13 = (uint16_t)((RC_buffer[18] >> 7 | RC_buffer[19] << 1 | RC_buffer[20] << 9) & 0x07FF);
        rc_info->RC_SBUS_t.ch14 = (uint16_t)((RC_buffer[20] >> 2 | RC_buffer[21] << 6) & 0x07FF);
        rc_info->RC_SBUS_t.ch15 = (uint16_t)((RC_buffer[21] >> 5 | RC_buffer[22] << 3) & 0x07FF);

        memcpy(&RC_data[RC_LAST], &RC_data[RC_TEMP], sizeof(RC_t));
        

        if (primask == 0U)
        {
            __enable_irq();
        }
        return 1;
    }

    /*
     * 如果进入函数前中断是打开的，这里恢复打开。
     * 如果进入函数前中断本来就是关闭的，则不擅自打开。
     */
    if (primask == 0U)
    {
        __enable_irq();
    }
    return 0;
}

/**
 * @brief  获取RC数据结构体的快照
 * @note   该函数用于获取RC数据结构体的快照，以便在中断处理程序中使用。
 * @retval 返回RC数据结构体的快照
 */
RC_t RC_GetInfo(void)
{
    RC_t rc;
    uint32_t primask;

    /*
     * PRIMASK 是 Cortex-M 内核的中断屏蔽寄存器。
     * primask = 0：当前全局中断打开。
     * primask = 1：当前全局中断关闭。
     */
    primask = __get_PRIMASK();

    /*
     * 关闭全局中断，进入临界区。
     */
    __disable_irq();

    /*
     * 拷贝结构体快照。
     */
    rc = RC_data[RC_LAST];

    /*
     * 如果进入函数前中断是打开的，这里恢复打开。
     * 如果进入函数前中断本来就是关闭的，则不擅自打开。
     */
    if (primask == 0U)
    {
        __enable_irq();
    }

    return rc;
}

/**
 * @brief  获取DMA当前接收的数据长度
 * @param  dma_stream: 指向DMA流的指针
 * @retval 返回DMA当前接收的数据长度
 */
uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream)
{
    if (dma_stream == NULL)
    {
        return 0U;
    }
    return ((uint16_t)(dma_stream->NDTR));
}

/**
 * @brief  UART空闲中断回调函数
 * @param  huart: 指向UART句柄的指针
 * @retval None
 */
static void uart_rx_idle_callback(UART_HandleTypeDef *huart)
{
    RC_t rc_temp;
    // 清除UART的空闲标志，以便下一次接收时能够正确检测到空闲状态
    __HAL_UART_CLEAR_IDLEFLAG(huart);

    if ((huart != RC_UART_Ptr) || (huart->hdmarx == NULL))
    {
        return;
    }
    else // 确保只处理DBUS串口
    {
        // 失能DMA接收，防止下一次接收的数据在上一次数据的尾部，而不是全新的数据
        __HAL_DMA_DISABLE(huart->hdmarx); // __HAL_DMA_DISABLE() 本质是清除 DMA_SxCR 寄存器中的 EN 位。
        // EN写 0 后，硬件不一定立刻清零，所以需要等待
        while ((huart->hdmarx->Instance->CR & DMA_SxCR_EN) != 0U)
        {
        }

        // 计算当前接收的数据长度，如果接收到的数据长度等于18||25字节，则调用处理数据函数
        if ((DMA_MAX_LEN - dma_current_data_counter(huart->hdmarx->Instance)) == DBUS_BUF_LEN || (DMA_MAX_LEN - dma_current_data_counter(huart->hdmarx->Instance)) == SBUS_BUF_LEN)
        {
            // 调用处理数据函数，将接收到的数据解析为各个通道的值，并进行异常数据检测
            if (RC_callback_handler(&rc_temp, RC_buffer) != 0)
            {
                RC_data[RC_LAST] = rc_temp;
            }
            else
            {
                memset(&RC_data, 0, sizeof(RC_data));
            }
        }
        __HAL_DMA_SET_COUNTER(huart->hdmarx, DMA_MAX_LEN); // 设置DMA接收预定义的缓冲区的长度，以便为下一次接收做好准备
        __HAL_DMA_ENABLE(huart->hdmarx);                   // 重新启用DMA接收，以便继续接收数据
        // 重新设置UART的接收状态和接收计数器，以便为下一次接收做好准备
        huart->RxXferCount = DMA_MAX_LEN;
        huart->RxState = HAL_UART_STATE_BUSY_RX;
    }
}

/**
 * @brief  UART接收中断处理函数
 * @param  huart: 指向UART句柄的指针
 * @retval None
 */
void uart_receive_handler(UART_HandleTypeDef *huart)
{
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) &&  // 检查UART是否设置了空闲标志，表示UART接收完成并进入空闲状态
        __HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE)) // 检查UART空闲中断是否被使能，只有在中断使能的情况下，才会处理空闲状态
    {
        uart_rx_idle_callback(huart); // 调用之前定义的函数，处理接收到的数据
    }
}
