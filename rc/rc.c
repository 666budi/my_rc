/**
 * @file rc.c
 * @author xioafo
 * @brief  SBUS/DBUS遥控器数据解析
 * @version 1.1
 * @date    2026-09-26
 * @note    ASCII 流程图:
 * @code
 * @endcode
 */
#include "rc.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

/** @brief 接收串口句柄指针定义，若新工程更换串口只需要修改RC_UART_Ptr地址 */
static UART_HandleTypeDef *RC_UART_Ptr = &huart1; // 定义RC_UART的指针

/** @brief 全部变量定义 */
uint8_t RC_buffer[DMA_MAX_LEN] = {0}; // 定义DBUS数据缓存区，初始化为0
RC_t RC_data[2] = {0};                // 定义遥控器数据结构体

/* ===========================================================================
 * 内部工具：临界区
 * ---------------------------------------------------------------------------
 * __disable_irq()(PRIMASK=1) 会屏蔽所有可配置优先级的中断，其中包括 FreeRTOS
 * 用来切换任务的 PendSV —— 所以它同时阻止了"任务被抢占"，等价于
 * taskENTER_CRITICAL()。本模块不依赖 RTOS，因此用它实现原子访问。
 * ========================================================================= */

/** @brief 进入临界区；返回值需交给 rc_exit_critical() 用于退出 */
static uint32_t rc_enter_critical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

/** @brief 退出临界区：只有进门时中断是开着的才恢复打开 */
static void rc_exit_critical(uint32_t primask)
{
    if (primask == 0u)
    {
        __enable_irq();
    }
}

/* ===========================================================================
 * 内部工具：读 DMA 剩余计数
 * NDTR：DMA 的剩余传输计数
 * ========================================================================= */

/**
 * @brief  读取 DMA 的剩余传输计数 NDTR
 * @param  dma_stream : DMA 流寄存器基地址
 * @retval 剩余未搬运的字节数；参数非法返回 0
 * @note   已收字节数 = DMA_MAX_LEN - NDTR。调用前 DMA 必须已停稳。
 */
static uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream)
{
    if (dma_stream == NULL)
    {
        return 0U;
    }
    return ((uint16_t)(dma_stream->NDTR));
}

/* ===========================================================================
 * 内部工具：不产生中断地启动 DMA 接收
 * ========================================================================= */

/**
 * @brief  以"不产生中断"的方式启动 UART 的 DMA 接收
 * @param  huart : 串口句柄
 * @param  pData : 接收缓冲区首地址
 * @param  Size  : 本次要接收的字节数
 * @retval HAL_OK / HAL_ERROR / HAL_BUSY
 * @note   不用 HAL_UART_Receive_DMA() 的原因：它会额外打开 DMA 中断并改写一批
 *         HAL 状态字段，而这里只需要"开始搬"这一个动作。
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
 * @brief  UART空闲中断回调函数
 * @param  huart: 指向UART句柄的指针
 * @retval None
 */
static void uart_rx_idle_callback(UART_HandleTypeDef *huart)
{
    // 清除UART的空闲标志，以便下一次接收时能够正确检测到空闲状态
    __HAL_UART_CLEAR_IDLEFLAG(huart);

    if ((huart != RC_UART_Ptr) || (huart->hdmarx == NULL))
    {
        return;
    }
    else // 确保只处理指定串口
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
            uint8_t len = DMA_MAX_LEN - dma_current_data_counter(huart->hdmarx->Instance);
            // 调用处理数据函数，将接收到的数据解析为各个通道的值，并进行异常数据检测
            if (RC_callback_handler(&RC_data[RC_TEMP], RC_buffer, len) != 0)
            {
                /* 数据没问题把暂存区数据赋值到可用区 */
                memcpy(&RC_data[RC_LAST], &RC_data[RC_TEMP], sizeof(RC_t));
            }
            else
            {
                /* 数据解析不正常，暂存区归0，不赋值到可用区 */
                memset(&RC_data[RC_TEMP], 0, sizeof(RC_data[RC_TEMP]));
            }
        }
        __HAL_DMA_SET_COUNTER(huart->hdmarx, DMA_MAX_LEN); // 设置DMA接收预定义的缓冲区的长度，以便为下一次接收做好准备
        __HAL_DMA_ENABLE(huart->hdmarx);                   // 重新启用DMA接收，以便继续接收数据
        // 重新设置UART的接收状态和接收计数器，以便为下一次接收做好准备
        huart->RxXferCount = DMA_MAX_LEN;
        huart->RxState = HAL_UART_STATE_BUSY_RX;
    }
}

void RC_Init(void)
{
    /** 用于清除UART的空闲标志，空闲标志指示UART在接收数据时处于空闲状态，通常在接收完成后设置
     * 清除这个标志是为了确保后续的接收操作能够正确检测到新的空闲状态 */
    __HAL_UART_CLEAR_IDLEFLAG(RC_UART_Ptr);
    /** 使能UART的空闲中断，当UART处于空闲状态并且接收缓冲区没有数据时，会触发这个中断
     * 使能这个中断后，可以在中断服务例程中处理空闲状态 */
    __HAL_UART_ENABLE_IT(RC_UART_Ptr, UART_IT_IDLE);
    /** 调用不产生中断的DMA开启接收函数，用DMA来接收串口数据 */
    uart_receive_dma_no_it(RC_UART_Ptr, RC_buffer, DMA_MAX_LEN);
}

uint8_t RC_callback_handler(RC_t *rc_info, uint8_t *RC_buffer, uint8_t len)
{
    if ((rc_info == NULL) || (RC_buffer == NULL) || len == 0)
    {
        return 0;
    }

    uint32_t primask;
    /*
     * PRIMASK 是 Cortex-M 内核的中断屏蔽寄存器
     * primask = 0：当前全局中断打开
     * primask = 1：当前全局中断关闭
     */

    /*
     * 关闭全局中断，进入临界区
     * 进入临界区，实现原子操作
     */
    primask = rc_enter_critical();

    /* 数据解析额，关闭中断是为了防止在解析数据的过程中被中断打断，确保数据的一致性和完整性 */
    /* 默认遥控器协议不更换，由于DBUS没有固定格式，利用DBUS只有18字节数据其他字节均为0不变判定 */
    if (len == DBUS_BUF_LEN && (RC_buffer[18] == 0x00 && RC_buffer[19] == 0x00 && RC_buffer[20] == 0x00 &&
                                RC_buffer[21] == 0x00 && RC_buffer[22] == 0x00 && RC_buffer[23] == 0x00))
    {
        // 将buff[0]和buff[1]的值组合为ch0通道的值，并将其限制在11位（通过与0x07FF按位与）
        rc_info->RC_DBUS_t.ch0 = (RC_buffer[0] | RC_buffer[1] << 8) & 0x07FF;
        rc_info->RC_DBUS_t.ch1 = (RC_buffer[1] >> 3 | RC_buffer[2] << 5) & 0x07FF;
        rc_info->RC_DBUS_t.ch2 = (RC_buffer[2] >> 6 | RC_buffer[3] << 2 | RC_buffer[4] << 10) & 0x07FF;
        rc_info->RC_DBUS_t.ch3 = (RC_buffer[4] >> 1 | RC_buffer[5] << 7) & 0x07FF;
        rc_info->RC_DBUS_t.roll = (RC_buffer[16] | (RC_buffer[17] << 8)) & 0x07FF;
        rc_info->RC_DBUS_t.sw1 = ((RC_buffer[5] >> 4) & 0x000C) >> 2;
        rc_info->RC_DBUS_t.sw2 = (RC_buffer[5] >> 4) & 0x0003;

        /*
         * 打开全局中断，退出临界区
         */
        rc_exit_critical(primask);
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

        rc_exit_critical(primask);
        return 1;
    }

    /*
     * 如果进入函数前中断是打开的，这里恢复打开
     * 如果进入函数前中断本来就是关闭的，则不擅自打开
     */
    rc_exit_critical(primask);
    return 0;
}

RC_t RC_GetInfo(void)
{
    RC_t rc;
    /* 进入临界区，实现原子操作 */
    uint32_t primask = rc_enter_critical();

    /* 拷贝结构体快照 */
    rc = RC_data[RC_LAST];

    /* 打开全局中断，退出临界区*/
    rc_exit_critical(primask);

    return rc;
}

void uart_receive_handler(UART_HandleTypeDef *huart)
{
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) &&  // 检查UART是否设置了空闲标志，表示UART接收完成并进入空闲状态
        __HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE)) // 检查UART空闲中断是否被使能，只有在中断使能的情况下，才会处理空闲状态
    {
        uart_rx_idle_callback(huart); // 调用之前定义的函数，处理接收到的数据
    }
}
