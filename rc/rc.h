/**
 * @file rc.h
 * @author TECHX
 * @brief  DJI DT7 遥控器 — 数据结构 / 宏 / 类型安全访问
 * @version 1.0
 * @date    2026-06-27
 * @note    ASCII 架构图:
 * @code
 *   SBUS 18字节帧 (0x0F ... 0x00)
 *        │
 *        ├─ Byte[0~4]:   通道数据 → rc.rocker_l_/l1, rc.rocker_r_/r1 (11-bit 摇杆)
 *        ├─ Byte[5]:     开关位  → rc.switch_left/right (2-bit 三档开关)
 *        ├─ Byte[6~9]:   鼠标    → mouse.x/y (16-bit 增量)
 *        ├─ Byte[12~13]: 鼠标按键 → mouse.press_l/press_r
 *        ├─ Byte[14~15]: 键盘    → key[0].keys (16-bit 位掩码)
 *        └─ Byte[16~17]: 拨轮    → rc.dial (11-bit)
 * @endcode
 *
 * 对其他遥控器类型无任何影响 (完全隔离).
 */
#ifndef __RC_H
#define __RC_H

#include "stm32f4xx_hal.h"
#include "main.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define DMA_MAX_LEN 36u  // 定义DMA缓存区36字节
#define DBUS_BUF_LEN 18u // 定义DBUS数据帧长度
#define SBUS_BUF_LEN 25u // 定义SBUS数据帧长度
#define RC_LAST 0
#define RC_TEMP 1

typedef struct
{
    struct
    {
        int16_t ch0;
        int16_t ch1;
        int16_t ch2;
        int16_t ch3;
        int16_t roll;
        uint8_t sw1;
        uint8_t sw2;
    } RC_DBUS_t;

    struct
    {
        uint16_t ch0;
        uint16_t ch1;
        uint16_t ch2;
        uint16_t ch3;
        uint16_t ch4;
        uint16_t ch5;
        uint16_t ch6;
        uint16_t ch7;
        uint16_t ch8;
        uint16_t ch9;
        uint16_t ch10;
        uint16_t ch11;
        uint16_t ch12;
        uint16_t ch13;
        uint16_t ch14;
        uint16_t ch15;
    } RC_SBUS_t;
} RC_t;

extern uint8_t RC_buffer[DMA_MAX_LEN];
extern RC_t RC_data[2];

void RC_Init(void);
uint8_t RC_callback_handler(RC_t *rc_info, uint8_t *RC_buffer);
RC_t RC_GetInfo(void);
uint16_t dma_current_data_counter(DMA_Stream_TypeDef *hdma);
void uart_receive_handler(UART_HandleTypeDef *huart);

#endif /* __RC_H */
