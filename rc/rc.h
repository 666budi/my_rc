/**
 * @file rc.h
 * @author xioafo
 * @brief  SBUS / DBUS 遥控器接收驱动 —— 数据结构与对外接口
 * @version 1.1
 * @date    2026-09-26
 * @note    ASCII 架构图:
 * @code
 *   SBUS 18字节帧 (0x0F ... 0x00)
 *        │
 *        ├─ Byte[0~4]:   通道数据 → rc.rocker_l_/l1, rc.rocker_r_/r1 (11-bit 摇杆)
 *        ├─ Byte[5]:     开关位  → rc.switch_left/right (2-bit 三档开关)
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

/** @brief 帧长(字节)  */
#define DMA_MAX_LEN 36u  // 定义DMA缓存区36字节
#define DBUS_BUF_LEN 18u // 定义DBUS数据帧长度
#define SBUS_BUF_LEN 25u // 定义SBUS数据帧长度

/** @brief 协议标识：上层据此决定读哪个子结构体 */
#define RC_PROTO_NONE 0u // 不来自任何协议
#define RC_PROTO_DBUS 1u // 来自DBUS协议
#define RC_PROTO_SBUS 2u // 来自SBUS协议

/** @brief 暂存结构体索引 */
#define RC_TEMP 0 // 直接处理缓存数据结构体
#define RC_LAST 1 // 确认可用的结构体

/** @brief 定义遥控器数据结构体 */
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

/**  @brief 全局变量声明 */
extern uint8_t RC_buffer[DMA_MAX_LEN];
extern RC_t RC_data[2];

/** @brief 自定义共有函数声明 */

/**
 * @brief  初始化RC UART接收功能
 * @note   该函数用于初始化RC UART接收功能，主要包括清除UART的空闲标志、使能空闲中断以及启动DMA接收。
 */
void RC_Init(void);

/**
 * @brief  RC UART接收回调处理函数
 * @note   该函数用于处理RC UART接收到的数据，将接收到的数据解析为各个通道的值，并进行异常数据检测。
 * @param  rc_info: 指向RC结构体的指针，用于存储解析后的数据
 * @param  RC_buffer: 指向RC数据缓冲区的指针，用于存储接收到的数据
 * @param  len: 有效字节长度
 * @retval 返回1表示处理成功，返回0表示参数无效
 */
uint8_t RC_callback_handler(RC_t *rc_info, uint8_t *RC_buffer, uint8_t len);

/**
 * @brief  获取RC数据结构体的快照
 * @note   该函数用于获取RC数据结构体的快照，以便在中断处理程序中使用。
 * @retval 返回RC数据结构体的快照
 */
RC_t RC_GetInfo(void);

/**
 * @brief  UART接收中断处理函数
 * @param  huart: 指向UART句柄的指针
 * @retval None
 */
void uart_receive_handler(UART_HandleTypeDef *huart);

#endif /* __RC_H */
