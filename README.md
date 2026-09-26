# FS_ST16 —— STM32F427 遥控器接收工程（SBUS / DBUS）

> 本工程通过 **UART + DMA + 空闲中断（IDLE）** 接收遥控器发来的 **SBUS** 或 **DBUS（DJI DT7）** 数据帧，
> 解析成统一的遥控器数据结构体，供应用层安全使用。
>
> 重点：**中断只搬运，任务做解析**。

---


### 0.1 涉及的寄存器位速查（硬件层）

> 读表姿势：STM32 的外设寄存器就是映射在内存上的 32 位变量，每一位是一个开关/标志。
> 下文所有 HAL 宏本质上都只是"对某一位读/写/置/清"的封装，对照见 §0.5。

| 寄存器·位 | 名称 | 它是什么 | 本模块怎么用它 | 出现在(rc.c) |
|---|---|---|---|---|
| `USART1->SR` bit4 | `IDLE` | 线路空闲标志：接收线静默超过 1 帧时间自动置 1 | **分帧依据**：IDLE=“一帧收完”；清除必须“先读 SR 再读 DR” | `uart_receive_handler` / `uart_rx_idle_callback` |
| `USART1->SR` bit0~3 | `PE / FE / NE / ORE` | 奇偶错 / 帧错 / 噪声 / 溢出（物理层错误标志） | 只在清标志前读一次做统计（`uart_err_cnt`），不处理、不开错误中断 | 上半部·第 0 步 |
| `USART1->DR` | — | 接收数据寄存器（收到字节的落点） | DMA 搬运的**源地址**；清 IDLE 时被顺带读一次 | `uart_receive_dma_no_it` |
| `USART1->CR1` bit4 | `IDLEIE` | IDLE 中断使能位 | `RC_Init()`：先关→清标志→DMA 启动成功才开 | `RC_Init` |
| `USART1->CR3` bit0 | `DMAR` | 串口“向 DMA 发接收请求”的总开关 | 先关再配 DMA，配好再开（防配置中途来数据） | `uart_receive_dma_no_it` |
| `DMA2_Stream2->CR` bit0 | `EN` | 这条 DMA 流的总开关 | 写 0 停搬运，但硬件不会立刻清零 → **带超时轮询等待**；一帧收尾重新置 1 | `uart_rx_idle_callback` |
| `DMA2_Stream2->NDTR` | — | 还剩多少字节没搬（硬件每搬 1 字节自动 −1） | **核心公式：帧长 = `DMA_MAX_LEN − NDTR`**；重启前重装为 36 | `dma_current_data_counter` |
| `NVIC` | `USART1_IRQn` | 中断优先级 | 优先级 5，与 `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` 同级 → 中断里调 `...FromISR` 合法 | CubeMX 配置（本模块不动） |

### 0.2 CMSIS 内核函数速查

| 符号 | 干什么 | 要点 |
|---|---|---|
| `__get_PRIMASK()` | 读“全局中断总开关”状态 | 0=中断开着；1=已关闭（临界区“记账”用它） |
| `__disable_irq()` / `__enable_irq()` | 关 / 开全局中断 | Cortex-M 上这会同时屏蔽 PendSV → **同时阻止任务抢占**，等价 RTOS 临界区，但不依赖 FreeRTOS |
| `__DMB()` | 数据内存屏障 | 禁止编译器/CPU 乱序：保证“数据先落地、下标后发布”（暂存区不加锁的关键之一） |

### 0.3 HAL 句柄里的“记账”字段（本模块手工维护的部分）

| 字段 | 含义 | 为什么要管它 |
|---|---|---|
| `huart->RxState` | 串口接收状态机（READY / BUSY_RX） | 没走 `HAL_UART_Receive_DMA()`，但要告诉 HAL“正在收”，防止其它 HAL 调用误判 |
| `huart->pRxBuffPtr` / `RxXferSize` / `RxXferCount` | 落点地址 / 总量 / 剩余量（软件侧） | 与硬件 NDTR 保持一致的记账值 |
| `huart->ErrorCode` | 错误码 | 启动前清零 |
| `huart->hdmarx` | 关联的接收 DMA 句柄 | CubeMX 的 `__HAL_LINKDMA()` 已绑定，本模块只读取 |
| `huart->Instance` | USART 寄存器基址（SR/DR/CR1/CR3） | 配合 `CLEAR_BIT/SET_BIT` 直接操作位 |
| `hdma->Instance` | DMA 流寄存器基址（CR/NDTR） | 停流、等停、读帧长的来源 |


### 0.5 HAL 宏 ↔ 实际硬件动作对照

| 宏 / 函数 | 实际做的事 |
|---|---|
| `__HAL_UART_ENABLE_IT(h, UART_IT_IDLE)` / `..._DISABLE_IT` | 置 / 清 `CR1` 的 `IDLEIE` |
| `__HAL_UART_CLEAR_IDLEFLAG(h)` | 读 `SR` 再读 `DR`（标准清除动作，顺带清掉 PE/FE/NE/ORE） |
| `__HAL_UART_GET_FLAG(h, UART_FLAG_IDLE)` | 读 `SR` 的 `IDLE` 位 |
| `__HAL_UART_GET_IT_SOURCE(h, UART_IT_IDLE)` | 读 `CR1` 是否使能了 IDLE 中断 |
| `CLEAR_BIT(reg, bit)` / `SET_BIT(reg, bit)` | `reg &= ~bit` / `reg |= bit` |
| `__HAL_DMA_DISABLE(h)` / `__HAL_DMA_ENABLE(h)` | 清 / 置 `CR` 的 `EN` |
| `__HAL_DMA_SET_COUNTER(h, n)` | `NDTR = n` |
| `HAL_DMA_Start(h, src, dst, len)` | 配置流 + 装 NDTR + 开 EN，**不开 DMA 中断** |
| `HAL_GetTick()` | 系统毫秒计数（TIM6 时基）—— `lost` 超时判定依据 |

---

## 1. 工程总览

### 1.1 硬件与软件基础

| 项目 | 配置 |
|---|---|
| MCU | STM32F427IIHx（Cortex-M4F） |
| 时钟 | HSE 12MHz → PLL(M=6, N=168, P=2) → **SYSCLK 168MHz**；AHB 168 / APB1 42 / APB2 84MHz |
| 遥控器串口 | **USART1**：PB6(TX)/PB7(RX)，**100000 baud，8 数据位 + 偶校验 + 2 停止位（8E2），仅接收** |
| 接收 DMA | **DMA2_Stream2 / Channel4**，外设→内存、内存地址递增、字节对齐、**循环模式**，长度 36 字节 |
| 中断优先级 | USART1 = 5，DMA2_Stream2 = 5，TIM6(HAL 时基) = 15，PendSV/SysTick = 15 |
| RTOS | FreeRTOS + CMSIS-OS2，tick 1000Hz，heap_4（15360B） |
| 任务 | `defaultTask`（512B 栈，osPriorityNormal）；`rc_task`（512B 栈，osPriorityNormal1=25） |
| 编译 | Keil MDK-ARM，C99，**-O3** |
| 其他外设 | CAN1（1Mbps）与 TIM3 已由 CubeMX 配置，当前**未使用** |

> 说明：`configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5`，USART1 中断优先级也是 5，
> 因此在串口中断里调用 `...FromISR` 版本的 FreeRTOS API 是**合法**的。
> `rc_task` 优先级(25) 高于 `defaultTask`(24)，收到通知可立即抢占执行。

### 1.2 目录结构

```
FS_ST16/
├─ Core/               CubeMX 生成：main.c、外设初始化(usart/dma/tim/can/gpio)、中断向量
├─ Drivers/            HAL 库 + CMSIS（ST 官方）
├─ Middlewares/        FreeRTOS 源码
├─ MDK-ARM/            Keil 工程与构建产物
├─ rc/                 ★ 遥控器驱动层
│   ├─ rc.h            对外接口 + 数据结构 + 配置参数
│   └─ rc.c            实现（上半部/下半部、环形暂存、解析、状态维护）
├─ task/               ★ 应用层示例
│   ├─ rc_task.h       任务声明 + 配置宏
│   └─ rc_task.c       事件驱动任务（注册回调 / 处理 / 取数 / 失效判断）
├─ FS_ST16.ioc         CubeMX 配置（外设与时钟的唯一来源）
└─ README.md           本文档
```

**分层原则**：`rc/` 只关心"收字节 → 拆位 → 判断可信度"，不碰业务；`task/` 只关心"什么时候处理、怎么用"。
`rc.c` 不包含任何 FreeRTOS 代码 —— 它只在"有新帧"时喊一声（回调），谁听由应用决定。

### 1.3 模块职责一览

| 文件 | 职责 | 是否依赖 RTOS |
|---|---|---|
| `Core/Src/usart.c` | USART1 + DMA 初始化（CubeMX 生成，**本模块不改动**） | 否 |
| `Core/Src/stm32f4xx_it.c` | `USART1_IRQHandler` 里调用 `uart_receive_handler(&huart1)` | 否 |
| `rc/rc.c` `rc/rc.h` | 接收驱动：分帧、解析、状态维护、失效判定 | **否** |
| `task/rc_task.c` | 事件驱动任务、通知回调、失效保护示例 | 是 |

---

## 2. 整体架构

### 2.1 分层图

```

```

### 2.2 一帧数据的完整旅程

```
① 遥控器发来 18 或 25 字节
② UART 硬件自动请求 DMA → 逐字节搬进 RC_buffer（CPU 全程不参与），NDTR 递减
③ 一帧结束、线路空闲 → 触发 USART1 中断
④ 【上半部】uart_rx_idle_callback()：                    ← 约 1~2µs，不关中断
     → 帧长 18/25 就把这 ≤25 字节拷进暂存槽；否则存一个"坏帧事件"
     → 重装 NDTR、重新使能 DMA
     → 调用通知回调，叫醒任务
⑤ 【下半部】RC_ProcessPending()：                        ← 任务里做重活

⑥ 应用层 RC_GetInfo() 取快照 → 使用通道值
```

### 2.3 为什么要"上半部 / 下半部"

| | 全部在中断里做（旧写法） | 中断搬运 + 任务解析（现在） |
|---|---|---|
| 中断停留时间 | 解析 16 通道 + 维护状态，约 5~10µs | 只拷 ≤25 字节，约 1~2µs |
| 对其他中断的影响 | 解析期间抢占被推迟 | 几乎无影响（中断内不关中断） |
| 解析器可测性 | 依赖硬件上下文 | 纯函数，可脱离硬件测试 |
| 代价 | — | 多一次 ≤25B 内存拷贝 + 一个调度周期延迟（都是微秒级） |

---
