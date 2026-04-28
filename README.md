# G431_MY_SIX_STEP_BMEF

STM32G431 六步无刷电机（BLDC）反电动势（BEMF）过零点检测模板
**Six-step sensorless BLDC motor control template for STM32G431**

---

## 功能概述 / Features

| 特性 | 说明 |
|------|------|
| 换相方式 | 六步换相（Six-step commutation） |
| 转子位置检测 | 反电动势过零点检测（Back-EMF zero-crossing） |
| 检测模式 | ADC 注入转换（默认）或片上比较器（COMP） |
| 启动方式 | 开环加速 → 检测到有效 BEMF 后切换闭环 |
| PWM | TIM1 中心对齐，互补输出，死区插入 |
| 保护 | 过电流、堵转、欠压检测 |
| 目标 MCU | STM32G431（170 MHz，STM32 HAL） |

---

## 目录结构 / Directory structure

```
Core/
├── Inc/
│   ├── main.h            # 引脚映射、应用版本
│   ├── motor_control.h   # 电机参数、状态机、六步换相接口
│   └── bemf.h            # BEMF 过零点检测接口
└── Src/
    ├── main.c            # 主循环、按键、LED、外设初始化存根
    ├── motor_control.c   # 六步换相逻辑实现
    └── bemf.c            # BEMF 过零点检测实现
```

---

## 快速入门 / Quick start

1. 用 **STM32CubeMX** 新建 STM32G431 工程，配置：
   - **TIM1**：中心对齐 PWM，互补输出 CH1–CH3，CC4 触发 ADC 注入转换。
   - **ADC1**：注入通道 IN1/IN2/IN3（PA0/PA1/PA2），由 TIM1 CC4 触发。
   - **SysTick**：1 ms（HAL 默认）。
   - **GPIO**：PA5 LED 输出，PC13 按键输入（上拉）。

2. 将 `Core/Inc` 和 `Core/Src` 下的文件复制到生成的工程中。

3. 在 `Core/Inc/main.h` 中取消 `stm32g4xx_hal.h` 的注释。

4. 在 `Core/Src/main.c` 中将 `/* HAL stub */` 注释替换为真实 HAL 调用。

5. 根据实际硬件调整 `motor_control.h` 中的电机参数（极对数、PWM 频率、供电电压等）。

6. 编译、烧录，按下 USER_BTN（PC13）启动电机。

---

## 关键参数 / Key parameters

在 `Core/Inc/motor_control.h` 中修改：

```c
#define MOTOR_SUPPLY_VOLTAGE_MV     24000U   // 供电电压 (mV)
#define MOTOR_POLE_PAIRS            4U        // 极对数
#define PWM_FREQUENCY_HZ            20000U    // PWM 开关频率 (Hz)
#define STARTUP_DUTY_PERMILLE       200U      // 开环启动占空比 (‰)
#define DEFAULT_DUTY_PERMILLE       400U      // 运行目标占空比 (‰, 在 main.c)
```

---

## 硬件接线参考 / Hardware connection

```
TIM1_CH1 / CH1N  →  PA8  / PB13   A相高侧 / 低侧
TIM1_CH2 / CH2N  →  PA9  / PB14   B相高侧 / 低侧
TIM1_CH3 / CH3N  →  PA10 / PB15   C相高侧 / 低侧

ADC1_IN1  →  PA0   A相 BEMF 分压采样
ADC1_IN2  →  PA1   B相 BEMF 分压采样
ADC1_IN3  →  PA2   C相 BEMF 分压采样

USER_BTN  →  PC13  启动/停止按键（低有效）
LED       →  PA5   状态指示灯
```

---

## 状态机 / State machine

```
IDLE ──[Start]──► STARTUP ──[BEMF OK]──► RUNNING
                      │                     │
              [Stall fault]           [Stop / fault]
                      │                     │
                      └──────► FAULT ◄──────┘
                                  │
                           [ClearFault]
                                  │
                                IDLE
```
