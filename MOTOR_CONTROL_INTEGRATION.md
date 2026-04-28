# 六步换相电机控制模块集成指南

## 概述

本项目现已添加完整的六步换相电机控制模块（BLDC六步换相）。该模块采用模块化设计，低耦合度，遵循ST MCSDK架构规范。

## 文件结构

### 新增文件

1. **Core/Inc/motor_control.h**
   - 电机控制模块的公共接口定义
   - 包含状态机枚举、数据结构和函数声明

2. **Core/Src/motor_control.c**
   - 六步换相算法核心实现
   - 包含PWM输出控制、BEMF采样、零交叉检测
   - 提供状态管理和刷新方法

### 修改的文件

3. **Core/Src/main.c**
   - 添加了`motor_control.h`包含
   - 添加了电机配置结构体
   - 在`main()`初始化中加入`Motor_Init()`
   - 在主循环中加入`Motor_Update()`

4. **Core/Src/stm32g4xx_it.c**
   - 修改了`HAL_GPIO_EXTI_Callback()`来实现按键启停
   - 添加了按键防抖逻辑（50ms）
   - 集成电机状态管理

## 功能说明

### 六步换相状态

| 扇区 | UH | VH | WH | UL | VL | WL | 监测相 |
|-----|----|----|----|----|----|----|--------|
| 0   | ON | OFF| OFF| OFF| OFF| ON | W      |
| 1   | ON | OFF| OFF| OFF| ON | OFF| V      |
| 2   | OFF| ON | OFF| ON | ON | OFF| U      |
| 3   | OFF| ON | OFF| ON | OFF| OFF| W      |
| 4   | OFF| OFF| ON | ON | OFF| OFF| V      |
| 5   | OFF| OFF| ON | OFF| ON | OFF| U      |

### 按键功能

- **按下按键（PC9）**：在电机空闲和运行状态之间切换
  - IDLE → RUNNING：LED点亮，电机启动
  - RUNNING → IDLE：LED熄灭，电机停止
- **按键防抖时间**：50ms

### 电机启动过程

1. 按键触发中断
2. 中断回调检查电机状态
3. 如果为IDLE状态，调用`Motor_Start()`
4. `Motor_Start()`设置初始扇区(0)并启用PWM输出
5. 主循环中`Motor_Update()`持续更新电机状态

### BEMF零交叉检测

- 每个扇区监测对应的相（U、V或W）
- 简单阈值比较：2048±100（12位ADC中点）
- 需要3次连续检测确认（防止噪声干扰）
- 零交叉确认后触发换相

## API使用示例

```c
/* 初始化 */
Motor_Config_T config = {
  .pwm_frequency = 15000,    /* 15 kHz PWM */
  .dead_time_ns = 2000,      /* 2000 ns死区 */
  .pole_pair_num = 2,        /* 2对极 */
  .min_speed_rpm = 100,
  .max_speed_rpm = 8000,
  .acceleration_ramp = 100
};
Motor_Init(&htim1, &config);

/* 设置PWM占空比 */
Motor_SetDuty(50);  /* 50% */

/* 主循环更新 */
while (1) {
  Motor_Update();
  HAL_Delay(1);  /* 1ms循环周期 */
}

/* 获取电机状态 */
Motor_Status_T status = Motor_GetStatus();
// status.state 当前状态
// status.sector 当前扇区
// status.bemf_u/v/w BEMF值
// status.commutation_count 换相计数
```

## ADC集成（BEMF值更新）

在ADC中断处理中调用以下函数更新BEMF值：

```c
/* 在ADC1转换完成中断中 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC1) {
    uint16_t bemf_u = HAL_ADC_GetValue(ADC1);  /* Channel 6 */
    uint16_t bemf_v = HAL_ADC_GetValue(ADC1);  /* Channel 7 */
    uint16_t bemf_w = HAL_ADC_GetValue(ADC1);  /* Channel 8 */
    
    Motor_SetBEMFValues(bemf_u, bemf_v, bemf_w);
  }
}
```

## 硬件配置检查表

- [x] TIM1：PWM生成，15kHz，死区时间~1064ns（≥1000ns要求）
- [x] GPIO：PWM输出（PA8-PA10, PB13, PA12, PB15）
- [x] GPIO：按键输入（PC9，EXTI9_5中断）
- [x] GPIO：LED输出（PB5，状态指示）
- [x] ADC1：BEMF采样（PC0-PC2）
- [x] ADC1：总线电压和温度采样

## 测试点（Debug）

### LED指示
- **LED熄灭**：电机停止（IDLE）
- **LED点亮**：电机运行（RUNNING）
- **LED在运行中闪烁**：每次换相时闪烁一次

### 调试变量
```c
Motor_Status_T status = Motor_GetStatus();

/* 获取当前BEMF值 */
Motor_Status_T bemf = Motor_GetBEMFReadings();
uint16_t u_val = bemf.bemf_u;
uint16_t v_val = bemf.bemf_v;
uint16_t w_val = bemf.bemf_w;

/* 获取当前扇区 */
Motor_Sector_E sector = Motor_GetSector();

/* 监测换相次数 */
uint32_t comm_count = status.commutation_count;
```

## 编译注意事项

### Keil MDK C语言标准设置

确保以下头文件被正确包含：
- `<stdint.h>` - 标准整数类型（uint8_t, uint16_t, uint32_t）
- `<stddef.h>` - 标准定义（NULL, size_t等）

如果遇到"未定义标识符 uint16_t"的错误：
1. 清理编译输出（Project → Clean）
2. 重建项目（Project → Rebuild）
3. 检查C语言版本是否为C99或更新

## 性能参数

- **PWM频率**：15 kHz
- **死区时间**：约1064 ns（181 clocks @ 170MHz）
- **采样周期**：1 ms（main loop）
- **零交叉检测延迟**：约3 ms（需要3次确认）
- **按键防抖时间**：50 ms

## 下一步优化建议

1. **BEMF阈值自适应**：根据电机运行速度动态调整
2. **启动加速梯形波**：实现平滑启动，避免失步
3. **速度反馈控制**：基于换相计数计算实时转速
4. **故障检测**：检测霍尔信号丢失、过流等
5. **UART调试输出**：实时监控电机参数

## 文件清单

- ? Core/Inc/motor_control.h - 286 行
- ? Core/Src/motor_control.c - 431 行
- ? Core/Src/main.c - 已修改
- ? Core/Src/stm32g4xx_it.c - 已修改

## 版本信息

- **模块版本**：1.0
- **创建时间**：2026年
- **目标MCU**：STM32G431CB @ 170MHz
- **PMSM参数**：极对数=2, RPM范围=100-8000

## 支持

如有问题或需要修改，请参考源代码注释或查阅STM32 HAL文档。
