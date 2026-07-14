# Motor_F103_TB6612

基于 **STM32F103 + TB6612 + 增量式编码器** 的单路有刷直流电机控制与 PID 调试工程。

## 1. 当前功能

- TB6612 单路电机正转、反转和刹车
- 开环 PWM 占空比控制
- TIM2 编码器模式累计计数
- 输出轴速度计算，单位 `rad/s`
- 1 kHz 速度 PID 闭环
- USART1 + DMA 串口收发
- Serialplot 波形回传与 PID 参数在线修改
- 默认正弦速度目标测试
- PC13 LED 运行指示

## 2. 开发环境

- MCU：STM32F103xB
- HAL：STM32F1 HAL
- 语言：C11 + C++17
- 构建：CMake + Ninja + `arm-none-eabi-gcc`
- 图形化配置：STM32CubeMX
- 工程文件：`Motor_F103_TB6612.ioc`

命令行编译：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

Release 编译：

```powershell
cmake --preset Release
cmake --build --preset Release
```

构建目录分别为：

```text
build/Debug
build/Release
```

## 3. 硬件引脚

| 功能 | STM32 引脚 | 外设 |
|---|---|---|
| TB6612 PWMA | PA8 | TIM1_CH1 |
| TB6612 AIN1 | PA12 | GPIO 输出 |
| TB6612 AIN2 | PA15 | GPIO 输出 |
| 编码器 A 相 | PA0 | TIM2_CH1 |
| 编码器 B 相 | PA1 | TIM2_CH2 |
| Serialplot TX | PA9 | USART1_TX |
| Serialplot RX | PA10 | USART1_RX |
| 运行 LED | PC13 | GPIO 输出 |

本工程初始化时没有传入 STBY GPIO，因此要求 TB6612 的 `STBY` 已在硬件上拉高；否则电机不会输出。

基本接线：

```text
STM32 PA8   → TB6612 PWMA
STM32 PA12  → TB6612 AIN1
STM32 PA15  → TB6612 AIN2

TB6612 AO1/AO2 → 电机两根动力线
TB6612 STBY     → 高电平
TB6612 VCC      → 逻辑电源
TB6612 VM       → 电机电源
STM32 GND       ↔ TB6612 GND

编码器 A → PA0
编码器 B → PA1
编码器 VCC/GND → 对应电源与地
```

插拔电机和编码器前先断电，所有模块必须共地。

## 4. 定时器配置

### TIM1

TIM1 同时用于：

- 1 kHz 基础定时中断
- TIM1_CH1 PWM 输出

当前参数：

```text
系统时钟：72 MHz
Prescaler：72 - 1
Period：1000 - 1
计数频率：1 MHz
更新频率：1 kHz
PWM 频率：1 kHz
PWM 比较值范围：0～1000
```

### TIM2

TIM2 使用编码器模式 `TI12`：

```text
PA0 → TIM2_CH1
PA1 → TIM2_CH2
计数范围：0～65535
输入滤波：4
```

电机类每 1 ms 读取一次 TIM2 计数器，通过 `int16_t` 差值处理 16 位计数器回绕。

## 5. 编码器参数

初始化使用：

```cpp
13 * 20 * 4
```

表示：

```text
编码器线数：13
减速比：1:20
正交四倍频：×4
输出轴每圈计数：1040
```

当前速度计算：

```text
Now_Omega =
    Encoder_Delta × 2π
    ÷ (1040 × 0.001s)
```

单位为：

```text
rad/s
```

如果实际电机编码器规格或减速比不同，需要修改 `Task_Init()` 中的计数参数。

## 6. 工程结构

```text
Core/
├─ Src/main.c
└─ CubeMX 生成的 GPIO、TIM、UART、DMA 等底层代码

User_File/
├─ 1_Middleware/
│  ├─ Algorithm/PID/
│  │  └─ alg_pid.cpp/.h
│  └─ Driver/
│     ├─ TIM/
│     └─ UART/
├─ 2_Device/
│  ├─ Motor/TB6612_Encoder/
│  │  └─ dvc_tb6612_encoder_motor.cpp/.h
│  └─ Plotter/Serialplot/
│     └─ dvc_serialplot.cpp/.h
└─ 5_Task/
   └─ tsk_config_and_callback.cpp/.h
```

调用关系：

```text
Core/Src/main.c
    ↓
Task_Init() / Task_Loop()
    ↓
Class_TB6612_Encoder_Motor
    ↓
PID、TIM、UART、HAL
```

`Core/Src/main.c` 只负责 HAL/CubeMX 初始化，然后调用：

```cpp
Task_Init();

while (1)
{
    Task_Loop();
}
```

实际应用逻辑主要位于：

```text
User_File/5_Task/tsk_config_and_callback.cpp
```

## 7. 电机对象初始化

当前初始化代码：

```cpp
Motor.Init(&htim1, TIM_CHANNEL_1,
           &htim2,
           GPIOA, GPIO_PIN_12,
           GPIOA, GPIO_PIN_15,
           13 * 20 * 4,
           0.001f);
```

含义：

```text
PWM：TIM1_CH1
编码器：TIM2
AIN1：PA12
AIN2：PA15
输出轴每圈：1040 count
控制周期：1 ms
STBY：硬件上拉
```

当前 PID 初值：

```cpp
Motor.PID_Omega.Init(24.0f, 0.0f, 0.0f, 0.0f,
                     Motor.Get_PWM_Max(),
                     Motor.Get_PWM_Max(),
                     Motor.Get_PWM_Max(),
                     0.001f);
```

即：

```text
Kp = 24
Ki = 0
Kd = 0
控制周期 = 1 ms
输出限幅 = ±PWM_Max
```

PID 参数仅为当前测试参数，换电机、供电、负载后需要重新调节。

## 8. 控制接口

### 开环 PWM

```cpp
Motor.Set_PWM_Percent(0.2f);   // 正转，20%
Motor.Set_PWM_Percent(-0.2f);  // 反转，20%
Motor.Set_PWM_Percent(0.0f);   // 输出为0
```

输入范围：

```text
-1.0 ～ +1.0
```

### 速度闭环

```cpp
Motor.Set_Target_Omega(20.0f);
```

目标单位：

```text
rad/s
```

### 关闭速度闭环，回到开环

```cpp
Motor.Close_Loop_Disable();
Motor.Set_PWM_Percent(0.1f);
```

### 刹车

```cpp
Motor.Brake();
```

刹车时：

```text
AIN1 = 1
AIN2 = 1
PWM = 0
```

### 方向修正

电机动力线或编码器 A/B 相方向与软件定义相反时，可以分别设置：

```cpp
Motor.Set_Motor_Reverse(true);
Motor.Set_Encoder_Reverse(true);
```

## 9. 默认程序行为：上电会自动运行正弦速度测试

当前 `Task1ms_Callback()` 中默认调用：

```cpp
Motor_Sine_Wave_Test();
```

该函数每 20 ms 更新一次目标速度：

```text
幅值：100 rad/s
偏置：0 rad/s
查表点数：100
完整周期：约2秒
```

因此：

> 连接电机和电源后，程序会自动进入正反转变化的速度闭环测试，不是静止待命程序。

首次接线或只想手动控制时，建议先注释：

```cpp
// Motor_Sine_Wave_Test();
```

然后在确认电机悬空、供电和方向均正常后，再使用较小的开环 PWM 测试：

```cpp
Motor.Set_PWM_Percent(0.05f);
```

## 10. Serialplot

USART1 配置：

```text
波特率：1,000,000
数据位：8
停止位：1
校验：None
TX：PA9
RX：PA10
DMA：RX DMA1_Channel5，TX DMA1_Channel4
```

Serialplot 当前发送四个变量：

```text
Motor_Now_Omega
Motor_Target_Omega
Motor_PWM_Out
Motor_Now_Encoder
```

接收变量：

```text
op → 修改 Kp
oi → 修改 Ki
od → 修改 Kd
```

Serialplot 使用：

```text
帧头：0xAB
数据类型：float
校验：Checksum8
```

## 11. LED 指示

`Task_Loop()` 中：

```cpp
HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
HAL_Delay(100);
```

因此 PC13 每 100 ms 翻转一次，完整亮灭周期约 200 ms。

它只表示主循环仍在运行，不代表电机、编码器或 PID 一定正常。

## 12. 推荐测试顺序

1. 不接电机主电源，先确认工程能编译和烧录。
2. 确认 PC13 LED 持续闪烁。
3. 注释默认 `Motor_Sine_Wave_Test()`。
4. 仅接编码器，手转输出轴，观察计数和速度。
5. 电机悬空，使用 5% 左右开环 PWM 测试。
6. 确认电机方向和编码器速度符号一致。
7. 再进入速度闭环并调节 PID。
8. 最后恢复正弦目标或上层控制逻辑。

## 13. 注意事项

- TB6612 的 `STBY` 必须为高电平。
- 电机电源 `VM` 与逻辑电源 `VCC` 不要混接。
- STM32、TB6612、编码器和串口模块必须共地。
- 首次测试不要满占空比。
- 当前默认正弦测试会自动正反转，接电机前必须确认机械安全。
- 修改 `.ioc` 并重新生成代码后，检查用户代码和 CMake 源文件列表是否仍然完整。
