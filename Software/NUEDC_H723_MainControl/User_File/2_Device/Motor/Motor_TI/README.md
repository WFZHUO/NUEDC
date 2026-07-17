# Motor_TI 黑盒使用说明

## 1. 文件放置

```text
User_File/2_Device/Motor/Motor_TI/
├─ dvc_motor_ti.h
└─ dvc_motor_ti.cpp
```


## 2. CMake

在 `target_sources` 中加入：

```cmake
User_File/2_Device/Motor/Motor_TI/dvc_motor_ti.cpp
```

在 `target_include_directories` 中加入：

```cmake
User_File/2_Device/Motor/Motor_TI
```

## 3. CubeMX要求

USART10：

```text
Baud Rate: 500000
Word Length: 8 Bits
Parity: None
Stop Bits: 1
Hardware Flow Control: None
RX DMA: Enable
TX DMA: Enable
USART10 Global Interrupt: Enable
```

## 4. 初始化

```cpp
#include "dvc_motor_ti.h"

Class_Motor_TI Motor_TI;
```

UART10回调：

```cpp
void UART10_Callback(uint8_t *Buffer, uint16_t Length)
{
    Motor_TI.UART_RxCpltCallback(Buffer, Length);
}
```

初始化：

```cpp
UART_Init(&huart10, UART10_Callback);

Motor_TI.Init(&huart10,
              MOTOR_TI_ENCODER_NUM_PER_ROUND,
              Motor_TI_Direction_NORMAL,
              Motor_TI_Direction_NORMAL);
```

A/B方向需要根据实机安装调整。调整原则：给正目标角速度时，两个轮子的逻辑正方向都应对应整车前进。

## 5. 1ms周期任务

每1ms严格按顺序调用一次：

```cpp
Motor_TI.TIM_1ms_Calculate_PeriodElapsedCallback();
Motor_TI.TIM_1ms_Write_PeriodElapsedCallback();
```

不要在正常控制期间同时重复调用 `Send_PWM_Command()`。

## 6. 第一次安全通信测试

初始化后默认：

```text
控制方式：PWM
A目标PWM：0
B目标PWM：0
```

因此保持1ms回调即可验证通信，不会主动让电机转动。

观察：

```cpp
bool online = Motor_TI.Is_Online();
bool command_active = Motor_TI.Get_Remote_Command_Active();
bool command_timeout = Motor_TI.Get_Remote_Command_Timeout();

float omega_a = Motor_TI.Motor_A.Get_Now_Omega();
float omega_b = Motor_TI.Motor_B.Get_Now_Omega();

float angle_a = Motor_TI.Motor_A.Get_Now_Angle();
float angle_b = Motor_TI.Motor_B.Get_Now_Angle();
```

正常双向通信稳定后：

```text
online          = true
command_active  = true
command_timeout = false
```

## 7. PWM开环模式

```cpp
Motor_TI.Motor_A.Set_Control_Method(Motor_TI_Control_Method_PWM);
Motor_TI.Motor_B.Set_Control_Method(Motor_TI_Control_Method_PWM);

Motor_TI.Motor_A.Set_Target_PWM(200.0f);
Motor_TI.Motor_B.Set_Target_PWM(0.0f);
```

PWM范围：

```text
-4000 ~ +4000
```

建议首次实机从绝对值200开始。

## 8. 角速度模式

单位统一为 `rad/s`。

```cpp
Motor_TI.Motor_A.Set_Control_Method(Motor_TI_Control_Method_OMEGA);
Motor_TI.Motor_B.Set_Control_Method(Motor_TI_Control_Method_OMEGA);

Motor_TI.Motor_A.PID_Omega.Init(
    Kp_A,
    Ki_A,
    Kd_A,
    0.0f,
    I_Out_Max_A,
    D_Out_Max_A,
    4000.0f,
    0.001f
);

Motor_TI.Motor_B.PID_Omega.Init(
    Kp_B,
    Ki_B,
    Kd_B,
    0.0f,
    I_Out_Max_B,
    D_Out_Max_B,
    4000.0f,
    0.001f
);

Motor_TI.Motor_A.Set_Target_Omega(Target_Omega_A);
Motor_TI.Motor_B.Set_Target_Omega(Target_Omega_B);
```

`PID_Omega`输入单位为 `rad/s`，输出单位为PWM。

## 9. 角度模式

角度单位为 `rad`，角速度单位为 `rad/s`。

```cpp
Motor_TI.Motor_A.Set_Control_Method(Motor_TI_Control_Method_ANGLE);

Motor_TI.Motor_A.PID_Angle.Init(
    Angle_Kp,
    Angle_Ki,
    Angle_Kd,
    0.0f,
    Angle_I_Out_Max,
    Angle_D_Out_Max,
    Omega_Max,
    0.001f
);

Motor_TI.Motor_A.PID_Omega.Init(
    Omega_Kp,
    Omega_Ki,
    Omega_Kd,
    0.0f,
    Omega_I_Out_Max,
    Omega_D_Out_Max,
    4000.0f,
    0.001f
);

Motor_TI.Motor_A.Set_Target_Angle(Target_Angle);
```

串级关系：

```text
角度环：rad → rad/s
角速度环：rad/s → PWM
```

## 10. 常用接口

```cpp
Motor_TI.Motor_A.Get_Status();
Motor_TI.Motor_A.Get_Now_Angle();
Motor_TI.Motor_A.Get_Now_Omega();
Motor_TI.Motor_A.Get_Total_Encoder();
Motor_TI.Motor_A.Get_Out();

Motor_TI.Motor_A.Set_PWM_Limit(4000.0f);
Motor_TI.Motor_A.Set_PWM_Change_Max_Per_MS(20.0f);
Motor_TI.Motor_A.Set_Control_Enable(false);
Motor_TI.Motor_A.Clear_Position();

Motor_TI.Stop();
Motor_TI.Ping();

Struct_Motor_TI_Protocol_Statistics statistics =
    Motor_TI.Get_Protocol_Statistics();
```

## 11. 接口语义

- `Is_Online()`：H723最近是否持续收到TI反馈。
- `Get_Remote_Command_Active()`：TI是否持续收到H723的PWM/STOP命令。
- `Get_Remote_Command_Timeout()`：TI是否已经进入100ms命令超时停车。
- `Clear_Position()`：将当前位置设为H723逻辑零点，不修改TI原始累计编码。
- `Stop()`：清空两路控制目标并发送STOP。
- `Send_PWM_Command()`：绕过PID和逻辑方向，仅用于底层测试。
- 离线或控制失能时，库会清除旧控制目标并发送双路零PWM。

## 12. 编码器参数

默认：

```text
1040 count / 输出轴一圈
```

该值来自13线编码器、1:20减速比和软件四倍频：

```text
13 × 20 × 4 = 1040
```

应在实机上将输出轴准确转动一圈，检查累计计数是否约为1040。
