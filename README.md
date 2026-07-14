# NUEDC

全国大学生电子设计竞赛相关的底层驱动、控制模块和测试工程。

## 软件工程

### 1. Motor_F103_TB6612

路径：

```text
Software/Motor_F103_TB6612
```

基于 STM32F103、TB6612 和增量式编码器的单路有刷直流电机测试工程，包含：

- PWM 与方向控制
- TIM2 编码器模式测速
- 1 kHz 速度控制任务
- PID 速度闭环
- Serialplot 串口调参和波形回传
- 正弦速度目标测试

详细说明见：

```text
Software/Motor_F103_TB6612/README.md
```

### 2. MSPM0_TI_ESC

路径：

```text
Software/MSPM0_TI_ESC
```

基于 MSPM0G3507 的双路电机驱动板工程，作为上层主控与电机功率级之间的电调使用，当前设计包含：

- A、B 两路电机 PWM 与方向控制
- C、D 两路驱动保持禁用
- 双路编码器累计计数与速度采集
- 1 Mbps 全双工 UART
- 1 kHz 控制命令与反馈
- CRC16 通信校验
- 通信超时自动停车
- LED 固定心跳指示

工程内的协议、说明见该目录中的中文 Markdown 文档。

## 仓库约定

- 源码、工程配置、原理图和说明文档提交到 Git。
- `build/`、`Debug/`、`Release/` 等构建产物不提交。
- CCS 的 `.project`、`.cproject`、`.ccsproject`、`.settings/`、`targetConfigs/` 和 `.syscfg` 文件需要保留。
- STM32 的 `.ioc`、CMake 配置和用户源码需要保留。
- 每个独立工程尽量在自己的目录中维护 `README.md`。
