# NUEDC

全国大学生电子设计竞赛相关的软件、硬件、底层驱动、控制模块和测试工程。

## 目录结构

```text
NUEDC/
├─ Hardware/
├─ Software/
│  ├─ Motor_F103_TB6612/
│  ├─ MSPM0_TI_ESC/
│  └─ NUEDC_H723_MainControl/
├─ .gitignore
└─ README.md
```

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

> 上电连接电机前，应先确认是否启用了自动正弦速度测试，避免电机意外转动。

---

### 2. MSPM0_TI_ESC

路径：

```text
Software/MSPM0_TI_ESC
```

基于 MSPM0G3507 的双路电机驱动板工程，作为 H723 上层主控与有刷电机功率级之间的电调使用。

当前设计包含：

- A、B 两路电机 PWM 与方向控制
- C、D 两路驱动保持禁用
- 双路编码器累计计数与速度采集
- 500 Kbps 全双工 UART
- 1 kHz 控制命令与反馈
- CRC16 通信校验
- 通信超时自动停车
- LED 固定心跳指示

控制架构：

```text
H723 主控
  ├─ 速度 PID
  ├─ 整车控制逻辑
  └─ 通过 UART 下发 PWM
          ↓
MSPM0 电机驱动板
  ├─ PWM / DIR 输出
  ├─ 编码器采集
  └─ 上传计数与速度反馈
```

工程内的协议和使用说明见该目录中的 Markdown 文档。

---

### 3. NUEDC_H723_MainControl

路径：

```text
Software/NUEDC_H723_MainControl
```

基于达妙 MC02 和 STM32H723 的电赛小车整车主控工程。

该工程由 RoboMaster 哨兵工程的达妙板底层框架复制并独立整理而来，用于电赛整车控制。复制后的工程属于 NUEDC 仓库，与原 RoboMaster 哨兵仓库相互独立。

当前定位：

- 作为电赛小车的上层主控
- 与 MSPM0 电机驱动板进行 500 Kbps UART 通信
- 接收双路编码器和速度反馈
- 在 H723 上运行速度 PID
- 后续实现底盘运动控制
- 后续接入导航、视觉或其他上层控制指令
- 后续增加整车状态管理、失联保护和急停逻辑

工程保留了原达妙板项目中的底层驱动、STM32 HAL、CMake 配置和相关中间件，电赛业务逻辑应在该工程内独立开发，不直接修改原 RoboMaster 哨兵仓库。

#### 编译方式

建议使用 VSCode 单独打开：

```text
Software/NUEDC_H723_MainControl
```

然后使用 CMake Tools 选择 `Debug` 或 `Release` Preset，也可以在工程目录执行：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

编译输出位于：

```text
build/Debug
```

`build/` 为构建产物，不提交到 Git。

## 工程关系

```text
Robomaster_XinxiangUniversity_2027_Sentry
└─ 原始 RoboMaster 哨兵工程，独立仓库

NUEDC
└─ Software/NUEDC_H723_MainControl
   └─ 从哨兵底层框架复制出的电赛主控工程
```

- 后续公共底层模块的修复需要人工选择性同步。

## 仓库约定

- 源码、工程配置、原理图和说明文档提交到 Git。
- `build/`、`Debug/`、`Release/` 等构建产物不提交。
- `.o`、`.elf`、`.out`、`.bin`、`.hex`、`.map` 等编译输出不提交。
- CCS 的 `.project`、`.cproject`、`.ccsproject`、`.settings/`、`targetConfigs/` 和 `.syscfg` 文件需要保留。
- STM32 的 `.ioc`、`CMakeLists.txt`、`CMakePresets.json`、`cmake/` 和用户源码需要保留。
- 仓库统一使用根目录 `.gitignore` 管理忽略规则。
- 每个独立工程应在自己的目录中维护 `README.md`。
