# MSPM0 双电机驱动板工程说明

本工程仅使用驱动板的 **A、B 两个电机通道**。

- 电机 C、D 通道保持休眠，不输出 PWM；
- 串口波特率为 **1,000,000 baud，8N1**；
- TI 板每 **1 ms（1 kHz）**发送一次反馈；
- 达妙 H723 向 TI 板发送两个电机的 PWM 目标值；
- TI 板负责产生 PWM、控制方向、采集编码器并回传数据；
- 速度闭环计划放在上层达妙 H723 中完成。

## 程序运行逻辑

1. MSPM0 上电后初始化时钟、GPIO、UART、电机 PWM 和编码器；
2. PA6 上的 LED 每 500 ms 翻转一次：
   - 亮 500 ms；
   - 灭 500 ms；
   - 完整闪烁周期约 1 秒，即约 1 Hz；
3. UART 接收两个有符号 PWM 命令；
4. PWM 命令范围为 `-4000 ~ +4000`：
   - 正负号决定转向；
   - 绝对值决定占空比；
   - `4000` 表示 100% 占空比；
5. 编码器 A/B 相采用双边沿解码；
6. 每 1 ms 发送一次反馈，反馈中包含：
   - A 电机累计编码值；
   - B 电机累计编码值；
   - A 电机速度；
   - B 电机速度；
   - 状态字节；
7. 若超过 100 ms 没有收到有效的 PWM 或 STOP 命令，两个电机自动停止。

## 工程文件结构

```text
main.c          程序入口、1 ms任务调度、中断入口、LED心跳
protocol.c/h    双电机串口协议、反馈发送、超时停车
bsp_board.c/h   电源、时钟、LED以及各模块初始化
bsp_uart.c/h    1 Mbps UART收发驱动
bsp_motor.c/h   A/B电机PWM、方向和驱动器使能
bsp_encoder.c/h A/B编码器解码、累计计数和速度计算
crc16.c/h       Modbus CRC16校验
app_config.h    引脚映射、频率和协议常量
board.syscfg    CCS SysConfig配置文件
```

## 各说明文件

- `HOST_PROTOCOL.md`：达妙 H723 与 TI 板之间的完整串口协议；
- `MOTOR_WIRING.md`：电机接口、编码器线序和供电注意事项；
- `TEST_FRAMES.md`：可直接复制到串口助手中的测试帧；
- `REVIEW_NOTE.md`：当前版本的修改和审核记录。
