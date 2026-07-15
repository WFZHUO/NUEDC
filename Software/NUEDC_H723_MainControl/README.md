# NUEDC H723 Main Control

本工程由 RoboMaster 哨兵 H723 底层工程复制得到，用作电赛小车整车主控。

- 主控：达妙 MC02 / STM32H723
- 所属仓库：NUEDC
- 与原 RoboMaster 哨兵仓库相互独立
- 当前保留原工程的 HAL、CMSIS、CMSIS-DSP、CMSIS-NN、CMSIS-RTOS、USB 中间件及全部用户代码，便于后续扩展
- 已移除原仓库 `.git/` 与本地 `build/` 编译目录

## 构建

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

构建结果生成在 `build/Debug/`，由 NUEDC 根目录 `.gitignore` 忽略。
