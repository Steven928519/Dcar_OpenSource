# Dcar_OpenSource

基于 **Dcar 标准版硬件** 开发的一套 STM32F407 麦克纳姆轮小车开源控制工程。

这个仓库对应的是一套面向教学和实战的完整小车控制代码，目标不是只让轮子转起来，而是把一台四轮麦轮小车从基础驱动、速度闭环、遥控控制，一直搭到 IMU 姿态感知、航向保持和系统调度，形成一套能跑、能调、能继续扩展的工程骨架。

课程讲解网页：
https://differ-tech.pages.dev/df-open-class-inertial-mecanum-car-control/index.html

课程名称：
**点浮公开课：惯导麦轮小车控制**

配套硬件购买：

【淘宝】7天无理由退货  
https://e.tb.cn/h.iIbivwuFlQzzmlO?tk=t0MS5SC6Pop CZ225  
「我在手机淘tao宝发现一个宝贝」  
点击链接直接打开 或者 淘宝搜索直接打开

## 项目定位

本工程适合下面几类使用场景：

- 想基于 Dcar 标准版硬件快速搭建一台可控的麦轮小车
- 想学习 STM32 小车控制系统从开环到闭环的完整实现过程
- 想参考一个已经拆分好模块的嵌入式运动控制工程
- 想把 PS2 遥控、编码器反馈、IMU 姿态和串口上位机控制整合到同一套代码里

本仓库默认硬件基线来自课程体系，代码入口就是这个 `Dcar_OpenSource` 工程。

## 已实现的核心功能

- 四路电机 PWM 驱动与方向控制
- 四路编码器测速
- 电机 PID 速度闭环
- 麦轮底盘运动解耦
- PS2/SBUS 遥控输入解析
- 基于 IMU 航向角的锁头控制
- ICM20602 SPI 通信与姿态解算
- 编码器 + IMU 融合的基础里程计
- USART1 `DMA + IDLE` 上位机速度控制协议
- UART4 调试输出，方便接 VOFA+ / 串口助手观察数据
- 1000Hz / 100Hz / 50Hz 分频任务调度

说明：
当前仓库已经实现串口**速度控制帧**解析，位移控制相关接口已预留结构，但默认主流程里仍以速度控制为主。

## 工程基线

主控平台：
`STM32F407VET6`

更具体一点说，这个工程默认面向的是 **STM32F407VET6** 这颗主控芯片，对应 Dcar 标准版硬件方案。

默认硬件组成：

- Dcar 标准版控制板
- 四个带编码器的驱动电机
- 麦克纳姆轮底盘
- ICM20602 IMU
- PS2 遥控接收机（SBUS）

工程里可以直接看到的外设分配包括：

- `TIM8`：4 路 PWM 电机输出
- `TIM2 / TIM3 / TIM4 / TIM5`：4 路编码器接口
- `TIM6`：1ms 基准中断，用于电机速度环分频更新
- `SPI2`：ICM20602 通信
- `USART2`：PS2 / SBUS 接收
- `USART1`：上位机串口控制
- `UART4`：调试输出

## 代码结构

核心代码主要集中在 `Core/Src` 和 `Core/Inc`：

- `main.c`：系统初始化与主循环入口
- `motor_control.c`：四电机速度环控制
- `encoder.c`：编码器测速
- `pid.c`：PID 控制器
- `remote_to_motion.c`：遥控量映射与麦轮解耦
- `ps2_receiver.c`：SBUS 数据接收与解析
- `imu.c`：ICM20602 驱动与 Mahony 姿态解算
- `odometry.c`：基础里程计
- `loop.c`：分频任务调度
- `usart1_control.c`：上位机串口协议解析

## 如何使用

### 1. 克隆仓库

```bash
git clone git@github.com:Steven928519/Dcar_OpenSource.git
```

### 2. 打开工程

你可以根据自己的开发环境选择：

- `DCar_OpenSource.ioc`：使用 CubeMX 查看或调整外设配置
- `MDK-ARM/DCar_OpenSource.uvprojx`：Windows 下可直接使用 Keil MDK 打开
- `Makefile`：适合 `macOS` 下配合 ARM GNU Toolchain 编译，推荐 Mac 用户优先使用这种方式

### 3. 编译

如果你本地已经配置好 ARM GNU Toolchain，可以直接：

```bash
make
```

或者使用仓库自带脚本：

```bash
./build.sh
```

如果需要烧录：

```bash
./build.sh flash
```

说明：
`Makefile` 里当前写死了本地工具链路径 `/Applications/ArmGNUToolchain/15.2.rel1/arm-none-eabi/bin`，如果你的环境不同，需要先改成你自己的路径，或者让 `arm-none-eabi-*` 进入 `PATH`。

建议：

- `macOS`：推荐优先使用 `Makefile` / `build.sh` 这套方式，和当前仓库结构更匹配
- `Windows`：使用 Keil、CubeMX 或 ARM GNU Toolchain 都可以，按你自己的开发环境选择即可

## 这个工程能学到什么

- STM32 小车控制项目如何做模块拆分
- 编码器测速与速度闭环如何搭建
- 麦轮小车的平移 / 侧移 / 旋转是怎么解耦到四个轮子的
- 为什么要做 IMU 航向保持，以及它怎么接入控制闭环
- 为什么串口协议接收更适合用 `DMA + IDLE`
- 一个小车工程如何从“单个功能 Demo”组织成“持续运行的系统”

## 对应课程

课程总目录：
https://differ-tech.pages.dev/df-open-class-inertial-mecanum-car-control/index.html

课程内容围绕下面这条主线展开：

- GPIO 与基础外设
- PWM 与电机驱动
- 编码器测速
- PID 速度闭环
- 麦轮运动学解耦
- PS2 遥控控制
- IMU 姿态感知
- 航向保持
- 任务调度
- 串口通信协议

如果你是跟着课程学习，建议直接把这个仓库作为课程代码主入口。

## 说明

- 本仓库以 Dcar 标准版硬件为默认基线，不保证直接兼容所有自定义底盘和接线方式
- 如果你修改了电机安装方向、编码器极性、轮径或减速比，需要同步调整代码里的相关参数
- 该工程更偏教学与工程实践结合，适合二次开发和继续扩展

## 仓库地址

GitHub：
https://github.com/Steven928519/Dcar_OpenSource
