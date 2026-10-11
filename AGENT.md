# 多人协作开发指南

> 当前分支：`robot/mine`，设备归属项目发起人，本车四轮均未调参。
> 按用户要求，PID 与全部记录暂时原样沿用张诗筠的车辆；下文“已调/已验证”仅指来源车辆，不代表本车。
> 分支用途及参数来源见 [设备归属](ROBOT_VARIANT.md)。

本项目由三人协作：项目发起人负责直流减速电机，一位成员负责超声波，另一位成员负责灰度传感器。
本文供三位开发者及协助开发的 AI 阅读。开始修改前先阅读本文和 [框架说明](FrameComponets/README.md)。

## 1. 当前工程状态

- 当前交付四轮 PID 清理版：上电禁能，恢复 LineFollowBoard 选择的应用；启动需校准、自检和显式请求。
  自动持续试转入口已删除；MotorContinuousTest 仅位于 tests/frame/fixtures，不参与固件编译。
  Nova 蓝牙通信与调参协议保留，`NOVA_MOTOR_TUNING=0` 默认关闭 AI 运动控制。
  四轮正转 PID、100 RPM 实测和数据来源统一记录于 [四轮参数记录](docs/motor_pid_20261011.md)。
  PWM/积分限幅仍为 69%，原 MotorProtection、超时、故障锁定和停稳互锁保持不变。

- 主控 STM32F103ZET6，72 MHz，Cortex-M3，无 FPU；Keil ARM Compiler V6，当前本机 V6.14.1。
- FreeRTOS 使用 `portable/GCC/ARM_CM3`，默认任务通过 osDelayUntil 每 1 ms 推进框架，TIM6 是 HAL 时间基准。
- V1_main 的 F407 框架已适配 F103，原 Libs 对应本项目 Bsps。
- 四路电机为 520、30:1、11 线，四倍频为输出轴 1320 计数/圈。用户已确认四路单轮测试正常。
  M1 左前、M2 左后、M3 右前、M4 右后；方向集中在 MainFrame，M4 原始计数不取反。
  初始 PI 为 Kp=0.0012、Ki=0.008、Kd=0，滤波 0.03 秒、PWM/积分限幅 0.69、目标上限 180 RPM。
  M1 正转已固化为 0.000957/0.009191/0（2026-10-11、100 RPM 台架复测）；M3 正转固化为 0.001176/0.013926/0；M2 正转固化为 0.001329/0.010857/0；M4 正转固化为 0.001442/0.010392/0；四轮反转仍未调参。
  旧 310 实测记录只作历史，不能套用到 520。
- 用户已确认八路灰度 OLED 测试正常。接线 x1～x8 为 PC0/PC1/PC2/PC3/PA4/PA5/PB0/PB1。
  正式循迹仍需要预热、操作者确认 KEY1 校准和新鲜有效样本；不自动伪造校准成功。
- 保留原厂超声波驱动并在 MainFrame 绑定：PF11 TRIG、PF12 ECHO、TIM7、EXTI12、DWT。
  OLED 为 I2C1 PB6/PB7，SSD1306 128×32、地址 0x3C。中断只保留一个实现。
- 已按用户要求清理固件中的电机自动试转、超声波距离显示、灰度显示入口和测试宏。
  Motor520Test / GrayOledTest / RangeDisplayApp 仅位于 tests/frame/fixtures，用于主机回归，不参与 Keil 编译。
- 原试车保护已抽为正式 `Apps/MotorProtection`，全局 `MotorSafety` 在默认任务中包围控制周期。
  保留方向、无转速、超速、反馈/驱动故障、50 ms 调度中断及故障锁定，异常禁能四轮。
  停稳等待接口保留 1 秒最小滑行、低于 6 RPM 持续 100 ms、3 秒超时；不会自动反向或重启。
  使用与恢复步骤见 [电机保护](docs/motor_protection.md)。DcMotor 原命令超时、限幅和输入检查保留。
- 恢复原 LineFollowBoard 选择的应用（目前循迹 Enabled=true），上电禁能并等待校准、自检和显式启动。
  循迹原 OLED 轮速显示保留；不再显示临时灰度/超声波诊断界面。
  OffroadApp 仍未实现完整越野比赛策略，挥手启动、障碍及整车联合验证仍待开发。
- 清理版通过主机回归和 V6 完整重编译，0 Error / 0 Warning；测试和编译不代替整车实测。

资源表和各模块接口详见 [框架说明](FrameComponets/README.md) 与 [电机说明](FrameComponets/Mods/MOTOR.md)。
后续 CubeMX 生成要同时检查 BSP 资源占用，不重复初始化 TIM7、I2C1 或同名 IRQ。

## 2. 分工及修改范围

以下路径均相对于工程根目录，即本文件所在目录。

| 负责人 | 主要文件 | 可复用的底层支持 | 交付目标 |
| --- | --- | --- | --- |
| 电机负责人（项目发起人） | `FrameComponets/Mods` 的 `dc_motor`、`motor_pwm_driver`；`Bsps` 的 `bsp_motor_board` | PWM、编码器及 `Algorithm` 的 PID | 实测正反转、停止、RPM，再调速度 PID |
| 超声波负责人 | `FrameComponets/Mods/Inc/ultrasonic.hpp`、`Mods/Src/ultrasonic.cpp` | `Bsps` 的 DWT、GPIO/EXTI、UART；按型号补充捕获支持 | 非阻塞测距、新数据时间戳、超时及无效测量处理 |
| 灰度负责人 | `FrameComponets/Mods/Inc/gray_sensor.hpp`、`Mods/Src/gray_sensor.cpp` | `Bsps` 的 GPIO、ADC、UART | 多通道采样、标定、归一化、完整新帧交付 |

表中简写的 `Mods/...`、`Bsps/...`、`Algorithm/...` 均位于 `FrameComponets` 下。
需要增加文件时，优先放在本人模块对应的 `Inc` 和 `Src`，使用模块名称前缀。
不要在其他成员的模块中顺手重构；共享接口变更应在合并前说明原因和调用方影响。

| 层 | 应放置的内容 |
| --- | --- |
| Bsps | MCU 外设操作，不包含比赛判断和具体传感器协议 |
| Mods | 具体设备驱动、协议、标定、数据有效性及设备接口 |
| Algorithm | PID、滤波、矩阵等可复用算法，不硬编码 GPIO 或比赛路线 |
| Apps | 板级绑定、挥手启动/停止、循迹、虚线、障碍、圈数和终点策略 |
| Sys | 应用生命周期、自检、调度、状态机、动作和日志 |

优先保留现有公开接口。模块正常开发不需要各自改写 `System` 或另建一套调度器。

## 3. 先约定硬件资源，再写驱动

已确定的电机资源如下，其他成员不得重复占用。尚未确定的资源保持“待定”。
电机初始化当前由板级 BSP 管理，未修改 `.ioc`；其他外设确定后同步更新本文和 `.ioc`。

| 用途 | GPIO / 外设 / 通道 | 负责人 | 备注 |
| --- | --- | --- | --- |
| 左前 M1 | PC6/PC7，TIM8 CH1/CH2；PD12/PD13，TIM4 CH1/CH2 | 电机 | AT8236，20 kHz，双低滑行；正反转方向已实测 |
| 左后 M2 | PC8/PC9，TIM8 CH3/CH4；PA15/PB3，TIM2 CH1/CH2 | 电机 | 四轮均为 520，减速比 30，11 线四倍频，输出轴 1320 计数/圈 |
| 右前 M3 | PE9/PE11，TIM1 CH1/CH2；PA0/PA1，TIM5 CH1/CH2 | 电机 | 官方输出极性：左侧反向、右侧不反向；反馈左正右负 |
| 右后 M4 | PE13/PE14，TIM1 CH3/CH4；PB5/PB4，TIM3 CH2/CH1 | 电机 | 方向表直接针对 TIM 原始计数，已涵盖实际 A/B 布线 |
| 蓝牙 / Nova | USART1 PA9 TX / PA10 RX；DMA1 CH4 TX / CH5 RX | 共享 | 115200 8N1，bsp_uart1_dma 独占；板载 CH340 共用网络，避免 TX 冲突；使用 DAPLink 下载 |
| 下载 / 调试 | PA13 SWDIO / PA14 SWCLK，SWD | 共享 | 当前 DAPLink，Keil 选择 CMSIS-DAP Debugger；保留 SWD 引脚，不得复用 |
| 原厂超声波 | PF11 TRIG、PF12 ECHO / EXTI12，TIM7 | 超声波 | 原厂接口，20 us 触发、60 ms 间隔、35 ms 超时；DWT 采集回波 |
| 灰度阵列 | x1～x8：PC0/PC1/PC2/PC3/PA4/PA5/PB0/PB1 | 灰度 | 用户确认接线；8-LP 数字输入，OLED 按 x1～x8 显示；车体左右方向待安装确认 |
| 原厂 OLED | I2C1 PB6/PB7，0x3C，128×32 | 显示 | 保留可复用驱动和循迹轮速显示；I2C1 IRQ 由 bsp_oled_bus 独占 |
| 状态 LED | 待定 | 三人协商 | 写明引脚和有效电平 |
| TIM6 | HAL 时间基准，已占用 | 共享 | 不得用于电机、编码器或超声波 |

分配时同时检查定时器通道、UART、DMA 通道、ADC 通道、AFIO 重映射和 EXTI 线是否冲突。
TIM1/8 和 TIM2/3/4/5 保留给电机；TIM1/4 全重映射、TIM2 部分 1、TIM3 部分重映射已启用。
JTAG 已关闭以使用 PA15/PB3/PB4，SWD 保留。其他模块不得改回相关 AFIO 设置。
不同 GPIO 端口的同一引脚号共享 EXTI 线，不能当作两条独立外部中断使用。
具体接线和电气兼容性依据实际板卡、模块资料核对。

`.ioc`、`Core` 外设初始化、中断文件及 `Apps/Src/MainFrame.cpp` 是共享修改点。
同一轮硬件配置由一人负责生成和合并，其他成员提交所需配置清单，避免同时生成工程覆盖彼此修改。

## 4. 电机模块怎么开发

现有库的初始化、API、独立实测步骤及 PID 接入顺序见 [电机说明](FrameComponets/Mods/MOTOR.md)。
`Init()` / `Bind()` 默认禁能；占空比模式需显式 `Enable()`。
`Neutral()` / `Stop()` 清空目标并滑行，保留使能；`Disable()` 同时禁能。
`FrameTickCpp()` 每 1 ms 调用批量控制，命令默认 250 ms 过期。
当前固件无临时试车入口；通用库默认增益为零，板级显式配置各轮 520 已调参数。
本板初始化显式应用各轮已调前进 PI，保持禁能；`MotorSpeedProfiles::Apply(motor, port, backward)`
只接受已初始化、禁能的电机，配置时清除旧目标和积分，完成后仍需显式 Enable。
不能在每次提交目标时重复 Apply；使用后退参数时也必须先停车、禁能再切换。
当前参数要求 `deadzone_duty=0`，不能叠加旧固定补偿。
电机只由同一默认任务控制，不能让其他任务同时写输出。

现有 `DcMotor::Driver` 通过回调隔离实际驱动板：

```cpp
bool SetDuty(void *context, float signed_duty);
void Stop(void *context);
bool ReadRpm(void *context, float *rpm); // 可选
```

- `set_duty` 和 `stop` 必须提供；`read_rpm` 可以为 `nullptr`。
- 占空比接口范围为 `[-1, 1]`；模块已负责限幅、非有限值检查和失败后停止。
- 团队方向约定：正值驱动车辆前进，负值后退。左右电机安装极性差异在各自驱动适配器内处理。
- `stop` 必须能够可靠停止且允许重复调用；按实际驱动板明确刹车或滑行行为。
  `Bind()` 也会调用停止接口，驱动初始化时应已经能够安全执行停止。
- 回调失败返回 `false`，不要假装已经输出成功；未就绪或故障时不得继续保持旧的驱动输出。
- 换向死区、方向脚顺序、PWM 启停及故障处理放在设备驱动内，不能靠上层调用者记住这些细节。
- 轮速反馈统一使用输出轮的 RPM，正值为前进。明确编码器每圈有效计数是否已经包含倍频和减速比。
  没有编码器或当前没有可信数据时返回 `false`，不能用目标转速冒充实测值。
- 编码器采样应足够频繁，避免一次运动超过计数周期的一半；按实际 ARR 处理回绕。
- 已复用 `Algorithm` 的 PID，每个电机单独保存状态；默认 RPM 反馈周期 10 ms、新鲜度 50 ms。
  PID 参数单位基于 RPM 和秒，输出为占空比；不得把主机合成测试增益当作实车调参结果。

验收至少覆盖：左右轮正反转、零占空比、限幅、重复停止、换向、驱动失败，
以及存在编码器时的轮速方向、单位和回绕。先架空车轮测试，再进入整车联调。

## 5. 超声波模块怎么开发

现有采集回调签名：

```cpp
bool ReadRange(void *context, Ultrasonic::Sample *sample);
```

`Sample` 的字段为 `distance_mm`、`timestamp_ms` 和 `valid`。

- 距离单位统一为毫米，正常测量必须是有限且大于零的值。
- 回调返回 `true` 表示交付了一次新的测量结果；还没有新结果时返回 `false`。
- 已完成但失败的测量，交付 `valid = false` 并返回 `true`，使模块及时作废旧结果。
  持续没有新结果时，旧结果通过新鲜度检查过期。不要将 `false` 当作清除旧样本的操作。
- `timestamp_ms` 使用 `HAL_GetTick()` 同一时间基准，记录该次测量完成/接收的时刻。
  不得因为上层又调用一次 Reader 就刷新旧数据的时间戳。
- 根据型号实现触发间隔、等待回波、上升/下降沿、超时和计数回绕。
  每 1 ms 调用采集入口不代表每 1 ms 发起测量；触发频率由驱动自行管理。
- 回调应非阻塞：在任务中推进状态并取走完成结果；不要循环等待回波或调用 `HAL_Delay()`。
- 中断只记录原始边沿、脉宽或接收数据，距离换算和协议处理尽量放在任务中。
- 默认样本新鲜度为 200 ms；如实际采样周期需要调整，要同步说明原因并修改检查调用。
- 挥手阈值、防抖、启动与非接触停机判断属于 `OffroadApp`，不能在测距驱动里直接切换系统状态。

验收至少覆盖：已知距离、无回波、超时、异常帧/异常脉宽、计数回绕、最小触发间隔，
以及连续读取不会重复刷新旧样本时间戳。

## 6. 灰度模块怎么开发

现有采集回调签名：

```cpp
bool ReadGray(void *context, GraySensor::Sample *sample);
```

`Sample` 的字段为 `line[]`、`channels`、`timestamp_ms` 和 `valid`。

- 按实际模块选择 GPIO、ADC 或 UART 采集，通道数不能超过 `FRAME_GRAY_MAX_CHANNELS`（当前 16）。
- 团队通道顺序约定：车头朝前时，从车体左侧到右侧依次放入 `line[0..channels-1]`。
  现有 `GetLineError()` 因此产生左负、右正的加权位置误差；PID 转向符号由上层联调确认。
- 标定后的值统一为 `[0, 1]`：白色引导线趋近 1，背景/黑色趋近 0。
  原始 ADC 值、数字有效电平或协议数值的差异在驱动内转换。
- 写明逐通道标定方法和保存方式；数字型传感器可以输出 0/1，但不能声称具有模拟分辨率。
- 交付一帧完整数据，不能把不同采样时刻、半更新的通道数组直接暴露给上层。
- Reader 的返回值和无效样本约定与超声波相同：无新帧返回 `false`；新帧失败则返回 `true` 且 `valid = false`。
- 时间戳使用与 `HAL_GetTick()` 一致的毫秒基准，只更新新帧；默认新鲜度为 50 ms。
- `GetLineError()` 返回 `false` 表示当前数据不能计算线位置，不等于已经确定“驶离赛道”。
  虚线空白、黑色终点线和真正丢线的区分属于 Apps。
- 当前 ADC BSP 只有单通道单次读取；多通道扫描/DMA 需要补充适配器，不能把阻塞扫描塞进 1 ms Reader。

验收至少覆盖：通道顺序、白/黑标定、线在左/中/右时的误差、全背景、全白、非法数值、
缺帧及数据过期。使用比赛场地相近的光照和安装高度做实测。

## 7. 绑定、调度及中断约定

三人交付驱动上下文及回调后，由当次集成人在 `Apps/Src/MainFrame.cpp` 的 `BindHardware()` 中绑定：

```cpp
// Motor1..4 已在 BindHardware() 中初始化并绑定，上电保持禁能。
// 当前恢复原应用，启动需校准、自检与显式请求；MotorSafety 在每周期检查四轮。
// 以下采集上下文及回调由传感器负责人提供，仍为示意名称。
RangeSensor.Bind(&range_context, ReadRange);
GrayArray.Bind(&gray_context, ReadGray);
```

- 驱动上下文、BSP 注册对象、接收缓冲区等必须保持静态或全程有效，不能绑定函数退出即失效的局部对象。
- 当前初始化在 `main()` 的 HAL、时钟、GPIO 初始化之后、RTOS 调度开始之前进行。
  电机由 `BspMotorBoard_Init()` 完成初始化；不要再运行重复的 `MX_TIM*_Init()`。
  其他新增外设先执行对应的初始化，再执行 BSP/驱动初始化和绑定。
- 现有基础周期为 1 ms：更新 DWT、传感器采样，随后按分频更新系统，最后执行控制。
  应用、系统、状态机和动作的周期为 5 ms；状态机/动作仅在 WORKING 时运行。
  最后调用 `DcMotor::ControlAllMotors()`，禁能状态也采集编码器。
  临时试车入口已清理；NovaLink 在每周期保护检查前处理有界通信，故障时仍可接收停车命令。
  蓝牙连接不自动启动电机；单轮模式仅在完成限幅配置、停稳并显式 set_target 后启动。
  参数仅在禁能时更新（可以正在滑行），重新使能仍须停稳；原保护和故障锁定不能绕过。
  循环使用 osDelayUntil，上述周期是设计目标，不能当作实测保证。
- Reader 和控制回调应及时返回，不得在这条共享循环中等待硬件、执行阻塞日志或使用 `Seq::Wait/WaitUntil`。
- 当前框架注册及应用操作由同一任务执行，不得从 ISR 修改注册表、调用 Monitor 或触发整套系统更新。
- ISR 和任务之间的多字段数据交接要使用短临界区、双缓冲等明确同步方案；仅加 `volatile` 不能保证整帧一致。
- GPIO/UART 的 HAL 回调已经在 BSP 中定义，先检查现有分发入口再增加模块回调。
  不得在各模块重复定义同名 `HAL_GPIO_EXTI_Callback`、`HAL_UART_RxCpltCallback` 或 RX Event 回调。
- 原 bsp_uart 仍提供普通 DMA；USART1 由独立 bsp_uart1_dma 实现循环 DMA 增量处理，不能重复注册。
  USART1 / DMA1 CH4/5 的 IRQ 在该 BSP 中唯一实现，不占用已有 HAL UART 回调。
  Bluetooth 负责透传，NovaProtocol 负责协议，NovaLink 负责四轮/循迹参数绑定；详见 [蓝牙与 Nova](docs/bluetooth_nova.md)。
  可选单轮调参模式（默认关闭）已绑定 configure/heartbeat/set_target，仅 `NOVA_TUNING_PORT` 指定通道宣告完整自动能力。
  所选电机使用 DcMotor 可选 D-on-measurement；其他电机和比赛模式仍使用原 D-on-error，不得混淆参数。
  心跳 0.8～2 秒、单轮 45 秒硬上限、目标绝对上限 150 RPM、PWM 硬上限 0.69；异常锁定至复位。
  STOP 立即禁能，后续启动仍等待原停稳互锁；配置与参数下发不会自行启动。
  蓝牙实物已确认是 DX-BT24 BLE 5.1，FFE0 服务、FFE1 通知、FFE2 写入；电脑使用
  `tools/ble_bridge` 桥接，桥接占 COM11、Nova 占 COM12，COM5 属于 DAPLink。
  桥接及固件只读查询已实测双向成功；桥接不自动重连/重放命令，不代替设备端停车保护。

模块 `IsReady()` 当前主要检查回调是否绑定，不代表线路正常或测量有效。
整车自检还要求灰度和超声波有新鲜有效样本；运行中数据过期会触发系统停止。
不要为绕过自检填入假样本或将 `WatchPoint()` 改为恒真。

## 8. 比赛策略与公共文件

比赛规则和具体策略集中在 Apps：白线循迹、三圈、挥手启动/非接触停机、虚线、障碍和终点停车。
准备完成后 LED 常亮、行驶时有节奏闪烁的逻辑已经在 System 中。
各模块交付可信设备数据和控制接口，路线判定、阈值和状态切换由集成时共同完成。
遇到规则细节不明确时查阅实际比赛文件，不在驱动层自行假定完成圈数或越障成功。

修改公共代码前，在变更说明里列出调用方及影响：

- 改接口签名/单位：同时更新调用方、本文和相关测试。
- 改采样周期/超时：说明三个模块的影响和实测依据。
- 改 `.ioc` 或 `Core`：说明新增初始化、引脚、中断、重映射及 DMA 分配。
- 改第三方 `Drivers`/`Middlewares`：须有具体必要性，普通设备开发优先在 BSP/Mods 实现。
- 新增 HAL 回调：沿用统一分发，保留其他模块已有处理。

## 9. 编码规范与提交前检查

沿用 `.clang-format`：4 空格、Allman 大括号、每行最多 100 字符。
头文件声明接口，普通实现放 `Src`；模板和简单访问器可保留在头文件。
变量及接口名称表达用途，注释写清单位、方向、有效电平、数据生命周期和限制。
禁止大段压成一行；不要保留调试用强制启动、假数据或无说明的魔法数。

从工程根目录运行：

```powershell
.\tools\Format-Code.ps1
.\tools\Format-Code.ps1 -Check
.\tests\frame\Run-Tests.ps1
```

格式化需要 clang-format 23 或更新版本；测试需要可用的 `g++`。
没有加入 PATH 时分别通过 `-Formatter`、`-Compiler` 指定可执行文件完整路径。
新增主机可测试的源文件或改变接口时，同步维护 `tests/frame/Run-Tests.ps1` 及对应测试。
主机模拟不能替代硬件采样、电机方向、接线和实时性能验证。

新增框架源文件或 CubeMX 重新生成后，先关闭 Keil 工程，再运行：

```powershell
.\tools\Sync-FrameProject.ps1
```

该脚本同步五层 `Src` 文件、Include Paths、HAL ADC/UART 支持和 V6 的 FreeRTOS 移植路径。
新引入的其他 HAL 外设源文件及编译宏需另外检查。
若新增 ADC/定时器等头文件，源代码也要包含相应 CubeMX 生成的声明头文件。
最后用 Keil 打开 `MDK-ARM/jiangsu_robot_offroad_2026.uvprojx`，选择 V6，执行完整 Rebuild，
提交前要求 0 Error、0 Warning。仅做文档修改时不必重新编译固件。

CubeMX 修改尽量放在 USER CODE 区。重新生成后还需检查 `MainInitCpp()`、`RobotSystemCpp()` 调用、
512 words 默认任务栈、8192 bytes RTOS 堆、唯一 GCC `port.c` 和新增外设初始化顺序。

## 10. 协作提交与交付内容

各人使用独立功能分支，例如 `feature/motor`、`feature/ultrasonic`、`feature/gray`。
不要同时修改同一套 CubeMX/Keil 配置；合并前由当次集成人协调共享文件。
提交只包含本人模块、必要的共享变更和验证资料，不夹带其他成员未完成的修改。

每次模块交付至少附上：

1. 硬件型号、接线和已使用资源，更新上面的资源表。
2. 初始化/绑定入口、接口单位、方向、周期及失败处理。
3. 修改文件清单，特别说明 `.ioc`、Core、公共 BSP 和接口的变更。
4. 格式检查、主机测试及 V6 Rebuild 结果；硬件实测结果与未验证项分别写明。
5. 可复现的测试步骤，以及仍待整车联调的参数和 TODO。

提交前用 `git status`、`git diff` 检查内容。根目录 `.gitignore` 已排除 Keil 编译目录、
AXF/HEX、对象文件、日志、测试输出和个人 IDE 配置；`.ioc`、`.uvprojx`、启动汇编、
RTE 头文件及 CMSIS 的 `.lib/.a` 依赖仍应提交。
不要用 `git add -f` 绕过忽略规则加入个人配置或编译产物，也不要夹带他人未完成的修改。

详细的移植清单、算法说明和硬件限制见 [框架说明](FrameComponets/README.md) 与
[Algorithm 说明](FrameComponets/Algorithm/README.md)。
