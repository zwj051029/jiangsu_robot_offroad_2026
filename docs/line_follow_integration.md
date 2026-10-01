# 四轮白线循迹 APP：接入评审与使用约定

## 新增内容

本工程仍为 STM32F103ZET6，不迁移到 MSPM0。参考以下模块的职责分离，自行实现适配代码：

- [Track](https://github.com/zwj051029/Eletronic_Design_Competition_NJUST_A1/tree/main/ZWJ_MSPM0_Framework/FrameComponets/Apps/Track)：加权线位置与转向控制。
- [SpeedMixer](https://github.com/zwj051029/Eletronic_Design_Competition_NJUST_A1/tree/main/ZWJ_MSPM0_Framework/FrameComponets/Apps/SpeedMixer)：基础 RPM、左右差速和 APP 指令优先级。

`Apps/Inc` 和 `Apps/Src` 新增 Track、SpeedMixer、MotorSpeedDisplay、LineFollowApp、LineFollowBoard。
它们分别负责线位置/恢复策略、带有效期的指令混合、OLED 显示、Application 生命周期及四轮提交、板级绑定。
构造函数不操作硬件；所有操作沿用同一框架任务，不创建新任务，不复制 MSPM0 GPIO 或 RTOS 代码。

## 修改内容：用户已确认并完成

| 既有文件 | 原因与最小改动 | 影响与兼容性 |
| --- | --- | --- |
| Apps/Src/MainFrame.cpp | 四轮初始化后按 APP 选择绑定灰度并初始化 LineFollow；按选择注册新 APP | 原 Offroad 对象、接口、四轮绑定和 PI 参数全部保留；新模式不注册旧停车占位 APP，避免两个应用竞争输出 |
| Sys/Src/RtosCpp.cpp | 按 APP 选择调用 LineFollow.Control；四轮控制后推进 OLED | 保留传感器采集、System/StateCore/Action 调度和原 Offroad 分支；只有新 APP 模式取代旧停车调用 |
| Core/Inc/FreeRTOSConfig.h | INCLUDE_vTaskDelayUntil 从 0 改为 1 | 当前 osDelayUntil 因支持关闭无法等待；启用已有 API 以实现原设计的 1 ms 周期，影响整个默认任务的实际节拍，须回归 |
| MDK-ARM/jiangsu_robot_offroad_2026.uvprojx | 追加五个 APP 源文件及现有 gray_yahboom_8lp、oled_factory、bsp_oled_bus 源文件 | 保持 V6、链接配置、依赖和既有编译项不变；不启用超声波硬件初始化，不重复定义中断 |
| tests/frame/Run-Tests.ps1 | 追加调用新 Run-LineFollowTests.ps1 | 保留原测试命令和测试内容，新行为纳入日常回归 |

不修改 dc_motor、MotorSpeedProfiles、PWM/编码器、灰度/OLED 通用驱动、OffroadApp 公共接口、.ioc 或第三方代码。
`LineFollowBoard::Enabled` 是新增的 APP 选择常量；设为 false 可以恢复原 Offroad 入口。
这是基础白线循迹模式，不能宣称已有三圈、终点、挥手启停、障碍或悬崖策略；这些原扩展点保留。
新模式不使用尚未绑定的超声波，因此不具备避障或超声波启停；后续加入时须明确合入自检及停止条件。

## 控制算法与速度分配

用户确认白线黑底，车头朝前时 x1 在左、x8 在右。沿用灰度驱动 WHITE=1、BLACK=0。
GPIO 顺序为 PC0、PC1、PC2、PC3、PA4、PA5、PB0、PB1；数字黑电平沿用原参考接线 RESET，
实际安装时仍需用白/黑样本确认电平。八路 GPIO 采样 1 ms、两次连续样本消抖。

Track 按左至右 `[-1,1]` 等距权重计算单个连续白线区域的质心，偏左负、偏右正。
5 ms 更新转向，使用实际采样时间差，误差一阶低通时间常数 25 ms，PD 初值 Kp=28、Kd=0.25，Ki=0。
这是整车联调初值，不能把已有轮速 PI 的悬空验收当成循迹外环验收。
转向输出以 RPM 表示，限差速 24 RPM；居中基础 40 RPM，偏差增大逐步降至 28 RPM。

`left=base+diff`，`right=base-diff`；右偏时左侧更快。M1/M2 分配 left，M3/M4 分配 right。
先去除负目标，再按最大侧缩放，使四轮不超过 60 RPM、保持左右比值。车体方向已经由现有驱动映射。
指令 50 ms 过期，跟车偏移必须有新鲜 Track 基础指令；直接指令优先级为 Navigation > Overtake > TurnAround。
这几个名称只预留仲裁接口，当前没有实现导航、超车、原地掉头；负 RPM 被拒绝。
四轮目标上升率 120 RPM/s、下降率 240 RPM/s。故障停止直接 Disable，绕过斜率限制。
调用现有 Enable/SetSpeed，由框架 DcMotor::ControlAllMotors 执行四个独立 PI；绝不周期性 Apply 参数。

## 丢线和异常

- 初始未找到线保持禁能，连续三个新的、可识别的样本才启动四轮。
- 失去可靠线位置后前 120 ms 低速保持最后转向；之后沿最后偏差方向低速寻线。
- 最后位置为居中时交替转弯搜索；寻线期间连续三帧重新看到线即恢复，复位微分避免跳变。
- 失线累计 1200 ms 后四轮禁能、System 进入 STOP；需要显式重新准备/启动，不自动反复启动。
- 全白宽标记及多个分离的白区域视为位置不确定，按上述有界流程处理，不能凭质心假装居中。
- 灰度无效/超过 50 ms、任一编码器离线、输出失败、部分轮意外禁能、控制间隔超过 50 ms 均停止并报错。
- GPIO 无法检测断线后恰好固定电平，通用驱动的硬件限制仍存在。

这些时长是初始整车参数。虚线间隙应按轮径、实测 RPM、传感器前伸距离调整，不保证 120 ms 能跨过比赛的 50 mm 空隙。
终点黑线与无白线不能仅靠这版丢线流程区分，三圈及终点停车需后续独立路线策略。

## OLED 与启动入口

复用 FactoryOled 的 SetLine、Refresh、Update 和 I2C1 PB6/PB7 的唯一 IRQ 实现。
128×32 屏四行显示 `M1 040.2 RPM` 到 `M4 040.2 RPM`（负转速带减号）。显示的是编码器实测低通 RPM，非目标 RPM；
反馈过期或非有限值显示 `---.-`。采用整数十分位格式化，不要求 printf 启用浮点支持。
每 50 ms 请求一帧（20 Hz），每 1 ms 推进非阻塞发送；健康总线下为不低于 10 Hz 留出裕量。
总线故障时保留原驱动的有界重试，不阻塞电机控制；实板仍须测量实际整帧刷新率与控制周期。

上电不行驶。灰度先完成 20 秒预热与 KEY1 标定，再在框架任务中调用
`LineFollowBoard::ConfirmCalibration(true)`；校准前调用 false，已知模块单独重新上电调用 NotifyGrayPowerOn。
GPIO 无法读取校准成功标志，不能默认上电已标定。System 自检进入 READY 后显式调用
`LineFollow.RequestStart()`，它沿用 System 的启动标志。可接后续真实按键/挥手入口，不添加强制启动测试。
STOP/ERROR 后排除原因，再通过现有 System.Init 的准备流程重新自检，之后显式启动。

## 可能受影响内容与验证

回归覆盖轮速 PI/滤波/超时、四轮方向与资源、Gray8LP 256 组合、OLED 总线及错误恢复、
系统自检/启动/停止与任务周期。新测试使用真实 APP、PID、电机、灰度和 OLED 上层代码，硬件回调模拟。
新增测试重点：左右转向符号、四轮映射、启动不动、命令有效期、目标斜率、丢线恢复/超时、
宽标记/分离白线、非法/过期样本、计时回绕、任一轮故障同时停车、实测速度格式及屏幕故障不阻塞控制。

Keil V6 全量 Rebuild 必须 0 Error、0 Warning。硬件需先架空确认四轮速度显示和停车，再低速地面联调：
居中/左右偏移、急弯、短间隙、寻线重获、长时间失线、拔灰度/编码器及断开 OLED。
主机测试和编译不能证明落地无抖动、线路接触正常或刷新周期达标；这些必须记录实车结果。

## 本轮验证记录（2026-10-01）

- Keil ARM Compiler 6 全量 Rebuild：0 Error、0 Warning；日志 MDK-ARM/build-line-follow-v6.log。
- 固件 Code=44978、RO-data=1490、RW-data=776、ZI-data=13904 bytes。
- Run-Tests.ps1（原框架/四轮 PI/灰度 OLED + 新循迹回归）与 Run-Gray8LpTests.ps1 通过；
  新测试以 -Wall -Wextra -Werror 编译。
- 新测试模拟健康总线的整帧刷新间隔为 50～60 ms，并检查发送像素取自实测 RPM，而不是目标 RPM。
- 现有单独 Run-OledTests.ps1 与 Run-UltrasonicTests.ps1 的 BSP 子测试把 .c 当作 C++ 编译，
  在 -Werror 下因 {0} 初始化触发 missing-field-initializers；本轮保留这些既有入口。
  随后使用相同源码、C11/C++11 分离编译且保持 -Wall -Wextra -Werror，OLED 与超声波 BSP 回归均通过。
- 当前可用 clang-format 是 22.1.3，新增/接入 C++ 文件按仓库配置格式化；全仓检查在未修改的
  Algorithm/Inc/hyperPID.hpp 报格式差异；本轮文件的 dry-run/Werror 格式检查通过。
  本轮不重排无关文件，不宣称已通过要求 23+ 的全仓格式验收。
- 未烧录、未进行本轮实车循迹或实板周期测试；参数仍需地面联调。
