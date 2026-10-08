# PosePilot

**全机载单人视觉追踪研究 / Onboard single-person visual pursuit**

[项目网站](https://jonatl5.github.io/posepilot/) · [English overview](#english-overview)

PosePilot 研究如何将人体姿态感知、紧凑状态估计与强化学习控制结合，让无人机跟随场景中的单一人员。感知与策略推理计划全部在机载 Raspberry Pi 5 上运行，通过有线 MAVLink 链路向 PX4 飞控发送高层设定值。

**当前状态：方案设计与项目介绍网站。** 本仓库包含网站源码与研究方案，不包含已实现的飞行控制器、训练好的 PPO 模型或实测飞行结果。网站上的交互可视化使用合成关键点，不运行摄像头、MediaPipe 或 PPO。

## 最新方案

| 层级 | 选定方案 | 职责 |
| --- | --- | --- |
| 飞行平台 | Holybro X500 V2 开发套件，Pixhawk 6C 配置 | 开放研究机架与 PX4 飞控 |
| 机载计算 | Raspberry Pi 5，4 GB 作为起点 | Pose、状态估计、策略推理、安全状态机 |
| 视觉输入 | Camera Module 3 标准版 | 通过 CSI-2 直接连接 Pi；Pi 5 需 Standard–Mini 15 针转 22 针相机线 |
| 感知 | MediaPipe Pose Landmarker，`num_poses = 1` | 提取 33 个姿态关键点 |
| 状态估计 | EMA / Kalman，方案待比较 | 中心、视觉尺度、变化率与观测有效性 |
| 学习策略 | PPO Actor，小型 MLP，待训练 | 前后速度和转向角速度 |
| 飞控接口 | MAVSDK / MAVLink over UART | 向 PX4 Offboard 模式发送设定值 |
| 地面端 | 工作站 / 笔记本 | 训练、调试、配置、遥测与结果分析 |

```text
Camera Module 3 ──CSI──> Raspberry Pi 5
                         ├─ MediaPipe Pose
                         ├─ State estimator
                         ├─ PPO Actor (inference)
                         └─ Supervisor + altitude loop
                                  │ UART / MAVLink
                                  ▼
                            Pixhawk 6C / PX4
                                  │
                              ESCs / motors

Workstation: simulation + training + analysis
Ground link: configuration / telemetry; outside the visual-control loop
```

## 研究范围

- 场景中只有一名可见目标，不做多目标身份匹配。
- 初版学习前后速度与转向角速度，高度由独立闭环保持。零垂向速度设定值不能直接替代经验证的高度闭环。
- 从双肩、双髋提取躯干中心与肩宽尺度，以变化率、关键点质量、观测年龄和上一步指令作为候选特征。
- 表观尺度是距离代理；人体转身和遮挡会影响它。MediaPipe 的髋部相对 world landmarks 不能直接测量相机到人的距离。
- `min_tracking_confidence` 是配置阈值，不能假设结果返回独立的 tracking confidence 分数。关键点的 visibility / presence 可用于质量判定。
- `LIVE_STREAM` 在忙碌时会忽略新输入帧，仍需显式控制缓冲与观测年龄。
- 训练在工作站进行；实机部署仅执行推理。Pi 5 的兼容性、帧率、时延与温升尚需基准测试。

## 研究路线与验收

这是建议的十周节奏，所有阶段均待实施。

1. **第 1–2 周：桌面感知。** 验证相机、ARM64 软件环境、延迟分布、关键点有效性、温度及电源稳定性。
2. **第 3–4 周：仿真与基线。** 在 PX4 SITL / Gazebo 中建立相同观测与动作约束，完成 PID 基线。
3. **第 5–6 周：PPO。** 随机化视觉噪声、延迟、目标运动与观测丢失，使用未参与训练的种子进行评估。
4. **第 7–8 周：机载集成。** 部署推理，拆桨测试串口链路、限幅、超时处理、人工接管及 PX4 故障保护。
5. **第 9–10 周：受控试验。** 从低速、固定高度开始，对比重复实验并记录日志、配置及局限。

计划比较 PPO 与调优后的 PID：画面中心 / 尺度误差、采集到指令的时延（中位数与 p95）、指令平滑性、目标保持率和恢复时间。尚无性能或 PPO 优势结论。

## 控制监督与集成要点

计划加入动作限幅、观测时效检查、目标丢失状态、独立高度控制以及遥控接管。悬停要求有效的位置估计；实际 Offboard 丢失动作必须按场地配置并验证。MAVSDK 维持的设定值心跳不能替代感知失效监测。

供电方案以稳定的 5 V / 5 A 电源轨为设计起点，结合合适的 Pi 5 USB-C 供电方式及主动散热。机架安装、重心、相机标定、串口电气兼容性、实机定位和负载下电压波动均需单独验收。

## 网站本地预览

使用 Node.js 18 或更高版本，无需安装第三方依赖：

```sh
npm start
```

访问 `http://127.0.0.1:4317/posepilot/`。运行 `npm run check` 可检查交互脚本语法。

网站文件位于 `docs/`，通过 GitHub Pages 从 `main` 分支的 `/docs` 目录发布。修改并推送这些文件会触发 Pages 更新。

网站提供中英切换、响应式布局、键盘可用的演示控件、播放 / 暂停和目标遮挡演示。语言偏好保存在浏览器本地；没有分析服务、表单或外部脚本。

## 官方资料

- [Holybro X500 V2](https://docs.holybro.com/drone-development-kit/px4-development-kit-x500v2)
- [MediaPipe Pose Landmarker for Python](https://developers.google.com/edge/mediapipe/solutions/vision/pose_landmarker/python)
- [Raspberry Pi 5](https://www.raspberrypi.com/products/raspberry-pi-5/)
- [Camera Module 3](https://www.raspberrypi.com/products/camera-module-3/)
- [Pi camera cable](https://www.raspberrypi.com/products/camera-cable/)
- [MAVSDK Offboard](https://mavsdk.mavlink.io/main/en/cpp/guide/offboard.html)
- [PX4 Offboard 与故障保护](https://docs.px4.io/main/en/flight_modes/offboard)

## English overview

PosePilot is a proposed open research platform for onboard single-person drone tracking. The latest architecture uses **Holybro X500 V2 + Pixhawk 6C + Raspberry Pi 5 + Camera Module 3**. MediaPipe Pose supplies landmarks; a filter extracts a compact state; a small PPO actor proposes forward velocity and yaw rate. PX4 handles low-level flight stabilization, with a separate altitude loop planned for the first version.

Training takes place on a workstation. Wired CSI and UART / MAVLink links keep perception and control inference onboard. Research will compare PPO with a tuned PID baseline under matched limits and observations, including delays, landmark noise and temporary target loss.

**Status: design and project website only.** No flight software, trained policy, measured performance or experimental results are included. The website visualization uses synthetic landmarks and illustrates state extraction rather than a real policy.

## License

The original website source and project documentation in this repository are licensed under [MIT](LICENSE). Referenced hardware, software, models and trademarks retain their respective owners' licenses and rights.
