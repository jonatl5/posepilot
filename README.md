# objectPilot — RoboMaster TT

识别贴在墙上的物品打印图，控制无人机对准目标、缓慢靠近，并用前向红外 ToF 在近距离测量墙面距离。电脑运行 YOLO、PID / PPO 和独立的安全约束；TT 负责底层飞行稳定，扩展板 ESP32 提供前向测距。仓库 URL 保留历史名称 `posepilot`，当前实现不使用 MediaPipe 或人物跟踪。

[项目网站 / Rapid prototypes](https://jonatl5.github.io/posepilot/#prototypes) · [输入输出定义](research/interfaces.md) · [中文数据采集方案](research/data_collection_zh.md) · [固件说明](firmware/README.md) · [验证记录](research/validation.md)

**当前阶段：无人机到货前的软件原型。** 现在可以在电脑上运行仿真 demo、训练和评估 PPO、对视频/摄像头做检测并记录控制输出、生成 ESP32 扩展固件源码。尚未完成真实打印图检测指标、固件编译/刷写或 TT 飞行验证；Python 端没有自动起飞或发送飞行 RC 的入口。

## 1. 安装：从全新 clone 开始

推荐 **Python 3.12 + Git**，CPU 即可。首次安装需要网络；后面的纯仿真 demo 不下载模型，也不需要摄像头、无人机或视频。

Windows PowerShell，在准备存放项目的目录运行：

```powershell
git clone https://github.com/jonatl5/posepilot.git objectpilot
cd objectpilot
py -3.12 -m venv .venv
$opPython = (Resolve-Path .\.venv\Scripts\python.exe).Path
& $opPython -m pip install -e ".[vision,test]"
& $opPython -m pytest -q
```

本文 PowerShell 命令均在**仓库根目录、同一个终端**执行。`$opPython` 直接使用虚拟环境，不要求修改 PowerShell 执行策略；重新开终端后再次设置它。只需要仿真/训练时，可以先安装 `-e .`，随后按需补装 `.[vision,test]`。

macOS / Linux：

```bash
git clone https://github.com/jonatl5/posepilot.git objectpilot
cd objectpilot
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install -e ".[vision,test]"
python -m pytest -q
```

后文用 `python` 替换 `& $opPython`；打开 HTML 时使用浏览器或文件管理器。`requirements-lock.txt` 是已验证 Windows / Python 3.12 环境的精确版本快照，其他平台优先使用上面的 extras 安装。需要复现该 Windows 环境时先 `pip install -r requirements-lock.txt`，再 `pip install -e . --no-deps`。

## 2. 无硬件 demo：先看 PID 自动靠近

```powershell
& $opPython -m objectpilot.demo --seed 10000 --out runs/demo_pid
Invoke-Item runs/demo_pid/demo.html
```

命令实际运行 Python 仿真，再生成一个**可离线打开、可播放/暂停/拖动进度**的 HTML。默认种子下，PID 应以 `success` 结束；页面显示轨迹、墙面间距、限幅后的 RC 和整段实验结果。浏览器播放的是本次产生的记录，页面不会实时执行 PPO。

| 输出 | 用途 |
|---|---|
| `runs/demo_pid/demo.html` | 演示用回放，所有数据内嵌，不需要 Web 服务 |
| `summary.json` | 控制器、种子、配置、终止原因、最小间距 |
| `trajectory.jsonl` | 每个时刻的 64 维观测、提议动作、约束后动作、RC 和仿真位置 |

这些观测是**仿真生成的检测框与 ToF**，不会验证 YOLO 识别效果。仿真位置只用于动力学、奖励、评估和显示，不直接输入策略。

## 3. PPO：训练 → 保存 → 同条件评估 → 回放

先用短训练检查整条链路。短训练只验证程序能运行，不应期待策略收敛：

```powershell
& $opPython -m objectpilot.train --steps 8192 --seed 0 --out runs/ppo_smoke
& $opPython -m objectpilot.evaluate --episodes 20 --seed 20000 --out runs/pid_dev
& $opPython -m objectpilot.evaluate --model runs/ppo_smoke/policy.zip --episodes 20 --seed 20000 --out runs/ppo_dev
& $opPython -m objectpilot.demo --model runs/ppo_smoke/policy.zip --seed 10000 --out runs/demo_ppo
Invoke-Item runs/demo_ppo/demo.html
```

训练输出 `policy.zip` 和 `metadata.json`；两者必须保存在同一目录。评估输出每局 `episodes.csv` 和汇总 `summary.json`，包括成功率、95% Wilson 区间、间距违规次数和终点误差。PID 和 PPO 要使用相同的评估种子、配置及 supervisor。失败或超时也是结果，demo 会直接显示。

当前训练使用 CPU、4 个环境、每环境 256 步 rollout、64×64 MLP。SB3 会完成整个 rollout 批次，所以 `--steps 100000` 实际产生 100,352 步；以 `metadata.json` 的 `actual_steps` 为准。网络结构、超参数在 [train.py](src/objectpilot/train.py)，动力学和奖励在 [env.py](src/objectpilot/env.py)。

进行较长的候选策略训练时，至少使用三个独立训练种子，例如：

```powershell
foreach ($seed in 0, 1, 2) {
    & $opPython -m objectpilot.train --steps 1000000 --seed $seed --out "runs/ppo_seed$seed"
    & $opPython -m objectpilot.evaluate --model "runs/ppo_seed$seed/policy.zip" --episodes 100 --seed 20000 --out "runs/ppo_seed${seed}_dev"
}
```

这是一个起始实验预算，不是收敛保证。先用 development / validation 场景调整 PID、奖励和 PPO；冻结后才运行未使用的最终评估种子。已展示的 `10000–10029` 和上面的 `20000…` 都属于开发数据。训练和评估都可加 `--nominal` 做降低随机化的对照实验，比较时须保持一致。

既有初测：PID 为 30/30；PPO 8,192 步为 0/30；PPO 100,352 步为 14/30。详细限制和 CSV 见 [原始测评证据](docs/evidence/README.md)。这些是单训练种子的早期仿真结果，不能代表真实飞行，也不是完成调优后的最终比较。权重和大型训练产物不放进 Git；可自行训练，或使用 [10 月 29 日实验包](https://drive.google.com/file/d/1CWB-8wcXvh0cUm5BgMetaEBFtSmdq6WW/view) 中的旧检查点及配套 metadata（可能需要团队权限）。

## 4. 真实视频 / 摄像头 demo：检测后只记录命令

用手机拍摄贴墙的物品打印图，把视频放入 `data/raw/`。例如选择 `bottle`，先用预训练 YOLO11n 跑通：

```powershell
& $opPython -m objectpilot.replay --source data/raw/approach.mp4 --class-name bottle --out runs/video_pid.jsonl --show
# 或使用笔记本摄像头；需要本机摄像头权限：
& $opPython -m objectpilot.replay --source 0 --class-name bottle --out runs/webcam_pid.jsonl --show
```

首轮 YOLO 使用可能联网下载 `yolo11n.pt`。`--class-name` 必须存在于该权重的类别表；替换为自训练模型时使用 `--weights path/to/best.pt`。窗口按 `q` / `Esc` 退出；无桌面环境去掉 `--show`。默认最多处理 1,000 个控制采样，可用 `--max-frames` 调整。

日志记录目标框、观测、提议动作、最终动作、RC、阻止原因和推理耗时。**此入口永不连接无人机或发送 RC。** 目标连续确认后才进入控制；没有有效前向测距时，进入近距离区域会得到 `near_range_unavailable` 并停止继续靠近，这是预期行为。

`--range-jsonl` 可为文件视频附加时间对齐的测距记录，使用与视频一致的秒时间轴；它不支持摄像头模式。不要用机身向下 ToF 替代前向距离。`--visual-scale` 默认为未标定的 0.22，只适合跑通链路；真正测距前应按打印图和相机标定。PPO 检查点要求完整 Config 与运行时一致，不能直接把改过视觉标定参数的视频输入旧策略。详见 [接口约定](research/interfaces.md)。

## 5. 无人机到货前如何收集与训练检测数据

先选 2–3 类尺寸较大、易打印的物品，用哑光 A3 纸贴墙；固定相机高度，在地面标出 0.4–3 m 的距离。录制靠近、转向、停顿、遮挡、画面裁切和空墙负样本，并更换光照和打印图。手机视频用于检测训练和观测噪声分析；**PPO 必须在动作会影响下一状态的环境里训练**。

```powershell
Copy-Item data/clips.example.csv data/clips.csv
# 编辑 clips.csv：替换成真实视频路径、group_id、print_id 和 split。
& $opPython -m objectpilot.data extract --manifest data/clips.csv --out data/frames/batch01 --fps 2
# 人工标注 YOLO 格式 class_id cx cy width height（坐标归一化）。
# 在 frames.csv 填写 label_path，逐帧核验后将 label_reviewed 改成 yes。
& $opPython -m objectpilot.data prepare --frames data/frames/batch01/frames.csv --out data/dataset/v1 --classes bottle,backpack,cup
& $opPython -m objectpilot.data train --data data/dataset/v1/dataset.yaml --epochs 50 --device cpu
```

同一录制 session / 打印图组必须使用同一 `group_id` 并只属于一个 split，不能把同一视频的相邻帧分进训练和测试。`--classes` 的顺序定义标签 ID（这里 bottle=0、backpack=1、cup=2）。空墙负样本也要有一个明确审核过的空 `.txt` 标签文件。准备工具拒绝未审核标签、跨 split 的同组数据和重复图像内容，并要求 train / val / test 都非空。只有少量视频时应人工指定分组，避免哈希分组产生空 split。

检测训练产物在 `runs/detector/printed_targets/` 下，验证集用于选模型；冻结模型和阈值后才查看 test。另用独立 calibration 样本建立 `distance_m,box_height_fraction,split` CSV（`split` 均为 `calibration`），运行 `python -m objectpilot.data calibrate --csv 路径` 估计视觉比例。无人机到货后补采 TT 视频，并重新测量相机、ToF、延迟和控制响应。完整流程见 [采集方案](research/data_collection_zh.md)。

## 6. ESP32 firmware：现在能做什么

仓库同步的是**自定义测距扩展源码、可选外部传感器适配器和基于固定官方版本的生成器**，不是已经编译、刷写或飞行验证的固件二进制。

```powershell
git clone https://github.com/RoboMaster/RMTT_Libs.git artifacts/references/RMTT_Libs
git -C artifacts/references/RMTT_Libs checkout b8ece60ec31eeca3350209a2559f549864e91dce
& $opPython firmware/prepare_firmware.py --upstream artifacts/references/RMTT_Libs --out artifacts/firmware/objectpilot_default
```

这一步只需要 Python 标准库和 Git。输出包括 `objectpilot_default.ino` 与 `range_snapshot.h`；生成器检查补丁位置，拒绝覆盖已有输出，也不会修改上游源码、安装板卡包或刷写硬件。生成后的 sketch 保留官方串口桥接和原有控制功能。

| 文件 | 作用 |
|---|---|
| [prepare_firmware.py](firmware/prepare_firmware.py) | 给已检查的官方 default sketch 加入测距采样和查询回调 |
| [range_snapshot.h](firmware/range_snapshot.h) | 原子读取采样序号、毫米值、有效标志和样本年龄 |
| [external_vl53l1x.h](firmware/external_vl53l1x.h) | 可选 Pololu VL53L1X 适配器；默认不启用，接线和 I2C 冲突待验证 |

自定义查询拟使用 `EXT oprange?`，返回 `oprange <sequence> <sampled_ms> <mm> <valid> <age_ms>`。它不是 DJI 官方 SDK 命令，实际路由需要到货后验证。现有 TT 扩展套件已带前向 ToF，应先验证内置传感器，再决定是否添加外置传感器。官方示例可能保留旧距离，因此“收到新回复”不等于“新采样”。

到货后再安装 DJI 对应的 TT 板卡工具链、编译、准备恢复方式并做拆桨台架检查。硬件读取入口单独安装：

```powershell
& $opPython -m pip install -e ".[hardware]"
# 仅在 TT 到货、拆桨并连接其 Wi-Fi 后执行：
& $opPython -m objectpilot.bench --seconds 15 --out runs/tof_bench.jsonl
# 自定义固件与路由确认后，才可附加 --custom。
```

`bench` 只做 SDK/测距读取；无需在无人机到货前运行。板卡、量程、时序、光轴、供电与 0x29 地址冲突的验证细节见 [firmware/README.md](firmware/README.md)。

## 7. Pipeline 与训练环境

```text
视频/摄像头 → YOLO → 目标连续确认 ─┐
前向 ToF → 来源/有效性/年龄检查 ──┤
                                ↓
                   16 个特征 × 4 个时刻 = 64 维
                                ↓
                         PID 或 PPO [forward, yaw]
                                ↓
                     独立 supervisor → dry RC 日志

Gymnasium 动力学 → 合成检测/ToF → 同一状态构建器与 supervisor
```

- **观测：** 目标位置/尺度、视觉距离、ToF、有效性与关联标志、置信度、数据年龄、变化率、上一动作和近距离测距锁存标志。初始历史补零，使用固定缩放。
- **动作：** 两个归一化连续值；supervisor 禁止后退、先对准再前进、限制动作变化，并对丢目标、过期传感器或过近距离停止。映射为 `rc 0 round(20*forward) 0 round(25*yaw)`；这些是 SDK 通道值，不是 m/s。
- **环境：** 固定高度、二维墙面目标、默认 10 Hz；随机初始位置、方向、动作/视觉延迟、增益、噪声和丢帧。它是待标定的简化模型。
- **奖励：** 距离误差、中心偏差、动作变化和时间惩罚；成功奖励，以及过近/边界/丢失惩罚。
- **结束：** 目标附近稳定保持 2 秒成功；触及最小间距、越界或长期丢失为终止；40 秒超时为 truncation。默认目标间距 0.65 m、最小间距 0.40 m，都须经过真机标定。

## 8. 文件、验证和常见问题

```text
src/objectpilot/   核心状态/约束、仿真、demo、训练、评估、视觉、数据与台架入口
firmware/          ESP32 扩展源码与生成器
tests/             状态、传感器、Gymnasium、数据泄漏与离线 demo 测试
research/          接口、采集计划、资料来源、验证记录
data/              采集 manifest 模板；真实视频/标签在本地
docs/              GitHub Pages 与已公开的原始测评 CSV/JSON
.github/workflows/ 无硬件 CI：测试、PPO save/load、demo、固件源码生成
```

- `FileExistsError`：输出受保护。选择新的 `--out`，保留已有实验；不是训练失败。
- `Policy schema/config mismatch`：保留匹配的 `policy.zip` + `metadata.json`，并使用对应 Config；修改特征或标定后应重新训练/评估。
- `Unknown class`：类别必须与加载的 YOLO 权重一致。
- 没有运动输出：先看 JSONL 的 `supervisor`；未确认目标、失去目标和近距离无有效 ToF 都会输出零。
- `--show` 没有窗口：在有桌面的本机运行；服务器/CI 去掉该选项。摄像头 demo 尚未在用户的真实摄像头上验证。
- `.venv/`、`runs/`、模型权重、采集视频、标签、生成固件与临时产物不提交 Git。已公开的仿真 CSV/JSON 保存在 `docs/evidence/`。

完整验证记录见 [research/validation.md](research/validation.md)，后续任务及课程日期见 [TASKS.md](TASKS.md)。网站在 `docs/`；安装 Node.js 后可用 `npm start` 预览。

## License

本项目原创代码采用 MIT。DJI 上游固件、Ultralytics 软件/模型、其他依赖和数据分别遵守其自身许可；仓库的 MIT 不重新授权这些第三方内容。生成的 DJI sketch 保留其原始许可头。
