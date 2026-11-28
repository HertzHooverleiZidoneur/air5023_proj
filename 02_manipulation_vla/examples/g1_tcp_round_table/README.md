# Stretch 3 圆桌抓放复跑

Stretch 3 从固定 dock 起步，在 MuJoCo 中抓起水瓶并放回圆桌。源动作来自 G1 真机抓瓶示教的 Isaac 回放，提取右手捏合中心近似位置（TCP）及开合事件。Stretch 用自身 IK 计算关节指令。放置时夹爪沿 Z 下探，确认有桌面承托后松开，停顿 0.55 秒再撤手。水瓶为自由刚体，夹持和放置依靠接触物理。

验证范围为固定圆桌、固定水瓶初始位置和保存的停靠状态。轨迹以仿真中的瓶子位置完成初始对齐。导航到停靠点、VLA 和 Stretch 真机部署是其他验证任务。200 集 Isaac G1 训练数据由原数据项目管理。

## 运行

从项目根目录运行。首次使用先按[项目安装说明](../../../README.md#快速运行)生成默认厨房。

```bash
.venv/bin/python 03_data_generate/build_round_table.py
.venv/bin/python 02_manipulation_vla/run_tcp_retarget.py --output out/round_table_demo
```

第一条命令用默认厨房（layout 1、style 0、seed 7）生成 `scenes/round_table/generated/kitchen_round_table.xml`。变体以直径 0.90 m、高 0.76 m 的圆桌替换岛台组，并加载水瓶模型。第二条命令运行抓放，将 `metrics.json` 和 20 Hz 的 `trajectory.jsonl` 写到本地 `out/round_table_demo/`。结果目录每次应选新名字。加 `--gui` 可用 MuJoCo 实时查看，加 `--render` 可保存逐帧总览图。

直接跟踪对照使用 `--no-support-projection --no-release-adapt`。使用 `--no-release-adapt` 保留落桌高度修正并跳过松爪停顿和专门撤手。退出码 1 表示物理判据未通过或操作中止。

## 交接文件

| 文件 | 用途 |
|---|---|
| [源输入 g1_tcp_source.npz](g1_tcp_source.npz) | 从单条 G1 示教的 Isaac 回放提取的时间、TCP 位置、XY 旋转和开合事件 |
| [source_manifest.json](source_manifest.json) | 源示教 ID、回放版本、校验值、采样率、标定窗口和字段定义 |
| [场景变体及初始状态](../../../scenes/round_table) | `objects.xml` 和 `assets/` 记录从 G1 圆桌和水瓶模型导出的几何、材质和碰撞。`dock.json` 按名称保存 Stretch 初始关节状态，底盘四元数顺序为 `wxyz` |
| [Stretch 仿真录像](../../../media/stretch_round_table.mp4) | 从停靠点开始的完整抓放 |

源轨迹为 433 帧、24 Hz，时间从 0 到 18 s。闭合和打开分别发生在 2.875 s、13.25 s。执行时对位置做 9 帧二阶 Savitzky–Golay 平滑，以闭合帧为原点，将 XY 位移旋转约 −117.57°，保持位移比例 1，并插值到 20 Hz 控制周期。腕部的姿态仅在离线提取 TCP 时用于计算捏合中心。执行时的 IK 约束 Stretch 的 TCP 位置。

源文件 `replay.usda` 和 `trajectory.jsonl` 来自 `g1_bottle_1103` 的 `left_hand_clear_fixed` 对齐回放。运行示例使用仓库内的 NPZ 和资产。如果已有源回放及其引用的 USD 资源，可在装有 `pxr` 的 Python 环境重新提取：

```bash
mkdir -p out/g1_extract
python 06_data_process/extract_g1_tcp.py \
  /path/to/left_hand_clear_fixed/replay.usda \
  /path/to/left_hand_clear_fixed/trajectory.jsonl \
  out/g1_extract/g1_tcp_source.npz
```

`TcpReplay.load(...)` 校验并加载轨迹。`run_retarget(robot, replay, ...)` 在调用方提供的 MuJoCo 实例中执行，期望已停在示例 dock。命令行入口调用 `restore_dock(...)` 设置该初始状态。未来导航联调可在同一个仿真实例停稳后直接调用 `run_retarget`。

## 验收

物理步进为 500 Hz，接触验收每 20 Hz 控制周期采样一次。通过条件包括：

- 两侧橡胶夹爪垫均有受力接触，瓶底比初始稳定位置抬高超过 5 cm 且离开支撑，持续至少 1 秒。
- 松爪前有桌面承托，撤手后 1.5 秒仅由桌面承托，倾角小于 6°，平动速度小于 0.025 m/s。

接触使用 0.01 N 的法向力阈值。`metrics.json` 记录抬升、稳定放置、TCP 跟踪误差和水平落点偏移。落点偏移作为测量项，当前通过条件允许放置在桌面上的不同位置。

`trajectory.jsonl` 包含仿真时间、阶段、世界坐标中的 TCP 目标与实测位置、瓶底位置和接触状态。`target_q` 和 `actual_q` 的顺序是 `lift, arm, wrist_yaw, wrist_pitch, wrist_roll`。前两项单位为米，后三项单位为弧度。这里的 `gripper_command` 表示开度（1 张开、0 闭合），源 NPZ 的 `close` 表示闭合请求。

重新生成圆桌场景后的本地测试结果：

| 模式 | 释放与放置结果 |
|---|---|
| 直接跟踪 | 抬升 8.41 cm。松爪前瓶底高于初始稳定位置 4.47 cm，桌面无承托，最终侧倒 |
| 仅修正落桌高度 | 松爪前有桌面承托，随后瓶子倾倒 |
| 下探、承托确认、停顿和撤手 | 抬升 8.41 cm，双垫夹持离桌且高度超过阈值 2.20 s，最终由桌面承托并直立 1.50 s |

完整模式的 TCP 跟踪 RMSE 为 1.56 cm，最终落点距初始位置 6.69 cm。20 Hz 验收采样中无超过阈值的 Stretch 与圆桌接触。项目现有 29 项回归测试通过。重新提取的四个样例数组与仓库文件一致。
