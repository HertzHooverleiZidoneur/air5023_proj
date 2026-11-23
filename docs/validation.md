# 实测记录

验证日期为 2026-11-23，主机为 Ubuntu 22.04，GPU 为 NVIDIA RTX 4090 Laptop。验证使用项目内 Python 3.10.19 环境和锁定依赖。

| 检查 | 结果 | 证据 |
|---|---|---|
| 固定依赖能否共同安装 | 通过，无依赖冲突 | `out/install_recheck.log` |
| 安装脚本重复执行 | `bash 07_extension/install.sh --skip-assets` 通过 | `out/install_recheck.log` |
| 官方 Stretch GUI | 可见窗口正常渲染 | `out/official_window.png` |
| RoboCasa 厨房与 Stretch 集成 | MJCF 成功编译，1318 个 geom | `out/build_scene.log` |
| 两路 RGB | 头部 240×424、腕部 270×480，RGB uint8 | `out/task/*_head.png`、`*_wrist.png` |
| 接口和负例测试 | 8 项通过 | `out/tests.log` |
| 无窗口导航抓取 | 成功 | `out/task_v2/metrics.json` |
| GUI 导航抓取 | 成功，退出码 0 | `out/task/metrics.json`、`out/task_gui.log`、`out/task_gui_window.png` |
| 连续仿真视频 | 成功，56.3 秒、720p、20 fps、1 倍速 | [MP4](../media/stretch_vla_demo.mp4)、[元数据](../media/stretch_vla_demo.json) |
| 视频完整解码与时间检查 | 1126 帧全部解码，帧间仿真增量 0.05 秒，SHA-256 一致 | `out/video_qa/report.json`、`out/video_run/frame_times.json` |

无窗口测得杯子抬升 **0.134812 m**，左右手指均接触目标，环境接触为 false，连续保持 **2.0 s**。GUI 测得结果约为 **0.1348 m**、保持 **2.0 s**。任务耗时约 **54.3 s 仿真时间**，其中导航约 35.5 s。物理 500 Hz，动作 20 Hz，相机 10 Hz。

最后一轮 GUI 验证包含初始化、阶段截图和结束后 8 秒停留，总墙钟耗时 67.84 秒；成功完成后自动捕获窗口并关闭本次验证进程。

测试覆盖了非有限底盘速度、超限关节/夹爪、错误 embodiment、未适配向量的拒绝，检查无效输入不产生动作；还覆盖了实际关节执行、不可达 IK 不改变状态，以及“物体升高但未被手指夹住”不能判成功。

固定场景中的预抓取、接近、夹紧、抬升和结束阶段都有状态记录。`*_state.npz` 保存实际 qpos、qvel、ctrl 和仿真时间，便于诊断；它们不是完整轨迹回放文件。结果总览在 `out/task/finished_overview.png`，夹爪视角在 `out/task/finished_wrist.png`。

当前结果属于固定厨房、固定杯子实例、仿真真值目标驱动的脚本基线。它验证了场景、控制、相机与抓取评估接口，尚不代表 VLA 推理能力或跨场景泛化成功率。脚本中的颜色检测只是一项视觉诊断，红色墙砖等背景也可能产生检测结果。

本次录制复现了相同抓取结果，并在任务完成后继续运行 2 秒物理展示结果。已检查各阶段抽帧，完整解码报告不等同于人工逐帧观看。新增阶段回调和录制器后，8 项现有测试通过；安装可选录制依赖后，`uv pip check` 检查 78 个包无冲突。开发任务的稳定编号、建议验收与交接边界见 [分工手册](development/README.md)，录制方法见 [媒体说明](../media/README.md)。

复现命令

```bash
.venv/bin/python -m pytest -q
MUJOCO_GL=egl .venv/bin/python 02_manipulation_vla/run_task.py --headless
.venv/bin/python 02_manipulation_vla/run_task.py --linger 10
```

## 2026-11-24 · 七组目录重构验证

实现已迁到 `01_navigation`～`07_extension`；上方历史记录中的运行路径同步更新为现布局。本次安装脚本执行 `bash -n` 语法检查，完整安装的历史证据仍为 2026-11-23 的日志。

| 检查 | 本次结果 | 证据 |
|---|---|---|
| 原接口测试与新增数据契约测试 | 12 项通过；上游 3 条弃用警告 | `out/team_layout/tests.log` |
| 独立导航入口 | 到达约 `[3.5250,-1.2999,-0.0028]`，正常退出 | `out/team_layout/navigation.log` |
| 完整任务与连续 RGB 采集 | 成功；1086 个控制周期、543 对 RGB | `out/team_layout/episode_rgb/episode.json` |
| 03 → 06 数据交接 | 1086 条记录通过时序、形状、图像引用检查，剔除 qpos/qvel | `out/team_layout/processed_rgb/report.json` |
| S2R 参数导出 | 仿真契约正常导出，标记未验证硬件 | `out/team_layout/sim_contract.json` |
| 批量评测入口 | 固定场景 1 次，成功 1 次，仅验证入口与重复性 | `out/team_layout/benchmark/summary.json` |
| 9 个 CLI 入口 | 从 `/tmp` 调用绝对路径的 `--help` 均正常 | `out/team_layout/entrypoint_checks.json` |
| 文档与源码 | README 文件链接存在，七组源码编译检查通过 | 本次检查 |

新数据测试覆盖特权字段剔除、非法数值、时间不连续和缺失图像拒绝。完整采集仍使用原有固定场景脚本策略，杯子抬升约 0.134812 m，保持 2 秒；VLA、真机后端及跨场景评测尚未实现。
