# 07 · Extension 公共基础与扩展组

负责跨组共用的仿真/机器人接口、安装配置、GUI、视频和新功能集成。负责人：待认领。总分工见 [协作手册](../docs/development/README.md)。

## 现有文件和运行

| 文件 | 当前能力 |
|---|---|
| [robot/robot_interface.py](robot/robot_interface.py) | 官方 MuJoCo 适配、控制/相机时钟、状态、底盘/关节/夹爪 |
| [install.sh](install.sh) | 本地独立环境、锁定依赖、官方仓库部署 |
| [run_sim.py](run_sim.py) | 厨房 GUI 或有界无窗口观测检查 |
| [record_demo.py](record_demo.py) | 连续仿真、多视角、阶段与指标字幕、MP4 |
| [../project_bootstrap.py](../project_bootstrap.py) | 七组目录的 Python 包导入初始化 |

在项目根目录运行：

```bash
bash 07_extension/install.sh
.venv/bin/python 07_extension/run_sim.py --seconds 60
# 额外录制依赖
UV_CACHE_DIR=.cache/uv .tools/uv pip install --python .venv/bin/python -r requirements-video.txt
.venv/bin/python 07_extension/record_demo.py \
  --output out/team07/demo_001/demo.mp4 --run-dir out/team07/demo_001/run
```

视频需要 EGL 和 Noto Sans CJK 字体，详见 [媒体说明](../media/README.md)。每次使用新输出目录。核心安装脚本同步依赖锁后，如需录制，重新安装可选视频依赖。

## 本组任务

- **07.1 基础维护**：管理 RobotInterface、配置和依赖。优先复用官方控制/传感器；公共签名、坐标系或频率变化先与上下游确认。
- **07.2 展示交付**：增加视角、批量结果可视化、失败片段导出和易用运行参数；结果标签来自 05。
- **07.3 扩展集成**：接入新的传感器、任务或机器人配置，保持各组入口能共同运行。

## 边界和验收

01 实现导航算法，02 实现抓取/VLA/编排，03 实现场景和采集，04 实现真机后端，05 定义评测，06 实现感知和数据处理。07 不复制各组算法，而是维护其共同依赖。

每次公共接口变化后运行 `python -m pytest -q` 和完整基线；涉及相机时执行有 RGB 的 smoke test；涉及视频时检查编码、时长与关键阶段画面。新组脚本从自身位置解析项目根目录，调用 `activate()` 后导入其他组的包，避免依赖调用者的工作目录。
