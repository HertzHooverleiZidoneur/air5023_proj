# 03 · Data Generate 数据生成组

负责厨房场景、资产、物体随机化、示范采集与原始数据记录。负责人：待认领。总分工见 [协作手册](../docs/development/README.md)。

## 现有文件和运行

| 文件 | 当前能力 |
|---|---|
| [build_scene.py](build_scene.py) | 复用官方厨房/Stretch 集成，处理生成文件兼容性 |
| [download_assets.py](download_assets.py) | 固定版本、SHA-256 校验，保留上游受控文件 |
| [collect_episode.py](collect_episode.py) | 完整脚本任务，逐控制周期记录，可选 10 Hz RGB |

在项目根目录运行；每次使用新的输出目录：

```bash
# 重建场景会更新 scenes/kitchen/generated 下的生成物
.venv/bin/python 03_data_generate/build_scene.py
# 采集状态/控制轨迹
.venv/bin/python 03_data_generate/collect_episode.py --output out/team03/episode_001
# 同时采集两路 RGB，需要 EGL；图像占用额外磁盘空间
.venv/bin/python 03_data_generate/collect_episode.py \
  --with-images --output out/team03/episode_002
```

## 数据交接格式

- `episode.json`：`episode-1.0`，指令、状态/执行器顺序、频率、seed、场景 hash、策略来源和结果。
- `trajectory.npz`：每条包含 `time/state` 和 `next_time/next_state`，以及 `actuator_ctrl`、完整仿真 `qpos/qvel`、`image_index/image_time`。
- `images/000000_head.png`、`000000_wrist.png`：可选图像；索引对应本控制周期结束后的观测。没有图像时索引为 -1。
- `stages/`：阶段指标、状态快照及可选视角图片。

`state` 使用底盘、关节及原始夹爪读数，顺序由 manifest 声明。`actuator_ctrl` 是 MuJoCo 底层执行器控制量，不能直接当作 VLA `Action`；`qpos/qvel` 含场景和目标的特权真值，只作诊断。采集失败且缺少 `episode.json` 的目录属于不完整产物，不能视为合格 episode。

## 本组任务与验收

- **03.1 场景随机化**：配置物体位置、实例、颜色和数量，验证不穿透、可抓取、同 seed 可重建。当前 seed 不会自动改变固定杯子 XY。
- **03.2 示范采集**：扩展不同控制来源和完整动作语义，与 06 确认时间对齐；当前采集器来自单个 oracle 脚本。
- **03.3 数据组织**：收集成功/失败示范，建立版本清单；避免覆写已有 episode。

03 → 06 交付原始 episode，03 → 05 交付固定场景清单。至少先完成 5 个可加载且可复现的场景配置。数据经 `06_data_process/process_episode.py` 校验后交接。资产与随机化改动放在本组及 `configs/scene.yaml`，保留官方代码。
