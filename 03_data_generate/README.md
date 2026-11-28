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

## 实拍圆桌参考场景

[round_table.xml](../scenes/round_table/round_table.xml) 是一份独立的 MuJoCo 场景，包含圆桌、红盖瓶、开口圆罐和浅色包装。以视频约 25 秒的桌面画面为主，利用标记卡校正透视，再结合其他角度和参考图估计物体的形状。桌面直径取 1.0 m、高度取 0.75 m。几何尺寸、质量和摩擦均为近似值。

在项目根目录用已安装 MuJoCo 的项目环境打开：

```bash
.venv/bin/python -m mujoco.viewer --mjcf scenes/round_table/round_table.xml
```

可以在查看器中选择 `overview` 或 `tabletop` 相机。三个桌面物体都是带碰撞的自由刚体。场景无需 RoboCasa 资产，当前用于检查布局和接触，尚未接入机器人或抓取任务。

### 把圆桌放入厨房

生成原厨房的 `kitchen.xml` 后，在项目根目录运行：

```bash
.venv/bin/python 03_data_generate/add_round_table.py --xy 0.55 -2.6 && .venv/bin/python -m mujoco.viewer --mjcf scenes/kitchen/generated/kitchen_round_table.xml
```

`--xy` 是圆桌中心在厨房世界坐标系中的位置，单位为米。这组坐标在岛台左侧。合并脚本写出独立的 `kitchen_round_table.xml`，原抓取入口继续使用 `kitchen.xml`。换位置后需重新检查桌体和周围家具的间隙。

### 生成多组圆桌条件

[配置清单](../configs/round_table_conditions.json) 定义五组固定偏移，单位为米，坐标方向沿厨房世界坐标轴。五组包括参考摆放、圆罐沿 X 轴向两侧各移动 4.5 cm、向桌内移动 4.5 cm，以及瓶子和包装靠近圆罐。

```bash
.venv/bin/python 03_data_generate/add_round_table.py --xy 0.55 -2.6 --conditions configs/round_table_conditions.json
```

生成器检查初始物体重叠，并运行 5 秒仿真确认物体直立、保持桌面接触。输出位于 `scenes/kitchen/generated/`，其中 `round_table_conditions.json` 记录条件 ID、场景文件、世界坐标和 SHA-256。使用相同厨房和配置可重建同一组条件。

### 生成圆桌和岛台的 16 组干扰条件

圆桌沿用实拍参考场景，目标为圆罐。另一套沿用 RoboCasa `one_wall_large` 厨房中的原岛台，目标为红杯。每套包含八组，原来五组圆桌摆位配置仍可单独生成。

| 条件 | 设计 |
|---|---|
| A | 原场景和原目标位置 |
| B | 原底盘路线放一个垃圾桶，保留绕行路线 |
| C | 用围挡封闭任务的原停靠区，预期规划返回不可达 |
| D1 / D2 | 分别用纸盒和圆柱容器挡住原机械臂接近路线 |
| E | 组合 B 和 D1 |
| F1 / F2 | 分别增加两个和三个桌面干扰物。F2 含与目标外观相近的圆柱体 |

```bash
.venv/bin/python 03_data_generate/build_challenge_scenes.py
```

生成器读取 [干扰物配置](../configs/tabletop_challenges.json)，在已生成的 `kitchen.xml` 同级目录写入 16 份 `challenge_*.xml` 和 `tabletop_challenges.json`。先检查新增地面障碍的几何间隙、桌面物体初始重叠与 5 秒静置，整批通过后写出。清单包含目标物体、原停靠位姿、场景哈希，以及 B/E 的候选绕行航点。在 macOS 上若环境设有 Linux 专用的 `MUJOCO_GL=egl`，运行时设为 `MUJOCO_GL=cgl`。

C 的负例针对清单指定的停靠区。B 和 E 提供候选绕行航点。

例如，在 MuJoCo 查看器中打开圆桌封路条件：

```bash
.venv/bin/python -m mujoco.viewer --mjcf scenes/kitchen/generated/challenge_round_c.xml
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
