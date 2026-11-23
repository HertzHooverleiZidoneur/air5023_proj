# 06 · Data Process 感知与数据处理组

负责相机/观测数据契约、视觉定位、示范数据校验清洗与模型数据格式转换。负责人：待认领。总分工见 [协作手册](../docs/development/README.md)。

## 现有文件和运行

| 文件 | 当前能力 |
|---|---|
| [perception/observation.py](perception/observation.py) | RGB、机器人状态、指令及时间戳结构 |
| [perception/red_cup.py](perception/red_cup.py) | HSV 红色区域诊断，尚非杯子语义识别 |
| [perception/camera.py](perception/camera.py) | 场景总览相机与图片保存 |
| [process_episode.py](process_episode.py) | 校验 03 的连续轨迹、图像引用/时序，输出统计和非特权状态 |

在项目根目录运行；先由 03 生成 episode：

```bash
.venv/bin/python 06_data_process/process_episode.py \
  --input out/team03/episode_001 --output out/team06/episode_001
```

输入为 `episode.json + trajectory.npz + 可选 images/`。输出 `transitions.npz` 和 `report.json`；图像仍引用源 episode，不复制，因此源目录应保留。处理器校验有限值、状态/控制形状、控制时间连续性、相邻状态一致性、图像引用和采样延迟；错误数据会失败，不静默跳过。

输出剔除包含场景真值的 `qpos/qvel`。保留的 `actuator_ctrl` 仍是底层执行器控制，报告明确 `vla_training_dataset: false`。此步骤没有自动完成模型动作适配、训练/验证划分或归一化。

## 本组任务

- **06.1 感知**：通过 07 暴露官方深度与相机参数，明确坐标变换；实现类别/颜色目标选择、三维位置与置信度。当前检测可能误检红色墙面。
- **06.2 数据质量**：异常/丢帧/重复检测、时间对齐、按 episode 划分训练验证集、统计和来源追踪。
- **06.3 格式转换**：与 02 明确目标模型数据契约，再实现对应格式导出、图像预处理和归一化参数生成。

## 交接和验收

03 提供原始数据；06 给 01/02 提供视觉目标，给 02 提供模型需要的数据；04 提供真实相机标定与时延，07 管理底层采样。

建议验证至少 5 个杯子位置、红色干扰物和目标不在视野的情形；三维误差目标 ≤3 cm，未检测到时返回 invalid。数据处理至少覆盖正常 episode、非法数值、时间断裂与缺失图像，并确保策略数据不包含目标真值字段。
