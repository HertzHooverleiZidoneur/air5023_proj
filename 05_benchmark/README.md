# 05 · Benchmark 评测组

负责成功判据、批量试验、失败分类和跨组回归。负责人：待认领。总分工见 [协作手册](../docs/development/README.md)。

## 现有文件和运行

| 文件 | 当前能力 |
|---|---|
| [evaluation/success_checker.py](evaluation/success_checker.py) | 双指接触、抬升、无环境支撑、持续保持 |
| [tests/](tests/) | 接口、动作拒绝、IK、评估与数据处理回归 |
| [run_benchmark.py](run_benchmark.py) | 多次独立启动完整基线，记录超时/退出码和结果 |

在项目根目录运行：

```bash
.venv/bin/python -m pytest -q
.venv/bin/python 05_benchmark/run_benchmark.py \
  --episodes 3 --output out/team05/baseline_001
```

默认关闭图像以提高物理评测速度；需要图片时加 `--with-images` 并确保 EGL 可用。每个 episode 保存 `run.log`、阶段状态和指标，根目录保存 `summary.json`。异常或任务失败计入分母，任意失败使 runner 返回非零退出码。默认单个子进程墙钟超时 300 秒。

当前 runner 重复的是同一个固定场景，报告明确 `scene_variation: false`，不能作为跨场景泛化评估。

## 本组任务与验收

- **05.1 指标**：扩充导航耗时/距离、碰撞、失败阶段；维护 `configs/task.yaml` 的统一阈值。
- **05.2 批量实验**：消费 03 的场景清单，增加策略选择、多 seed 和版本追踪；建议先固定至少 20 个试验条件。
- **05.3 回归**：每组交付正常与失败样例；持续保留“升高但未抓住”“仍有支撑”“夹持后掉落”等负例。

所有策略使用同一判据，不能用不同阈值改善单个方法结果。可以读取仿真真值做评估，但不能将其混入模型 Observation。与 07 协作，让视频上的成功标签来自实际指标。
