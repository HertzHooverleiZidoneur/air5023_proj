# 05 · Benchmark 评测组

负责成功判据、批量试验、失败分类和跨组回归。05.1～05.3 由本组负责人统一完成。总分工见 [协作手册](../docs/development/README.md)，交付进度和后续实测计划见 [PLAN.md](PLAN.md)。

本机已有完整环境与资产：Ubuntu-22.04 `/root/air5023_proj`。Windows checkout 与 WSL 副本独立，复用方式见 [LOCAL_ENVIRONMENT.md](LOCAL_ENVIRONMENT.md)。双任务、C 独立拒绝赛道、信息权限审查和操作碰撞报告已经接入，**新版清单、指标语义及使用命令以 [V2_GUIDE.md](V2_GUIDE.md) 为准**。下面保留原红杯任务的基础用法；最新实测证据见 [VALIDATION.md](VALIDATION.md)。

## 现有文件和运行

| 文件 | 当前能力 |
|---|---|
| [evaluation/success_checker.py](evaluation/success_checker.py) | 双指接触、抬升、无环境支撑、持续保持 |
| [tests/](tests/) | 接口、动作拒绝、IK、评估与数据处理回归 |
| [run_benchmark.py](run_benchmark.py) | 场景 × 策略 × policy seed × 重复次数，隔离进程、超时、版本和报告 |
| [run_episode.py](run_episode.py) | 在 05 内继承公共接口，逐控制周期采样，消费已有任务阶段回调 |
| [evaluation/criteria.py](evaluation/criteria.py) | 阈值校验、连续保持判据，独立于仿真依赖 |
| [evaluation/telemetry.py](evaluation/telemetry.py) | 导航距离/耗时/误差/碰撞与失败分类 |
| [evaluation/reporting.py](evaluation/reporting.py) | 按任务/赛道/权限/split 分组、条件等权均值、置信区间、CSV/Markdown、回归对比 |
| [evaluation/placement.py](evaluation/placement.py) | G1 抬升保持、松爪前支撑、最终稳定放置的独立时间戳判据 |
| [evaluation/rejection.py](evaluation/rejection.py) | C 条件明确拒绝、碰撞与真实停稳检查 |
| [evaluation/tasks.py](evaluation/tasks.py) | 任务兼容性、信息权限声明与 API 调用审查 |
| [import_suite.py](import_suite.py) | 将已生成厨房/G1/16 挑战条件转换为 v2 清单 |

在项目根目录运行：

```bash
.venv/bin/python -m pytest -q
.venv/bin/python 05_benchmark/run_benchmark.py \
  --episodes 3 --output out/team05/baseline_001
```

默认关闭图像以提高物理评测速度；需要图片时加 `--with-images` 并确保图形后端可用（Linux 默认 EGL、Windows 默认 GLFW）。每个 episode 保存日志、原始阶段结果和独立评测指标。异常、超时、缺失/损坏指标都计入失败分母，任意失败使 runner 返回 1；输入配置错误返回 2；用户中断返回 130。默认单个子进程墙钟超时 300 秒，可通过 `--timeout` 修改。

未指定清单时仍重复原固定场景，报告明确 `scene_variation: false`，不能作为跨场景泛化评估。多 seed 只设置 Python/NumPy RNG 和子进程 Python hash seed，不生成新场景；其他框架的随机性由策略适配器负责。固定 oracle 策略本身通常不会因 policy seed 改变。

## 原红杯任务指标与口径

评测器读取 `configs/task.yaml`，运行前记录判据；运行期间配置或场景/metadata 文件变化会显式失败。下表限于移动抓取任务；G1 放置与拒绝任务的判据见 V2 指南。记录本次判据不代表统一仿真执行预算已经冻结。

| 指标 | 判据或定义 |
|---|---|
| `grasp_success` | 双指接触目标、抬升 ≥ 0.05 m、无环境支撑接触，连续保持 ≥ 1 s；仿真状态必须有限 |
| `navigation_success` | 完成 `navigate` 阶段，最终 XY 误差 ≤ 0.05 m、朝向误差 ≤ 5° |
| `navigation_collision_events` | 导航阶段机器人与非地面几何对从无接触变为有接触的次数；持续接触只计一次，分离后重碰再计 |
| `task_success` / `success` | 无执行异常、导航成功、导航碰撞次数不超过配置（默认 0），且结束时抓取成功 |
| 导航距离 | 每个控制周期真实底盘 XY 位移的累加，单位 m |
| 导航耗时 | `navigate` 阶段实际采样的仿真时长，单位 s；墙钟耗时另列 |

抓取成功不是锁存值：成功后掉落仍判失败。相邻抓取观测间隔超过 1.5 个控制周期、时间倒退或失去有效接触会重新计时，避免跨未观测区间累积“连续保持”。初始杯高在 `initialize` 完成、进入 `navigate` 时采样，与现有任务的稳定高度基准对齐。

接触使用实际 MuJoCo 接触数据。导航碰撞排除已知地面以及位于地面高度的水平 plane；导航阶段接触目标物也算碰撞。操作阶段的目标接触由抓取判据判断，不套用“导航零接触”。碰撞只在控制周期末采样，可能遗漏周期内的短暂接触；不是 500 Hz 全物理步碰撞统计。

报告分别给出完整任务、导航和抓取成功率，全部包含运行失败的分母。数值指标的均值/中位数/最大值带 `observed_episodes`，缺失数据不会当成 0。成功率附 Wilson 95% 区间；确定性重复不满足独立泛化试验假设，区间仅用于描述。

## 03 → 05 场景清单

旧格式 `benchmark-suite-1.0` 继续兼容，参考 [examples/suite.fixed.json](examples/suite.fixed.json)。新任务与挑战条件使用 v2，见 [V2_GUIDE.md](V2_GUIDE.md)。路径相对**清单文件目录**解析，也接受绝对路径。

```json
{
  "schema_version": "benchmark-suite-1.0",
  "conditions": [
    {
      "id": "cup-left-start-near",
      "scene": "scenes/condition_001/kitchen.xml",
      "metadata": "scenes/condition_001/metadata.json",
      "seed": 7,
      "description": "由 03 描述实际变化和可复现配置"
    }
  ]
}
```

`id` 必须唯一；metadata 至少包含 `seed`、`target_body` 和非空的 `navigation_waypoints`（世界坐标 `[x,y,yaw]`）。清单 seed 必须与 metadata 一致。场景必须在目标机器上可加载，XML 引用的资产、include 和 metadata 一起交付；05 不生成场景、不改写物体状态。XML hash 不覆盖外部 mesh/include 内容，03 仍需保留生成配置、资产版本及校验清单。

先检查再运行：

```bash
.venv/bin/python 05_benchmark/run_benchmark.py --manifest out/team03/suite.json --output out/team05/batch_001 --preflight
.venv/bin/python 05_benchmark/run_benchmark.py --manifest out/team03/suite.json --seeds 7 19 31 --episodes 1 --output out/team05/batch_001
```

`--preflight` 校验清单、阈值和可比较性，打印矩阵规模，不创建输出目录；不保证 MuJoCo 能加载场景。20 个场景条件 × 3 个 policy seed × 1 个策略 × 1 次重复 = 60 次运行。20 次固定场景重复不等于 20 个场景条件。`scene_variation` 仅表示存在不同 XML hash，不自动证明语义上的跨场景泛化。

## 02 → 05 策略接入

原红杯策略 `scripted_oracle_baseline` 直接调用现有 `tasks.pick_red_cup.run`；默认根据任务匹配兼容内置策略。其他策略通过 `--strategy-file` 登记权限，旧 `package.module:run` 入口仍兼容但标为权限未声明。适配器保持既有任务签名：

```python
def run(robot, metadata, output, instruction, *, on_phase):
    ...
```

适配器须使用传入的 robot，在稳定初始化后调用 `on_phase("navigate", robot.data.time)`，导航完成后调用下一阶段回调，再执行抓取、抬升和保持。所有物理推进经过 `robot.step()` / 其高层调用；不得直接调用 `mujoco.mj_step` 绕过采样。回调时间必须为当前仿真时钟。适配器负责创建自己的 output 子目录，可返回 `{"success": ...}` 作为诊断；正式成功标签由 05 的独立采样确定。

当前支持完整任务适配器，不自动把 `Policy.predict()` 拼成导航抓取状态机。VLA 对接由 02 的适配器完成，RGB 策略使用 `--with-images`。metadata/目标真值只用于 oracle 基线与评估，模型观测仍通过公共 `get_observation()`，不得把 metadata 或评测轨迹注入模型输入。

## 输出、失败与版本对比

根目录：`plan.json`（预声明矩阵/阈值/版本）、`summary.json`、`episodes.csv`、`report.md`。每个 `episode_XXXX/` 保存 `request.json`、`command.json`、`run.log`、`progress.json`、`runtime.json`、`telemetry.jsonl`、`metrics.json`；原任务产物在 `stages/`。失败时可能仅有部分文件，并有 `error.json`。每个完成的 episode 后保存一次汇总；中断后保留已尝试部分及 planned/completed 数，不宣称整个矩阵完成。

`telemetry.jsonl` 含逐控制周期时间、阶段、底盘真值和接触/抓取状态，只用于评测诊断。版本记录包括 Git commit/dirty 状态、相关源码与依赖锁 hash、Python、实际包版本、策略入口源码 hash、场景和 metadata hash。依赖与源码必须由协作组一并保留，hash 本身不是可执行环境快照。

失败类型包括 `navigation_timeout`、`navigation_failed`、`navigation_collision`、`missing_finger_contact`、`object_supported`、`dropped_after_grasp`、`insufficient_lift`、`insufficient_hold`、`non_finite_state`、`execution_error`、`setup_error`、`wall_clock_timeout`、`launch_error`、`missing_metrics`、`invalid_metrics`、`nonzero_exit`、`interrupted`。异常记录类型、消息和最后阶段；未知异常不通过模糊日志匹配强行分类。

```bash
.venv/bin/python 05_benchmark/run_benchmark.py --manifest out/team03/suite.json --seeds 7 19 31 --output out/team05/batch_002 --compare out/team05/batch_001/summary.json
```

对比要求相同报告版本、任务/赛道/split/权限，以及场景/metadata hash、条件 ID、策略名、policy seed、重复次数、阈值、指令、图像开关和超时预算。报告列出以前成功、现在失败的配对案例与成功率变化。更改实验条件时须新建实验，不直接做配对回归。

## 测试与真实验收

独立测试使用 Python、NumPy、PyYAML、pytest，无需厨房资产或 MuJoCo：

```bash
python -m pytest -q 05_benchmark/tests/test_benchmark_criteria.py 05_benchmark/tests/test_benchmark_runner.py 05_benchmark/tests/test_benchmark_worker.py 05_benchmark/tests/test_benchmark_v2.py
```

覆盖抓高但未抓住、单指、环境支撑、保持不足、掉落、非有限状态、采样断档、导航碰撞去重、非法清单、损坏指标、真实子进程超时、失败计入分母、配对回归，以及双任务和拒绝赛道。2026-11-28 已复用本机环境完成全项目 85 项测试与 11 次真实仿真：红杯 6/8、G1 1/1、C 安全拒绝 2/2。另有圆桌 A 探针结果与可达标注不一致，详见 [VALIDATION.md](VALIDATION.md)。这些是开发联调结果，尚非冻结协议下的正式成绩。

## 本组任务与验收

- **05.1 指标**：扩充导航耗时/距离、碰撞、失败阶段；维护 `configs/task.yaml` 的统一阈值。
- **05.2 批量实验**：消费 03 的场景清单，提供策略选择、多 seed 和版本追踪；正式矩阵规模、split 与物理执行预算待确认。
- **05.3 回归**：每组交付正常与失败样例；持续保留“升高但未抓住”“仍有支撑”“夹持后掉落”等负例。

所有策略使用同一判据，不能用不同阈值改善单个方法结果。可以读取仿真真值做评估，但不能将其混入模型 Observation。与 07 协作，让视频上的成功标签来自实际指标。
