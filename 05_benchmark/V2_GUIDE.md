# 已确认设计的使用说明

本轮实现双任务、独立负例赛道、信息权限登记/审查与分组、操作碰撞诊断，以及 G1 桌面任意位置稳定放置。开发/测试划分保持 `unspecified`；统一仿真时长/步数预算尚未冻结。墙钟超时默认仍为 300 s，原控制器自身导航超时继续生效。因此当前验证不能宣称已经完成同物理预算的方法比较。

## 两种任务与独立负例

| task / track | 成功条件 | 不适用的指标 |
|---|---|---|
| `mobile_pick_hold` / `feasible` | 原导航与碰撞要求，最终双指无支撑抬升 ≥5 cm、连续 ≥1 s；无执行异常 | 放置成功 |
| `fixed_dock_pick_place` / `feasible` | 按固定 dock 初始化，双指无支撑抬升 ≥5 cm、连续 ≥1 s；松爪前有桌面支撑；末尾连续 ≥1.5 s 只由桌面支撑、无夹爪接触、倾角 <6°、速度 <0.025 m/s；无执行异常 | 导航成功（写 null，不伪装为导航成功） |
| `mobile_pick_hold` / `safe_rejection` | 明确返回指定停靠区不可达；正常退出；全程无机器人与非地面环境/目标接触；末尾连续 ≥0.3 s 线速度 <0.015 m/s、角速度 <0.04 rad/s | 正常抓取任务成功率、导航/抓取成功 |

G1 的初始高度在其固定 1 s 初始化结束时记录；测量来自 worker 独立采样，不采用控制器 `passed`。时长由相邻有效时间戳累计，不以“样本数÷频率”多算第一个区间；相邻间隔超过 1.5 个控制周期重置连续计时。稳定放置不依赖控制器给某阶段起名为 `settle`，最终掉落或倾倒会失败。

松爪前支撑沿用上游的最近 4 个控制周期窗口，接触力阈值为 0.01 N。XY 偏移只作诊断，没有新增放置区域要求。G1 初始化使用固定 dock 是该任务的公开条件，不属于自主导航。

C 条件只描述指定停靠区，不代表所有底盘位置或抓取姿态均不可行。安全拒绝成功写 `safe_rejection_success=true`、`task_success=false`；统一 `success` 表示该赛道检查通过。负例永远不进入正常任务成功率分母。超时、异常、无理由返回均不算拒绝成功。

## 导入现有场景

`import_suite.py` 消费上游已生成的文件，不修改 03 的生成器。路径相对新 suite 目录保存。以本机 WSL 环境为例，在当前开发仓库根目录运行：

```bash
/root/air5023_proj/.venv/bin/python 05_benchmark/import_suite.py \
  --kitchen-scene /root/air5023_proj/scenes/kitchen/generated/kitchen.xml \
  --kitchen-metadata /root/air5023_proj/scenes/kitchen/generated/metadata.json \
  --g1-scene 05_benchmark/artifacts/confirmed_v2_scenes/g1.xml \
  --challenges 05_benchmark/artifacts/confirmed_v2_scenes/tabletop_challenges.json \
  --output 05_benchmark/artifacts/my_suite

/root/air5023_proj/.venv/bin/python 05_benchmark/run_benchmark.py \
  --manifest 05_benchmark/artifacts/my_suite/suite.json \
  --output 05_benchmark/artifacts/my_run --preflight
# 检查矩阵后，去掉 --preflight 运行。
```

`--g1-scene` 和 `--challenges` 可省略。上例两者都有时共保留 18 个条件：原厨房、G1、16 个挑战。导入器校验挑战 XML 哈希、上游静置检查结果、目标物体与 C 标签。

- 8 个可运行红杯条件：原厨房 + 岛台 A/B/D1/D2/E/F1/F2。
- 1 个 G1 放置条件。
- 2 个 C 条件，单独进入安全拒绝测试。
- 7 个正常圆罐条件：保留在清单，标记 `pending_controller`。不会复用红杯的抓取偏移和指令。

B/E 的候选绕行航点保留在 `source_condition` 供审查，当前原红杯基线只收到指定最终停靠航点，不自动改变其导航算法。未解决绕障可能产生真实失败，应原样报告。生成器的静置通过不保证抓取可解。

旧 `benchmark-suite-1.0` 继续可用，自动解释为移动抓取、正常任务、未划分 split。新 `benchmark-suite-2.0` 增加 `task`、`track`、`split`、`layout`、`object`、`variation_axis`、`availability`、`pending_reason`、`instruction`、`resources` 和 `source_condition` 等字段。G1 必须附带 dock 和 replay 资源，运行前后核对哈希。C 必须显式标注 `expected_nominal_dock_reachable=false`。

## 策略与权限

未指定 `--strategies` 时按兼容性匹配以下内置策略，未匹配项列入 `excluded_pairs`，不冒充已尝试失败：

| 策略 | 支持范围 | 权限 |
|---|---|---|
| `scripted_oracle_baseline` | 红杯正常任务 | 目标/底盘真值、场景航点 |
| `g1_tcp_retarget` | G1 固定 dock 放置 | 目标/底盘/TCP 真值、接触真值、固定 dock、公开 replay |
| `oracle_dock_probe` | C 的停靠区拒绝检查 | 场景碰撞几何、底盘真值、指定 dock；不读 C 标签 |

`oracle_dock_probe` 复用 01 的场景栅格和保守规划器，只检查名义停靠区并停稳，不是完整抓取策略，也不证明全局不可达。规划器使用当前姿态的圆形外接足迹及 5 cm 净空。其拒绝可能来自保守几何，不能直接推断所有其他规划器也无法到达。

2026-11-28 实测对照：岛台 A 返回可达，但圆桌 A 也被探针拒绝，与上游设计期望的可达标注不一致。几何诊断确认圆桌终点距桌面投影包围盒 0.39 m，小于保守足迹加净空的 0.51579 m，终点检查不通过；不能据此证明实际三维机器人一定碰撞。C 的 2/2 仅表示两条拒绝输出通过独立安全检查，不能解释为可达性分类准确率；对照数据单独保存在 `artifacts/confirmed_v2_probe_controls/`。该问题需进一步核验，当前没有降低净空或改写标签来让结果通过。

G1 适配器须通过 `robot.set_gripper(open=True/False)` 明确发出开合命令；兼容 `position` 时以归一化开度 >0.5 记为开、≤0.5 记为闭。不得绕过该接口直接写 actuator，否则独立评测不能认定已观察到松爪动作。

自定义策略使用 `--strategy-file declarations.json --strategies my_method`。声明格式如下，其中入口需要替换成实际实现：

```json
{
  "schema_version": "benchmark-strategies-1.0",
  "strategies": {
    "my_method": {
      "entrypoint": "your_package.adapter:run",
      "tasks": ["mobile_pick_hold"],
      "tracks": ["feasible", "safe_rejection"],
      "targets": ["red_cup_main"],
      "information_class": "oracle",
      "base_pose": "simulator_truth",
      "target_pose": "simulator_truth",
      "scene_information": ["navigation_waypoints"],
      "observations": ["joint_state", "rgb"],
      "audit_status": "declared_only",
      "audit_notes": "Describe actual inputs and what has been reviewed."
    }
  }
}
```

任务签名仍为 `run(robot, metadata, output, instruction, *, on_phase)`。安全拒绝正常返回 `{"outcome":"rejected","reason":"nominal_dock_unreachable"}`；只声明这一返回值不够，还须通过独立碰撞/停稳检查。正常任务的最终成功由 worker 判断。

权限类别支持 `oracle`、`estimated_state`、`sensor_only`、`undeclared`；审查状态区分 `declared_only` 和 `source_reviewed`。运行时记录公开状态/目标真值/RGB API 调用次数，对声明与观察到的真值 API 调用冲突标记 `review_required`。这种审查不能观察所有 `robot.model/data` 直接访问，不是强制信息隔离，也不能据“未发现调用”证明没有泄漏。旧的裸 `module:function` 入口保持可运行，但标为 `undeclared`，只默认支持移动抓取正常任务。

## 碰撞、统计和输出

导航仍按原规则判定零碰撞；操作阶段单独记录接触事件、接触样本数和几何对。操作诊断排除地面和目标物的正常抓取接触，不排除桌子/其他障碍物。连续接触计一次，分离后再次接触再计；操作碰撞目前不改变成功标签。所有接触仍按控制周期采样，不能声称覆盖 500 Hz 物理子步。

报告版本为 `benchmark-report-2.0`，单回合为 `benchmark-episode-2.0`。主表按 **策略 × 任务 × 赛道 × split × 信息权限 × 审查状态** 分组；含每组条件数、尝试数、成功率、条件等权均值和适用的子任务指标。Wilson 区间保存在 JSON 内，仅作描述。顶层 `success_rate` 仅是正常任务的混合概览，不包含 C；只有负例时为 null。缺失数据不记成零，不适用阶段不记成失败。

每次试验保留 request、runtime、独立 telemetry、metrics、控制器原始输出和异常。声明、任务资源、放置/拒绝判据写入 plan。版本配对要求相同任务/赛道/split/权限/条件/阈值/矩阵；不直接跨 v1/v2 做结果回归。

真实验收结果见 [VALIDATION.md](VALIDATION.md)。正式开发/测试划分、统一仿真预算、圆罐完整控制器、真实 VLA 比较仍需后续完成。
