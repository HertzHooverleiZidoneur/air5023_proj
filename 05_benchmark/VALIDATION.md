# 05 组验证记录

## 最新：2026-11-28 已确认功能验收

已完成双任务、C 独立安全拒绝、权限声明/源码审查/API 记录与分组、操作碰撞诊断、G1 桌面任意位置稳定放置。实现细节和命令见 [V2_GUIDE.md](V2_GUIDE.md)。全量回归 **85 passed，3 个上游 DeprecationWarning，16.21 s**，原始 JUnit 为 `artifacts/v2_full_tests.xml`。

复用 Ubuntu-22.04 `/root/air5023_proj/.venv/bin/python` 和现有资产，运行 Windows 当前源码（HEAD `532cee1` 加本地改动），未覆盖 WSL `60bdc73` 的源码。Python 3.10.19 / MuJoCo 3.2.6 / NumPy 1.23.3。全量测试命令：

```bash
cd /mnt/c/Users/Administrator/Documents/ChatGPT/机器人数学基础/air5023_proj
PYTHONDONTWRITEBYTECODE=1 \
NUMBA_CACHE_DIR=/root/air5023_proj/.cache/numba \
MPLCONFIGDIR=/root/air5023_proj/.cache/matplotlib \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
/root/air5023_proj/.venv/bin/python -m pytest -q -p no:cacheprovider \
  --basetemp 05_benchmark/artifacts/v2_full_tests \
  --junitxml 05_benchmark/artifacts/v2_full_tests.xml
```

新增回归覆盖时间戳时长（20 Hz 的 30 个样本只能提供 1.45 s）、放置后的掉落、采样断档、支撑/松爪、非法状态、明确拒绝/碰撞/停稳/异常、操作碰撞计数、负例不混入正常分母、联合分组、16 条件导入与待适配项、权限声明和矛盾指标拒绝。构造轨迹只用于软件验证。

### 真实场景与 11 次矩阵

调用上游生成器，在 `artifacts/confirmed_v2_scenes/` 生成 G1（1053 geom）及 16 个挑战场景；16 个条件全部通过生成器的加载、初始几何和 5 s 静置检查。**这些检查不验证机器人导航可达性**。导入器生成 18 条件的 v2 清单；其中 7 个圆罐正常条件为 `pending_controller`，实际可运行 11 个配对。

每个配对 policy seed=7、repeat=0、无 RGB，按兼容性匹配三个内置 oracle 策略。每回合独立进程，默认墙钟上限 300 s；控制频率 20 Hz，物理步长 0.002 s。运行命令：

```bash
PYTHONDONTWRITEBYTECODE=1 \
NUMBA_CACHE_DIR=/root/air5023_proj/.cache/numba \
MPLCONFIGDIR=/root/air5023_proj/.cache/matplotlib \
/root/air5023_proj/.venv/bin/python 05_benchmark/run_benchmark.py \
  --manifest 05_benchmark/artifacts/confirmed_v2_suite/suite.json \
  --output 05_benchmark/artifacts/confirmed_v2_run
```

输出目录必须新建；复跑时更换名称。矩阵已完整执行，runner 返回 1 是因为有两个真实失败，非启动或工具故障。

| 条件 | 任务/赛道 | 结果 | 仿真时间 s | 操作碰撞事件 |
|---|---|---|---:|---:|
| fixed-kitchen | 红杯移动抓取 | 成功 | 54.3 | 0 |
| g1-fixed-dock | 固定停靠抓取放置 | 成功 | 26.9 | 0 |
| round_c | 安全拒绝 | 通过 | 3.5 | 0 |
| island_a | 红杯移动抓取 | 成功 | 54.3 | 0 |
| island_b | 红杯移动抓取 | navigation_timeout | 48.0 | 0 |
| island_c | 安全拒绝 | 通过 | 3.5 | 0 |
| island_d1 | 红杯移动抓取 | 成功 | 54.3 | 14 |
| island_d2 | 红杯移动抓取 | 成功 | 54.3 | 2 |
| island_e | 红杯移动抓取 | navigation_timeout | 48.0 | 0 |
| island_f1 | 红杯移动抓取 | 成功 | 54.3 | 0 |
| island_f2 | 红杯移动抓取 | 成功 | 54.3 | 0 |

- 红杯 **6/8（75%）**；G1 **1/1**，分任务报告。可执行任务混合概览为 **7/9**，仅作概览。
- C **2/2** 独立报告；不计入正常任务成功率。均明确返回 `nominal_dock_unreachable`，正常退出，全程采样机器人碰撞为 0，返回前连续停稳约 **3.25 s**，满足用户确认的 0.3 s / 0.015 m/s / 0.04 rad/s 规则。
- G1 独立指标：峰值抬升 **0.08414 m**，双指无支撑保持 **2.15 s**，最终稳定桌面支撑 **5.10 s**，XY 偏移 **0.06692 m**；导航成功为 null。稳定时长不依赖上游阶段名称，因此与上游仅在 settle 段报告的 1.5 s 不同。
- D1/D2 的操作碰撞按已确认规则仅作诊断。D1 包含夹爪几何与 `challenge_table_1` 接触；没有因非零碰撞事后改判或隐藏记录。
- B/E 保留原控制器的导航超时失败，不使用候选绕行点暗中改进其他组策略。
- 全部内置策略按 oracle/source_reviewed 分组；本次公开 API 审查无声明冲突，但直接 model/data 访问不受强制隔离。

产物：[汇总报告](artifacts/confirmed_v2_run/report.md)、[完整 JSON](artifacts/confirmed_v2_run/summary.json)、[逐回合 CSV](artifacts/confirmed_v2_run/episodes.csv)。每个 episode 中另有独立采样、指标、策略阶段和运行日志。文件位于 Git 忽略目录，交接时需单独保存。

### 额外可达场景对照及局限

在评分矩阵之外运行同一个 `oracle_dock_probe`：岛台 A 返回 `route_available`，圆桌 A 返回 `rejected`。因此“两个正例均应可达”的诊断断言失败，结果原样保存在 [controls.json](artifacts/confirmed_v2_probe_controls/controls.json)，没有混入软件回归通过数或负例成绩。

上游的 `expected_nominal_dock_reachable` 来自 `03_data_generate/build_challenge_scenes.py` 的 `case_id != "c"`，是设计期望，未经过该生成器的路径搜索验证。探针使用 01 的静态栅格、外接圆足迹和 5 cm 净空；与圆桌 A 标签的差异需继续核验。**C 的 2/2 仅证明已声明负例上的拒绝与安全检查通过，不证明探针有准确的可达性分类能力，更不是全局不可达证明。**

追加几何诊断已定位到终点净空检查：起点 `start_clear=true`，终点及终点栅格中心均不满足净空。当前机器人外接圆半径 **0.46579 m**，加净空后 **0.51579 m**；圆桌名义停靠点为 `(0.65, -3.49)`，桌面 `reference_tabletop` 的投影包围盒为 `x=[0.05, 1.05]、y=[-3.10, -2.10]`，最近距离 **0.39 m**。距离小于保守圆半径，所以规划器无需搜索中间路线就返回 None。该二维简化不能证明真实形状在指定朝向下一定碰撞；本轮没有改变净空或上游标签。完整记录见 [round_a_diagnosis.json](artifacts/confirmed_v2_probe_controls/round_a_diagnosis.json)。

当前 split 全为 `unspecified`、统一仿真时长/控制步数预算未冻结；7 个圆罐控制器、真实 VLA 对比、RGB 联调、完整物理故障注入仍待后续。当前数据属于开发联调，不是正式泛化成绩。

---

以下保留历史记录，涉及“尚未接入/待完成”的描述仅表示当时状态；当前状态以上文为准。

## 历史：2026-11-25 初版独立工具

验证对象：初版 05.1～05.3 的独立评测工具实现。

## 已验证

本机 Windows，Python 3.14，复用 NumPy 2.5.2、PyYAML 6.0.3；pytest 9.2.0 安装在本组忽略目录 `.test_deps/`。相邻项目未找到可直接复用的 pytest，默认源连接失败后使用清华镜像。没有改动项目依赖锁，也没有把该本机测试环境当作锁定仿真环境。

结果：**53 passed**，其中新增 05 测试 44 项、原有导航/数据管线测试 9 项。JUnit 原始结果位于 `05_benchmark/artifacts/tests.xml`，该目录不提交 Git。

```powershell
$env:PYTHONPATH = (Resolve-Path '05_benchmark/.test_deps').Path
$env:PYTEST_DISABLE_PLUGIN_AUTOLOAD = '1'
python -m pytest -q 05_benchmark/tests/test_benchmark_criteria.py 05_benchmark/tests/test_benchmark_runner.py 05_benchmark/tests/test_benchmark_worker.py 05_benchmark/tests/test_navigation_core.py 05_benchmark/tests/test_data_pipeline.py --basetemp 05_benchmark/artifacts/pytest --junitxml 05_benchmark/artifacts/tests.xml
```

`artifacts/` 父目录需预先存在。在正常项目环境里已安装 pytest 时，无需前两行，直接运行 pytest 即可。

| 范围 | 核验内容 |
|---|---|
| 判据 | 双指、升高、无支撑、连续保持、掉落重置、非有限状态、观测断档/时间倒退、非法阈值拒绝 |
| 导航指标 | 真实轨迹长度计算、耗时、角度绕回、接触事件去重、缺失导航与再次导航未完成 |
| 批量与结果 | 相对路径解析、seed 一致、重复 ID 拒绝、损坏 JSON/矛盾指标拒绝、失败计入分母、按 seed 分组 |
| 子进程 | 实际启动 Python 子进程验证非零退出、超时终止、启动失败；未启动机器人仿真 |
| worker 联调 | 使用机器人替身运行真实采集入口，验证独立指标不信任策略自报、掉落、碰撞、地面排除、导航超时、输入 hash 变化 |
| 可复现性 | preflight 不写文件、拒绝旧输出目录、同条件回归对比、条件不一致拒绝、中断保留已尝试数量 |
| 兼容性 | 原有无仿真依赖的导航核心、数据清洗契约测试通过；两个新增/更新 CLI 的 `--help` 正常 |

代码语法检查及 `git diff --check` 通过。本次改动仅位于 `05_benchmark/` 和 `configs/task.yaml`；已有 07 组机器人接口修改保留。

## 未验证与后续条件

- 此处原先写“锁定环境待组员提供”有误；2026-11-28 已找到并复用本机 WSL 环境，新版 SuccessChecker 已通过单次真实红杯运行。详见下方补充记录。
- 20 条件矩阵与真实 VLA 对比尚未运行。
- 测试中的成功/失败数据是专门构造的验证输入，不能作为机器人成功率或泛化成绩。
- 正式验收顺序与交接需求见 [PLAN.md](PLAN.md)，本机环境及资产位置见 [LOCAL_ENVIRONMENT.md](LOCAL_ENVIRONMENT.md)。

## 2026-11-28 本机真实环境复核

运行 Python 来自 Ubuntu-22.04 `/root/air5023_proj/.venv/bin/python`，加载代码来自 Windows 当前 checkout 的 WSL 挂载路径；没有重新安装环境、下载资产或覆盖 WSL 旧源码。环境实读：Python 3.10.19、MuJoCo 3.2.6、NumPy 1.23.3。

使用 `examples/suite.local-wsl.json` 引用已有厨房，当前 05 preflight 与真实单次红杯试验均通过。结果目录为 `artifacts/local_wsl_smoke_20261128/`：

- `task_success=true`，`navigation_success=true`，最终 `grasp_success=true`。
- 仿真时间 54.300 s；导航时间 35.500 s；导航路径 1.3295 m。
- 导航 XY 误差 0.02502 m、航向误差 0.002312 rad；采样导航碰撞事件 0。
- 最终抬升 0.13481 m，双指接触且无环境支撑；05 连续控制周期测得保持 3.350 s。
- `controller_reported_success=true` 与独立评测一致。任务原阶段日志只在保持阶段累计约 2 s，与逐控制周期测量的开始时间不同。

这是一次固定场景物理联调，不是多条件成功率或泛化成绩；未执行的新矩阵及负例不能由它替代。

新 pull 的 `build_round_table.py` 已在已有厨房上成功生成并由 MuJoCo 加载 G1 圆桌场景（1053 个 geom），输出 `artifacts/local_scene_20261128/kitchen_round_table.xml`。这验证新场景资产可用，不能直接等同于新版双任务 benchmark 完成。

随后执行上游 `02_manipulation_vla/run_tcp_retarget.py`，结果保存于 `artifacts/local_g1_smoke_20261128/`：上游 `passed=true`，最大抬升 0.08414 m，报告双指无支撑抬升 2.2 s、稳定桌面支撑 1.5 s，落点偏移 0.06692 m，机器人与桌面采样接触计数 0。该示例的保持/放置时长目前按样本数量统计；这只是上游示例复现结果，尚未通过新版独立双任务判据，不能把它直接汇入红杯的成功率。

本次同一 WSL 锁定环境执行项目全量回归：**73 passed，3 个上游 DeprecationWarning，15.18 s**。JUnit 位于 `artifacts/wsl_tests_20261128.xml`。命令：

```bash
PYTHONDONTWRITEBYTECODE=1 \
NUMBA_CACHE_DIR=/root/air5023_proj/.cache/numba \
MPLCONFIGDIR=/root/air5023_proj/.cache/matplotlib \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
/root/air5023_proj/.venv/bin/python -m pytest -q -p no:cacheprovider \
  --basetemp 05_benchmark/artifacts/wsl_pytest_20261128 \
  --junitxml 05_benchmark/artifacts/wsl_tests_20261128.xml
```

这一结果验证当前代码与已安装依赖的回归兼容性，不替代全部挑战条件物理实验。未运行三次重复、真实负例、RGB 新版 05 或正式多条件矩阵。
