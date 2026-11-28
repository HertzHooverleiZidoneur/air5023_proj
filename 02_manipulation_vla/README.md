# 02 · Manipulation & VLA 操作与模型组

负责机械臂、夹爪、抓取规划、VLA 推理与适配，以及整项任务的编排。负责人：待认领。总分工见 [协作手册](../docs/development/README.md)。

## 现有文件和运行

| 文件 | 当前能力 |
|---|---|
| [manipulation/arm_controller.py](manipulation/arm_controller.py) | 五个独立手臂坐标的有界 IK |
| [manipulation/tcp_retarget.py](manipulation/tcp_retarget.py) | G1 TCP 位置迁移、圆桌承托检查和撤手 |
| [tasks/pick_red_cup.py](tasks/pick_red_cup.py) | 初始化→导航→抓取→抬升→检查，支持阶段回调 |
| [vla/policy_interface.py](vla/policy_interface.py) | `reset/predict` 协议和动作块执行循环 |
| [vla/observation_adapter.py](vla/observation_adapter.py) | 明确状态顺序、图像、时间与夹爪归一化 |
| [vla/action_adapter.py](vla/action_adapter.py) | 标准 Action、范围校验与执行 |
| [run_task.py](run_task.py) | 完整固定任务 GUI / headless 入口 |
| [run_tcp_retarget.py](run_tcp_retarget.py) | 固定 dock 起步的圆桌水瓶抓放 |

在项目根目录运行：

```bash
.venv/bin/python 02_manipulation_vla/run_task.py
.venv/bin/python 02_manipulation_vla/run_task.py --headless --no-cameras
```

圆桌示例的运行命令、输入数据和验收结果见 [抓放交接说明](examples/g1_tcp_round_table/README.md)。`run_retarget(robot, replay, ...)` 接收已停稳的机器人实例。示例入口使用 `dock.json` 初始化该实例。

## 本组任务

- **02.1 操作**：抓取候选、接近路径碰撞检查、到位判定、夹持失败恢复。固定杯子补偿配置化；不通过瞬移或焊接生成成功。
- **02.2 VLA**：新增具体 checkpoint 客户端/加载器和模型专属 adapter。先 mock，再真实推理；明确 state/action 维度、单位、坐标系、绝对/增量语义与时间块。
- **02.3 编排**：构建有限状态机；调用 01 导航和 06 感知；按状态设置预算与有限重试，允许选择脚本、视觉或 VLA 路线。

组内可由一人做 manipulation，一人做 VLA，一人做任务联调；都在本组相应子包工作。

## 交接和验收

01 提供导航，06 提供观测/处理后的数据，07 提供仿真执行，04 后续提供真实执行后端，05 提供统一判据。

`Policy.predict(Observation)` 返回 `Sequence[Action]`，一个 Action 对应一个控制周期，当前动作块最多 1 秒。抓取建议覆盖三个可达位置、不可达与被阻挡情形；VLA 用人工单维动作验证映射，并检查非法输入整块拒绝。当前未加载 VLA，也没有模型服务，不能将脚本成功标为模型成功。
