# 相关开源工作与复用位置

核查日期为 2026-11-23。以下区分已集成组件与可供后续参考的项目。

| 工作 | 已有能力 | 在本项目中的位置 |
|---|---|---|
| [Stretch MuJoCo](https://github.com/hello-robot/stretch_mujoco) | Stretch 3 模型、相机、底盘/关节/夹爪控制、RoboCasa 场景转换 | 已集成，作为机器人仿真核心 |
| [RoboCasa](https://github.com/robocasa/robocasa) | 家庭厨房布局、家具、物体和任务框架 | 已集成其 Stretch 配套旧版本，提供单厨房和杯子 |
| [robosuite](https://github.com/ARISE-Initiative/robosuite) | MuJoCo 任务组合、物体模型、接触检查 | 已通过配套子模块复用 |
| [Stretch AI](https://github.com/hello-robot/stretch_ai) | 面向 Stretch 的建图、导航、抓取、操作及语言代理 | 可参考后续感知和任务分解，当前未集成 |
| [OK-Robot](https://github.com/ok-robot/ok-robot) | 将语言、导航和抓取模块组合为家庭 pick-and-place 系统 | 可参考语义地图与模块边界，当前未集成 |
| [OpenPI](https://github.com/Physical-Intelligence/openpi) | π 系列策略及模型适配/推理代码 | 后续可实现 `Policy` 客户端，当前不下载 checkpoint |
| [Isaac GR00T](https://github.com/NVIDIA/Isaac-GR00T) | 通用机器人策略、embodiment 配置和适配流程 | 后续候选模型来源，当前不加载模型 |

Stretch AI 的官方 README 明确列出 Stretch 3 的抓取、建图、导航和语言代理能力。仓库已于 2026-11-13 归档，应作为代码参考并审查其接口与依赖，再决定是否复用。其 README 也披露了旧版网络反序列化问题，若后续采用通信代码应先修复。[官方 README](https://github.com/hello-robot/stretch_ai/blob/main/README.md)

OK-Robot 最接近“在家庭中理解目标、导航并抓取”的系统目标。作者报告在 10 个真实家庭、170 多件物体上进行了实验。它依赖真实 Stretch、环境扫描和 AnyGrasp 等组件，不能当作本工程可直接运行的 MuJoCo VLA checkpoint；后续可参考其分开的 navigation、manipulation 和 hardware 模块。[官方项目说明](https://github.com/ok-robot/ok-robot)

OpenPI 的 DROID 输入示例明确给出两路图像、7 维关节位置和 1 维夹爪状态，其输出适配器只保留前 8 维。Stretch 的动作则包括差速底盘、升降、伸缩和手腕，因此数组维度凑齐并不意味着动作语义兼容。本工程使用有名称、有单位的 `Action`，把模型专属映射留在 adapter。[官方 DROID adapter](https://github.com/Physical-Intelligence/openpi/blob/main/src/openpi/policies/droid_policy.py)

当前工程的价值在于提供能检查物理抓取结果的仿真底座。后续可以逐步把脚本中的仿真真值目标替换成视觉定位，把任务阶段选择和动作生成替换成 VLA，同时复用相同的相机、控制与成功检查接口。
