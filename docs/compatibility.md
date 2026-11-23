# 部署与兼容性记录

核查和部署日期为 2026-11-23。工程位于独立的 `stretch_vla_home/`，未修改原 AIR5023 项目。

## 固定版本

| 组件 | 本项目版本或提交 |
|---|---|
| Python | 3.10.19，项目内 uv 管理的独立解释器 |
| Stretch MuJoCo | 0.5.0 / `c78d6a1cd79bb5ead1b679610299563c1a91124a` |
| 配套 RoboCasa | 0.2.0 / `336e8d98c69bcc06238ff59728c5919308923622` |
| 配套 robosuite | 1.5.1 / `a9f03224dd0cb9b4c1f1a51082f13add0ab09dc1` |
| MuJoCo | 3.2.6 |
| NumPy / Numba | 1.23.3 / 0.56.4 |
| Mink / SciPy | 0.0.5 / 1.10.1 |
| OpenCV / Matplotlib | 4.8.1.78 / 3.10.1 |
| Torch | 2.5.1+cpu，旧版 RoboCasa 的传递依赖 |
| ROS2 / Nav2 | 未安装，不是当前 waypoint Demo 的依赖 |

完整直接依赖在 `requirements.in`，全部解析后的依赖固定在 `requirements.lock`。RoboCasa 和 robosuite 使用 Stretch 官方 `.gitmodules` 指向并锁定的 fork。上游官方入口见 [Stretch 仓库](https://github.com/hello-robot/stretch_mujoco) 和 [RoboCasa 仓库](https://github.com/robocasa/robocasa)。

## 实际遇到的问题与处理

### 新旧 RoboCasa 不可直接混装

当前 RoboCasa365 主线要求的 MuJoCo 为 3.3.1，Stretch 当前选定提交固定 3.2.6。采用 Stretch 官方锁定的旧版 RoboCasa，避免把两套主线直接拼接。未来模型推理服务建议独立环境部署。

### Stretch 的 extra 与子模块依赖冲突

Stretch `[robocasa]` 可选依赖中固定 NumPy 1.23.5、Numba 0.59；其 RoboCasa 子模块固定 NumPy 1.23.3、Numba 0.56.4。解析器确认二者直接冲突。

安装 Stretch 基础包，再显式安装其两个子模块及兼容版本。没有用 `--no-deps` 绕开检查。最终 `uv pip check` 通过。

### 系统 Python 缺少开发头文件

系统 Python 3.10.12 在构建 `evdev` 时缺少 Python.h。改用项目本地的 Python 3.10.19 发行包，构建成功。机器原先无 git，安装脚本提供 Ubuntu 22.04 的项目内解包方案；其他发行版可以预装 git 和 C 编译器后运行脚本。

### Box 旧资产链接失效与备份版本差异

配套 RoboCasa 下载脚本中的旧 Box 链接返回 404。改用 [RoboCasa 官方 Hugging Face 备份](https://huggingface.co/datasets/robocasa/robocasa-assets/tree/1b92c3d02ca4354984fec961357db0bff7b32166)，将 revision、文件大小和 SHA-256 固定在 `configs/assets.lock.json`。

仅下载 fixtures、textures、objaverse，不下载当前任务不用的 AI 物体、生成纹理及模型权重。下载器支持断点续传和压缩包路径检查。解压只补充缺失文件，不覆盖仓库已有 XML。

该备份不等同于原 v0.2 资产包，已确认两类差异。

1. 杯子的尺寸元数据从 `bottom_site/top_site/horizontal_radius_site` 改成了 `reg_bbox` 几何体。`adapt_cup_asset` 在生成目录中复制官方 `cup_2` XML，根据 bbox 恢复旧版尺寸 site，移除非物理语义区域，并将网格、纹理路径绝对化。视觉和分解碰撞网格均来自官方资产。
2. fixtures 包中缺少旧版电器的部分网格和纹理。任务子类的 `edit_model_xml` 在生成时保留全部可用官方碰撞几何；缺失的纯视觉几何使用透明占位保留 robosuite 的名称索引，并显示对应碰撞几何作为简化外观。纯视觉、无碰撞的静态部件设置明确的小惯性，避免 MJCF 二次导出中的零质量错误。缺失碰撞网格会直接报错。

最终场景记录了 68 个缺失视觉资产路径，详见 `scenes/kitchen/generated/asset_compatibility.json`。柜体、岛台、墙地面、椅子、机器人和杯子正常加载；部分电器外观简化，因此这不是原版厨房的完整视觉复现。该适配只用于当前 Demo，不用于声称复现 RoboCasa 的官方基准成绩。

### 官方生成器与模型导出

使用 `stretch_mujoco.robocasa_gen.model_generation_wizard` 进行 RoboCasa → Stretch 集成。它内部先生成 PandaMobile 场景，再用官方清理与 include 机制替换机器人。生成器会产生未追踪的 `stretch_temp_abs.xml`，工程随后将模型保存为展开的 `kitchen.xml`，避免后续生成覆盖 include 文件影响已导出的场景。

官方生成器文档中的四元数顺序注释有误，本工程遵循 MuJoCo 的 `wxyz`。

### 控制、抓取与运行性能

- 默认 500 Hz 物理、20 Hz 动作、10 Hz 相机。频率通过仿真时间调度，头部和腕部图像分别带有采样时间。
- 官方服务器默认启动异步传感器线程。同步包装器在启动阶段停止该线程，使用官方同步相机管理器和单份 MjData，避免评估与状态读取跨进程失配。
- 当前不用 lidar，关闭 MuJoCo sensor 计算，保留 RGB 渲染与机器人运动状态。可通过 `compute_mujoco_sensors` 恢复计算。
- 官方夹爪闭合时指尖会回缩。脚本对 `link_grasp_center` 的接近目标作 2.8 cm 补偿，该参数只验证了当前杯子与夹持方向。
- 没有修改机器人碰撞网格、夹爪接触摩擦或杯子动力学来制造成功。

### GUI 验证

官方 `launch_sim` 成功创建可见 MuJoCo 窗口，已保存 `out/official_window.png`。第一次桌面全屏抓图因合成器返回黑图，随后使用 X11 指定窗口捕获确认了渲染。官方示例收到 SIGINT 后偶尔出现其自身的连接关闭异常；本工程上下文管理器负责关闭相机与 viewer。

## 第三方变更范围

第三方仓库的受版本控制文件保持不变。全部兼容转换位于项目自身的生成脚本和生成目录。部署早期资产包覆盖过两份配套附件 XML，已恢复原始版本，并修正下载器避免再次覆盖。

迁移项目根目录后重新执行 `03_data_generate/build_scene.py`。若要升级上游提交，应重新解析依赖、核查资产 schema 并重跑物理和 GUI 测试。

## 七组开发目录

2026-11-24 将本项目实现迁入 `01_navigation`～`07_extension`。官方仓库、资产和场景生成物路径保持原位置；本次只移动本项目代码，不需要修改上游源码或升级依赖。

编号目录通过根目录 `project_bootstrap.activate()` 暴露内部合法 Python 包名。已有 CLI 自动初始化路径；Notebook 先执行 `activate()`。机械臂 IK 的导入由 `robot.arm_controller` 改为 `manipulation.arm_controller`，测试统一执行 `.venv/bin/python -m pytest -q`。迁移和团队归属见 [七组分工](development/README.md)。
