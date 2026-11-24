# 01 · Navigation 导航组

负责底盘导航、避障和可操作接近位姿。负责人：待认领。总分工见 [协作手册](../docs/development/README.md)。

## 现有文件和运行

| 文件 | 当前能力 |
|---|---|
| [navigation/navigation_interface.py](navigation/navigation_interface.py) | 已知空闲通道内 waypoint 闭环、超时停止 |
| [run_navigation.py](run_navigation.py) | 独立导航入口，支持预设航点或栅格规划 |
| [navigation/grid_navigation.py](navigation/grid_navigation.py) | 用静态栅格规划并跟踪路线，支持连续巡检点 |

在项目根目录运行：

```bash
.venv/bin/python 01_navigation/run_navigation.py --headless
# GUI 查看底盘运动
.venv/bin/python 01_navigation/run_navigation.py
```

## 静态栅格导航与巡检

显式加上 `--planner grid` 启用移植的 A* 和路径跟踪基线。省略目标时，使用厨房场景的默认航点。

```bash
.venv/bin/python 01_navigation/run_navigation.py --headless --planner grid
```

重复指定 `--goal X Y YAW` 可按顺序巡检。坐标在 MuJoCo 世界系，位置单位为米，航向为弧度。加上 `--capture-dir` 后，每次停车都保存一张头部 RGB，并在 JSON 输出中列出路径、实际停车位姿、仿真时间和图片路径。抓取任务仍使用原有航点接口。

```bash
.venv/bin/python 01_navigation/run_navigation.py --planner grid \
  --goal 3.55 -1.30 0 --goal 2.20 -1.30 0 \
  --timeout 150 --capture-dir out/navigation-inspection
```

栅格以 5 cm 分辨率投影 MuJoCo 碰撞几何，覆盖起点和目标附近的区域。净空检查使用机器人当前碰撞外形的包络圆，导航期间机械臂保持固定。可以直达时使用直线，否则用八邻域 A* 规划。底盘闭环使用 `RobotInterface.get_state()` 提供的仿真真值位姿，以 `move_base(v,w)` 执行，遇到不可达、净空不足或超时会发送零速。

基线适用于平坦静态场景。碰撞包围盒可能让窄通道变得不可达，地图在任务开始时生成一次。LiDAR 定位、动态避障和自动重规划待后续接入。

导航回归测试覆盖直行、绕障、堵塞、转向停车和超时。锁定版本的官方 Stretch 模型在简化 MuJoCo 场景中通过了直行、绕障和堵塞停车，绕行到点误差为 2.94 cm / 1.97°。完整 RoboCasa 厨房和相机渲染待联验。

```bash
.venv/bin/python -m pytest -q 05_benchmark/tests/test_navigation_core.py 05_benchmark/tests/test_grid_navigation.py
```

## 本组任务

- **01.1 路径规划与避障**：以现有静态栅格规划为基线，继续接入传感器定位、动态障碍检测和局部避障。
- **01.2 目标接近**：输入世界坐标目标，考虑桌面边缘和机械臂工作空间，输出停靠 `[x,y,yaw]`。
- **01.3 失败返回与日志**：区分路径不可达、超时、定位失效；输出最终位姿与停止原因。

## 交接和验收

06 组提供视觉目标，02 组调用本组导航；07 组提供 `RobotInterface.move_base(v,w)`。底盘速度单位为 m/s、rad/s；本组不直接修改机械臂目标。

可先用仿真真值和已知地图开发。建议覆盖空闲、单障碍、完全堵塞三种路径，终点误差 ≤5 cm、朝向 ≤5°；异常时停止底盘。当前接口为 `navigate(robot, waypoints, timeout=45)`，新增结果结构需与 02/05 共同确定。
