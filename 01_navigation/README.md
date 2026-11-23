# 01 · Navigation 导航组

负责底盘导航、避障和可操作接近位姿。负责人：待认领。总分工见 [协作手册](../docs/development/README.md)。

## 现有文件和运行

| 文件 | 当前能力 |
|---|---|
| [navigation/navigation_interface.py](navigation/navigation_interface.py) | 已知空闲通道内 waypoint 闭环、超时停止 |
| [run_navigation.py](run_navigation.py) | 只执行导航，不执行抓取 |

在项目根目录运行：

```bash
.venv/bin/python 01_navigation/run_navigation.py --headless
# GUI 查看底盘运动
.venv/bin/python 01_navigation/run_navigation.py
```

## 本组任务

- **01.1 路径规划与避障**：在本组 `navigation/` 增加地图、规划和局部避障；当前没有 SLAM 或动态障碍处理。
- **01.2 目标接近**：输入世界坐标目标，考虑桌面边缘和机械臂工作空间，输出停靠 `[x,y,yaw]`。
- **01.3 失败返回与日志**：区分路径不可达、超时、定位失效；输出最终位姿与停止原因。

## 交接和验收

06 组提供视觉目标，02 组调用本组导航；07 组提供 `RobotInterface.move_base(v,w)`。底盘速度单位为 m/s、rad/s；本组不直接修改机械臂目标。

可先用仿真真值和已知地图开发。建议覆盖空闲、单障碍、完全堵塞三种路径，终点误差 ≤5 cm、朝向 ≤5°；异常时停止底盘。当前接口为 `navigate(robot, waypoints, timeout=45)`，新增结果结构需与 02/05 共同确定。
