# 本机已有仿真环境与资产

核实日期：2026-11-28。此前仅检查 Windows checkout 就判断“本机没有场景/资产”不正确；本文件纠正该状态，避免重复安装。

## 两份路径

| 用途 | 位置 | 本次核实版本 |
|---|---|---|
| 当前开发源码 | `C:\Users\Administrator\Documents\ChatGPT\机器人数学基础\air5023_proj` | `532cee1`，含未提交 05 改动 |
| 已安装运行副本 | WSL Ubuntu-22.04 的 `/root/air5023_proj` | `60bdc73`，有本地 GUI 退出修复 |
| Python | `/root/air5023_proj/.venv/bin/python` | 3.10.19；MuJoCo 3.2.6；NumPy 1.23.3 |
| 下载缓存 | `/root/air5023_proj/.cache/assets/` | fixtures.zip、objaverse.zip、textures.zip 均存在 |
| 解压资产 | `/root/air5023_proj/third_party/stretch_mujoco/third_party/robocasa/robocasa/models/assets/` | 已用于本次实际模型加载 |
| 已生成厨房 | `/root/air5023_proj/scenes/kitchen/generated/kitchen.xml` | 本次新版 05 真实测试成功 |

Windows 资源管理器路径：`\\wsl.localhost\Ubuntu-22.04\root\air5023_proj`。Windows 代码 pull 不会自动更新这个 WSL 独立副本。普通受限工具账户看不到该 WSL 分发版时，也不能据此判定用户没有安装它。

历史运行证据位于开发仓库父目录的 `output/local_reproduction/`：`复现说明.md`、`verification.json`、安装/构建日志和最终图片。2026-11-24 的记录包含 GUI、纯物理和 RGB 红杯成功运行；它们与 2026-11-28 的新版 05 测试应分开引用。

## 项目已有下载入口

`03_data_generate/download_assets.py` 读取 `configs/assets.lock.json`，从 RoboCasa 官方 Hugging Face 备份下载三个包，固定 revision 为 `1b92c3d02ca4354984fec961357db0bff7b32166`。入口由 `07_extension/install.sh` 调用，兼容性说明在 `docs/compatibility.md`。

下载源：<https://huggingface.co/datasets/robocasa/robocasa-assets/tree/1b92c3d02ca4354984fec961357db0bff7b32166>。

`.venv/`、`.cache/`、`third_party/`、生成厨房和生成圆桌被 Git 忽略。因此新 clone/pull 没有这些生成物并不说明本机未安装。应优先复用已有环境和缓存。

## 新 pull 带来了什么

- `scenes/round_table/round_table.xml`：独立桌面参考场景，含瓶/罐/包装，不含 Stretch 任务链。
- `scenes/round_table/objects.xml` 及 `assets/`：G1 示例圆桌与瓶子的模型/纹理。
- `03_data_generate/build_round_table.py`：基于已有厨房生成含 Stretch 的 G1 抓取放置场景。
- `03_data_generate/add_round_table.py`、`build_challenge_scenes.py`：参考圆桌变体，以及圆桌/岛台共 16 个挑战条件生成器。
- `02_manipulation_vla/run_tcp_retarget.py`：固定停靠位置的 G1 轨迹重定向抓取放置示例。

“存在场景和生成器”“通过加载/静置检查”“控制器能完成该条件”是三个不同的验收项。尤其圆罐目标不同于 G1 瓶子和原红杯，不能因为 XML 可加载就宣布已有完整控制器。

## 直接复用 WSL 环境测试当前源码

在 Ubuntu-22.04 终端运行下列命令；无需把当前源码覆盖到 `/root/air5023_proj`。输出目录必须是新的。

```bash
cd /mnt/c/Users/Administrator/Documents/ChatGPT/机器人数学基础/air5023_proj
export NUMBA_CACHE_DIR=/root/air5023_proj/.cache/numba
export MPLCONFIGDIR=/root/air5023_proj/.cache/matplotlib
/root/air5023_proj/.venv/bin/python 05_benchmark/run_benchmark.py \
  --manifest 05_benchmark/examples/suite.local-wsl.json \
  --output 05_benchmark/artifacts/my_next_smoke --preflight
# 去掉 --preflight 执行真实单次测试。
```

`suite.local-wsl.json` 是本机专用示例，Linux 绝对路径不能直接由 Windows Python 解析成同一资源。正式可移植套件须单独整理路径及资产记录。
