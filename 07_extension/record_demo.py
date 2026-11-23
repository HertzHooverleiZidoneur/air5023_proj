"""TEAM-07: record an actual continuous rollout with Chinese status labels.

20 fps H.264, 1280x720, 1x simulation time; no synthesized robot motion/audio.
Optional encoder: .tools/uv pip install --python .venv/bin/python -r requirements-video.txt
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from project_bootstrap import activate
activate()
os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("NUMBA_CACHE_DIR", str(ROOT / ".cache/numba"))
os.environ.setdefault("MPLCONFIGDIR", str(ROOT / ".cache/matplotlib"))

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageOps
from robot.robot_interface import RobotInterface
from perception.camera import configure_overview
from evaluation.success_checker import SuccessChecker
from tasks.pick_red_cup import run

PHASES = {
    "initialize": ("01  初始化与观测", "采集头部 / 腕部 RGB，准备底盘和机械臂。", "TEAM-06 / 02"),
    "navigate": ("02  导航到岛台", "沿预设 waypoint 闭环导航；目标坐标来自仿真真值。", "TEAM-01"),
    "pregrasp": ("03  调整预抓取位姿", "升降、伸臂和手腕联合调整，IK 使用官方机器人运动学。", "TEAM-02"),
    "approach": ("04  接近红色杯子", "缓慢接近，补偿夹爪闭合时的指尖回缩。", "TEAM-02"),
    "close": ("05  闭合夹爪", "检查左右手指与杯子的实际接触。", "TEAM-02 / 05"),
    "lift": ("06  抬升杯子", "由关节控制与接触物理完成抬升。", "TEAM-02"),
    "verify": ("07  检查并保持", "双侧接触 + 抬升至少 5 cm + 无环境支撑 + 保持至少 1 s。", "TEAM-05"),
    "finished": ("08  验收结果", "", "TEAM-05 / 07"),
}


class DemoRecorder:
    def __init__(self, robot, destination, font_path, log_dir):
        import imageio_ffmpeg
        self.robot, self.destination = robot, destination
        self.fps = robot.control_hz
        self.phase = "initialize"
        self.events = []
        self.result = None
        self.checker = None
        self.frames = 0
        self.samples = []
        self.poster_saved = False
        self.fonts = {size: ImageFont.truetype(str(font_path), size, index=2)
                      for size in (15, 17, 18, 22, 26, 30)}
        self.canvas = self.make_canvas()
        robot.model.vis.global_.offwidth = max(robot.model.vis.global_.offwidth, 888)
        robot.model.vis.global_.offheight = max(robot.model.vis.global_.offheight, 500)
        self.renderer = mujoco.Renderer(robot.model, height=500, width=888)
        self.camera = mujoco.MjvCamera()
        configure_overview(self.camera)
        self.option = mujoco.MjvOption()
        self.option.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = False
        self.ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
        self.log = (log_dir / "encode.log").open("w")
        self.encoder = subprocess.Popen([
            self.ffmpeg, "-nostdin", "-v", "error", "-n",
            "-f", "rawvideo", "-pixel_format", "rgb24", "-video_size", "1280x720",
            "-framerate", str(self.fps), "-i", "pipe:0", "-an",
            "-vf", "scale=out_color_matrix=bt709:out_range=tv,setsar=1",
            "-c:v", "libx264", "-preset", "fast", "-crf", "20", "-pix_fmt", "yuv420p",
            "-color_primaries", "bt709", "-color_trc", "bt709", "-colorspace", "bt709",
            "-color_range", "tv", "-movflags", "+faststart", str(destination)
        ], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=self.log)

    def text(self, draw, xy, message, size=18, color="#dbe6f2"):
        draw.text(xy, message, font=self.fonts[size], fill=color)

    def make_canvas(self):
        canvas = Image.new("RGB", (1280, 720), "#0b1220")
        draw = ImageDraw.Draw(canvas)
        self.text(draw, (24, 13), "STRETCH 3  /  家庭移动抓取 Demo", 30, "#f1f5f9")
        self.text(draw, (24, 56), "MuJoCo + RoboCasa  ·  实际仿真录制  ·  脚本基线 / VLA 接口预留", 17, "#92a8c1")
        self.text(draw, (24, 83), "场景总览  /  Navigate to the table and pick up the red cup.", 17)
        self.text(draw, (936, 102), "头部 RGB  ·  原始光学方向", 17)
        self.text(draw, (936, 338), "腕部 RGB", 17)
        draw.rounded_rectangle((24, 622, 1256, 699), radius=10, fill="#162337")
        return canvas

    def on_phase(self, phase, sim_time):
        self.phase = phase
        self.events.append({"phase": phase, "simulation_time": sim_time,
                            "video_time": self.frames / self.fps})
        if phase == "navigate":
            self.checker = SuccessChecker(self.robot)

    def capture(self):
        canvas = self.canvas.copy()
        self.renderer.update_scene(self.robot.data, camera=self.camera, scene_option=self.option)
        canvas.paste(Image.fromarray(self.renderer.render()), (24, 110))
        for name, xy in (("head", (936, 132)), ("wrist", (936, 368))):
            # Contain preserves source image aspect and the documented native orientation.
            image = Image.fromarray(self.robot.get_camera_image(name))
            image = ImageOps.pad(image, (320, 180), color="#111b2b")
            canvas.paste(image, xy)
        draw = ImageDraw.Draw(canvas)
        title, description, task_id = PHASES[self.phase]
        self.text(draw, (1010, 18), f"仿真 {self.robot.data.time:05.1f} s", 22)
        self.text(draw, (1010, 54), f"1×  ·  {self.fps:g} fps  ·  无音轨", 17, "#92a8c1")
        self.text(draw, (44, 629), title, 26, "#72ddcb")
        self.text(draw, (1050, 637), task_id, 17, "#94a3b8")
        if self.result is not None:
            passed = self.result["success"]
            description = (f"{'通过' if passed else '未通过'}：抬升 {self.result['lift_metres'] * 100:.1f} cm"
                           f"，持续保持 {self.result['held_seconds']:.1f} s。VLA 尚未接入。")
        self.text(draw, (44, 668), description, 17)
        if self.checker:
            metrics = self.checker.update()
            self.text(draw, (936, 570), f"抬升  {metrics['lift_metres'] * 100:5.1f} cm", 18)
            contacts = f"双侧接触  {'✓' if metrics['left_contact'] and metrics['right_contact'] else '—'}"
            contacts += f"    环境支撑  {'有' if metrics['environment_contact'] else '无'}"
            self.text(draw, (936, 595), contacts, 15)
        self.encoder.stdin.write(canvas.tobytes())
        self.samples.append({"frame": self.frames, "simulation_time": float(self.robot.data.time),
                             "image_time": self.robot._camera_time, "phase": self.phase})
        self.frames += 1
        if self.result is not None and not self.poster_saved:
            canvas.save(self.destination.with_name("demo_poster.png"))
            self.poster_saved = True

    def close(self):
        try:
            self.encoder.stdin.close()
            code = self.encoder.wait(timeout=60)
            if code:
                raise RuntimeError(f"Video encoder exited {code}; inspect encode.log")
        finally:
            self.renderer.close()
            self.log.close()


class RecordingRobot(RobotInterface):
    """Recording hooks observe finished control ticks; they do not change commands."""
    recorder = None

    def step(self, steps=1):
        for _ in range(steps):
            super().step()
            if self.recorder:
                self.recorder.capture()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=ROOT / "media/stretch_vla_demo.mp4")
    parser.add_argument("--run-dir", type=Path, default=ROOT / "out/video_run")
    parser.add_argument("--font", type=Path, default=Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"))
    parser.add_argument("--preview-seconds", type=float, default=0,
                        help="Renderer smoke test only; records initialization, no task success claim")
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Output exists. Choose a new --output path to preserve previous recordings.")
    if not args.font.exists():
        parser.error("Chinese font missing; pass --font pointing to a NotoSansCJK TTC file")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.run_dir.mkdir(parents=True, exist_ok=True)
    scene = ROOT / "scenes/kitchen/generated/kitchen.xml"
    metadata = json.loads(scene.with_name("metadata.json").read_text())
    result = None
    with RecordingRobot(scene, gui=False, realtime=False) as robot:
        recorder = DemoRecorder(robot, args.output, args.font, args.run_dir)
        robot.recorder = recorder
        try:
            if args.preview_seconds > 0:
                robot.set_joint_targets(lift=.55, head_pan=-1.57, head_tilt=-.45)
                robot.hold(args.preview_seconds)
            else:
                result = run(robot, metadata, args.run_dir, on_phase=recorder.on_phase)
                recorder.result = result
                recorder.on_phase("finished", float(robot.data.time))
                robot.hold(2)  # Actual continued physics, displaying the acceptance result.
        finally:
            robot.recorder = None
            recorder.close()
    manifest = {
        "video": args.output.name, "source": "continuous MuJoCo simulation rollout",
        "width": 1280, "height": 720, "fps": recorder.fps, "frames": recorder.frames,
        "duration_seconds": recorder.frames / recorder.fps, "playback_speed": 1.0,
        "audio": None, "controller": "scripted_oracle_baseline", "vla_loaded": False,
        "scene_sha256": hashlib.sha256(scene.read_bytes()).hexdigest(),
        "video_sha256": hashlib.sha256(args.output.read_bytes()).hexdigest(),
        "phase_events": recorder.events, "task_result": result,
        "upstream_assets": ["Hello Robot Stretch MuJoCo", "RoboCasa official assets"],
        "encoder": subprocess.check_output([recorder.ffmpeg, "-version"], text=True).splitlines()[0],
    }
    args.output.with_suffix(".json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
    (args.run_dir / "frame_times.json").write_text(json.dumps(recorder.samples, indent=2) + "\n")
    print(json.dumps(manifest, ensure_ascii=False, indent=2))
    return 0 if result is None or result["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
