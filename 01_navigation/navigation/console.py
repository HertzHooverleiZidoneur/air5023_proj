"""Loopback web console for the persistent Stretch navigation session."""

from base64 import b64encode
from dataclasses import asdict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from io import BytesIO
import json
import logging
import math
from pathlib import Path
from threading import RLock
from uuid import uuid4

from .grid_navigation import GridNavigationError
from .map_exports import rasterize
from .tracking import Pose

WEB_ROOT = Path(__file__).with_name("web")
ASSETS = {"/": ("index.html", "text/html; charset=utf-8"),
          "/app.js": ("app.js", "text/javascript; charset=utf-8"),
          "/styles.css": ("styles.css", "text/css; charset=utf-8")}
ACTIVE = {"planning", "running", "cancelling", "resetting"}


class Console:
    """HTTP threads enqueue requests and read snapshots. The owner drives the robot."""

    def __init__(self, profile, session, session_manager):
        self.profile = profile
        self.session = session
        self.session_manager = session_manager
        self.lock = RLock()
        self.command = None
        self.cancel_requested = False
        self.snapshot_png = None
        self.state = self._initial_state("仿真已启动，等待导航目标。")
        raster = rasterize(profile.grid)
        self.map_data = dict(origin=raster.origin, resolution=raster.resolution,
                             width=raster.width, height=raster.height,
                             pixels=b64encode(raster.pixels).decode("ascii"),
                             stations={name: asdict(station.pose) for name, station in profile.stations.items()},
                             keepouts=profile.keepouts, name=profile.name)

    def _initial_state(self, detail):
        return dict(episode=uuid4().hex, id=None, status="idle", detail=detail, phase=None,
                    goal=None, path=[], trajectory=[], pose=asdict(self.session.pose),
                    sim_time=float(self.session.robot.data.time), seq=0)

    def snapshot(self):
        with self.lock:
            return dict(self.state, trajectory=list(self.state["trajectory"]))

    def submit(self, kind, *, goal=None, station=None):
        with self.lock:
            if self.state["status"] in ACTIVE:
                raise RuntimeError("当前操作尚未结束。")
            shown_goal = None if goal is None else dict(x=goal.x, y=goal.y, station=station)
            self.state.update(id=uuid4().hex, status="resetting" if kind == "reset" else "planning",
                              detail=None, phase=None, goal=shown_goal, path=[], trajectory=[])
            self.command = kind, goal, station
            self.cancel_requested = False
            return self.snapshot()

    def cancel(self, job_id):
        with self.lock:
            if job_id != self.state["id"] or self.state["status"] not in ACTIVE - {"resetting"}:
                raise RuntimeError("任务已结束或编号已失效。")
            self.cancel_requested = True
            self.state["status"] = "cancelling"
            return self.snapshot()

    def cancelled(self):
        with self.lock:
            return self.cancel_requested

    def update(self, **values):
        with self.lock:
            self.state.update(values)

    def serve_commands(self, open_session, viewer=False, capture_dir=None, stop_event=None, *, cameras=True):
        """Advance the same world even when no command is queued."""
        while stop_event is None or not stop_event.is_set():
            with self.lock:
                command, self.command = self.command, None
            if command is not None:
                kind, goal, label = command
                if kind == "reset":
                    self.close()
                    manager = open_session(self.profile, viewer=viewer, realtime=True, cameras=cameras)
                    self.session = manager.__enter__()
                    self.session_manager = manager
                    with self.lock:
                        self.snapshot_png = None
                        self.state = self._initial_state("仿真已重置。")
                    continue
                self._run(goal, self.profile.stations.get(label), capture_dir, cameras)
            self.session.tick()
            self._progress(self.session.pose)
            if stop_event is not None:
                stop_event.wait(0.001)

    def _run(self, goal, station, capture_dir, cameras):
        try:
            visit = self.session.go(goal, station=station, cancelled=self.cancelled,
                                    on_plan=lambda route: self.update(path=route.waypoints),
                                    on_update=self._progress)
        except TimeoutError:
            self.update(status="error", detail="导航超时，底盘已停止。")
            return
        except GridNavigationError as error:
            self.update(status="unreachable", detail=str(error))
            return
        except InterruptedError:
            raise
        except Exception:
            logging.exception("Navigation failed")
            self.update(status="error", detail="导航中断，底盘已停止。")
            return

        detail = "已到达目标。"
        if visit is not None and cameras:
            try:
                self._capture(capture_dir)
            except Exception:
                logging.exception("Arrival image failed")
                detail = "已到达目标，截图失败。"
                with self.lock:
                    self.snapshot_png = None
        with self.lock:
            if visit is None or self.cancel_requested:
                self.state.update(status="cancelled", phase="cancelled", detail="导航已取消，机器人已停稳。")
            else:
                self.state.update(status="arrived", phase="arrived", detail=detail)

    def snapshot_image(self):
        with self.lock:
            return self.snapshot_png

    def close(self):
        manager, self.session_manager = self.session_manager, None
        if manager is not None:
            manager.__exit__(None, None, None)

    def _progress(self, pose, phase=None):
        with self.lock:
            self.state.update(pose=asdict(pose), sim_time=float(self.session.robot.data.time),
                              seq=self.state["seq"] + 1)
            if phase is not None:
                self.state.update(status="cancelling" if self.cancel_requested else "running", phase=phase)
                point = (pose.x, pose.y)
                trajectory = self.state["trajectory"]
                if not trajectory or trajectory[-1] != point:
                    trajectory.append(point)

    def _capture(self, directory):
        from PIL import Image

        image = Image.fromarray(self.session.robot.get_camera_image("head")).convert("RGB")
        with BytesIO() as output:
            image.save(output, format="PNG")
            data = output.getvalue()
        with self.lock:
            self.snapshot_png = data
        if directory is not None:
            directory.mkdir(parents=True, exist_ok=True)
            (directory / f"arrival-{self.state['id']}.png").write_bytes(data)


class Handler(BaseHTTPRequestHandler):
    def _local(self):
        host = self.headers.get("Host")
        port = self.server.server_port
        return (host in (f"127.0.0.1:{port}", f"localhost:{port}")
                and self.headers.get("Origin") in (None, f"http://{host}"))

    def _send(self, code, payload, content="application/json; charset=utf-8"):
        raw = payload if isinstance(payload, bytes) else json.dumps(payload, ensure_ascii=False,
                                                                     allow_nan=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", content)
        self.send_header("Content-Length", str(len(raw)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(raw)

    def do_GET(self):
        if not self._local():
            return self._send(403, {"error": "仅允许本机访问。"})
        console = self.server.console
        if self.path == "/api/state":
            return self._send(200, console.snapshot())
        if self.path == "/api/map":
            return self._send(200, console.map_data)
        if self.path == "/api/snapshot":
            image = console.snapshot_image()
            if image is not None:
                return self._send(200, image, "image/png")
            return self._send(404, {"error": "尚无到达截图。"})
        if self.path in ASSETS:
            name, mime = ASSETS[self.path]
            return self._send(200, (WEB_ROOT / name).read_bytes(), mime)
        self._send(404, {"error": "页面不存在。"})

    def do_POST(self):
        if not self._local():
            return self._send(403, {"error": "仅允许本机操作。"})
        try:
            if self.headers.get_content_type() != "application/json":
                raise ValueError("需要 JSON 请求。")
            size = int(self.headers.get("Content-Length", "0"))
            if not 0 < size <= 2048:
                raise ValueError("请求格式无效。")
            payload = json.loads(self.rfile.read(size))
            if not isinstance(payload, dict):
                raise ValueError("请求格式无效。")
            state = self._dispatch(payload)
        except (ValueError, TypeError, OverflowError) as error:
            return self._send(400, {"error": str(error)})
        except RuntimeError as error:
            return self._send(409, {"error": str(error)})
        self._send(202, state)

    def _dispatch(self, payload):
        console = self.server.console
        fields = set(payload)
        if self.path == "/api/run":
            label = payload.get("station")
            if fields == {"station"} and isinstance(label, str) and label in console.profile.stations:
                return console.submit("run", goal=console.profile.stations[label].pose, station=label)
            if fields == {"x", "y"} and all(type(value) in (int, float) and math.isfinite(value)
                                             for value in payload.values()):
                return console.submit("run", goal=Pose(float(payload["x"]), float(payload["y"]), None))
            raise ValueError("请选择地图位置或有效工位。")
        if self.path == "/api/cancel" and fields == {"id"}:
            return console.cancel(payload["id"])
        if self.path == "/api/reset" and not fields:
            return console.submit("reset")
        raise ValueError("接口或请求无效。")

    def log_message(self, *_args):
        pass


class Server(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, console, port):
        self.console = console
        super().__init__(("127.0.0.1", port), Handler)
