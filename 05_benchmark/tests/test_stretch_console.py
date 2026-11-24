"""HTTP and owner-thread lifecycle checks for the live Stretch console."""
from io import BytesIO
from threading import Event, Thread
from time import monotonic, sleep
from urllib.request import Request, urlopen
import json

from PIL import Image

from navigation.console import Console, Server
from navigation.docking import Station
from navigation.tracking import Pose, Route


class _Grid:
    resolution = 0.5
    cells = {(0, 0): -3.0, (1, 0): 3.0, (0, 1): -3.0}


class _Profile:
    name = "fake"
    grid = _Grid()
    keepouts = ()
    stations = {"A": Station(Pose(2.0, 1.0, 0.0), 0.6)}


class _Robot:
    class Data:
        time = 0.0
    control_hz = 100.0

    def __init__(self):
        self.data = self.Data()

    def get_camera_image(self, _name):
        import numpy as np
        return np.full((5, 7, 3), (30, 100, 180), dtype=np.uint8)


class _Session:
    def __init__(self, index):
        self.robot = _Robot()
        self.pose = Pose(float(index), 0.0, 0.0)
        self.calls = 0

    def tick(self):
        self.robot.data.time += 0.01
        self.pose = Pose(self.pose.x + 0.001, self.pose.y, self.pose.yaw)

    def stop(self):
        pass

    def go(self, goal, *, station, cancelled, on_plan, on_update):
        if goal == _Profile.stations["A"].pose:
            assert station is _Profile.stations["A"]
        self.calls += 1
        on_plan(Route(((self.pose.x, self.pose.y), (goal.x, goal.y))))
        for _ in range(80):
            self.pose = Pose(goal.x, goal.y, goal.yaw or 0.0)
            on_update(self.pose, "track")
            if cancelled():
                return None
            if self.calls > 1:
                return object()
            sleep(0.005)
        return object()


class _Manager:
    def __init__(self, counter):
        self.counter = counter
        self.session = None

    def __enter__(self):
        self.counter[0] += 1
        self.session = _Session(self.counter[0])
        return self.session

    def __exit__(self, *_args):
        return False


def _json(url, payload=None):
    if payload is None:
        return json.load(urlopen(url, timeout=2))
    request = Request(url, data=json.dumps(payload).encode(),
                      headers={"Content-Type": "application/json"}, method="POST")
    return json.load(urlopen(request, timeout=2))


def _wait_for(url, predicate, timeout=3):
    end = monotonic() + timeout
    while monotonic() < end:
        state = _json(url)
        if predicate(state):
            return state
        sleep(0.01)
    raise AssertionError(f"Timed out waiting for console state: {_json(url)}")


def test_http_run_cancel_next_goal_and_explicit_reset_share_one_session():
    count = [0]
    manager = _Manager(count)
    profile = _Profile()
    session = manager.__enter__()
    console = Console(profile, session, manager)
    stop = Event()
    worker = Thread(target=console.serve_commands,
                    args=(lambda *_args, **_kwargs: _Manager(count),),
                    kwargs={"stop_event": stop}, daemon=True)
    server = Server(console, 0)
    http = Thread(target=server.serve_forever, daemon=True)
    worker.start()
    http.start()
    base = f"http://127.0.0.1:{server.server_port}"
    try:
        initial = _json(base + "/api/state")
        sleep(0.06)
        idle = _json(base + "/api/state")
        assert idle["sim_time"] > initial["sim_time"]
        assert idle["pose"] != initial["pose"]

        first = _json(base + "/api/run", {"x": 1.0, "y": 1.0})
        running = _wait_for(base + "/api/state", lambda state: state["status"] == "running")
        assert running["path"]
        cancelled = _json(base + "/api/cancel", {"id": first["id"]})
        assert cancelled["status"] == "cancelling"
        _wait_for(base + "/api/state", lambda state: state["status"] == "cancelled")
        assert count[0] == 1

        second = _json(base + "/api/run", {"station": "A"})
        arrived = _wait_for(base + "/api/state", lambda state: state["status"] == "arrived")
        assert arrived["id"] == second["id"]
        assert arrived["path"] and arrived["trajectory"]
        assert count[0] == 1
        png = urlopen(base + "/api/snapshot", timeout=2).read()
        assert Image.open(BytesIO(png)).format == "PNG"

        episode = arrived["episode"]
        _json(base + "/api/reset", {})
        reset = _wait_for(base + "/api/state", lambda state: state["status"] == "idle" and state["episode"] != episode)
        assert reset["trajectory"] == [] and reset["path"] == []
        assert count[0] == 2

        after_reset = _json(base + "/api/run", {"x": 1.5, "y": 1.0})
        _wait_for(base + "/api/state", lambda state: state["status"] == "arrived")
        assert after_reset["id"]
        assert count[0] == 2
    finally:
        stop.set()
        worker.join(timeout=2)
        server.shutdown()
        server.server_close()
        http.join(timeout=2)
        console.close()
