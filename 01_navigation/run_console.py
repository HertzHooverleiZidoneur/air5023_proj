"""Run the local Stretch Web Console."""

import argparse
import os
from pathlib import Path
import sys
from threading import Thread

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from project_bootstrap import activate

activate()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, default=None,
                        help="navigation profile (default: tracked profile)")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--viewer", action="store_true", help="open the MuJoCo viewer")
    parser.add_argument("--no-cameras", action="store_true", help="skip head RGB snapshots")
    parser.add_argument("--capture-dir", type=Path,
                        help="save a PNG camera snapshot after each arrival")
    args = parser.parse_args()
    if args.capture_dir and args.no_cameras:
        parser.error("--capture-dir needs camera rendering")
    os.environ.setdefault("MUJOCO_GL", "glfw" if args.viewer or sys.platform == "win32" else "egl")
    from navigation.console import Console, Server
    from navigation.prototype import load_profile, open_session

    profile = load_profile(args.profile)
    cameras = not args.no_cameras
    manager = open_session(profile, viewer=args.viewer, realtime=True, cameras=cameras)
    session = manager.__enter__()
    console = Console(profile, session, manager)
    server = None
    http = None
    try:
        server = Server(console, args.port)
        http = Thread(target=server.serve_forever, name="stretch-console-http", daemon=True)
        http.start()
        print(f"Stretch Web Console: http://127.0.0.1:{server.server_port}", flush=True)
        console.serve_commands(open_session, args.viewer, args.capture_dir, cameras=cameras)
    except (KeyboardInterrupt, InterruptedError):
        pass
    finally:
        if server is not None and http is not None and http.is_alive():
            server.shutdown()
            http.join(timeout=3)
        if server is not None:
            server.server_close()
        console.close()


if __name__ == "__main__":
    main()
