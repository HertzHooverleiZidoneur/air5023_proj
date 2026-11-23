"""TEAM-05: repeat the fixed scripted baseline in isolated episode directories."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--with-images", action="store_true")
    args = parser.parse_args()
    if args.episodes < 1 or args.output.exists():
        parser.error("Require positive --episodes and a new --output directory")
    args.output.mkdir(parents=True)
    rows = []
    for i in range(args.episodes):
        episode = args.output.resolve() / f"episode_{i:04d}"
        episode.mkdir()
        command = [sys.executable, str(ROOT / "02_manipulation_vla/run_task.py"),
                   "--headless", "--output", str(episode)]
        if not args.with_images:
            command.append("--no-cameras")
        start = time.monotonic()
        with (episode / "run.log").open("w") as log:
            try:
                process = subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, timeout=300)
                exit_code, reason = process.returncode, None
            except subprocess.TimeoutExpired:
                exit_code, reason = 124, "wall_clock_timeout"
        metrics_path = episode / "metrics.json"
        metrics = json.loads(metrics_path.read_text()) if metrics_path.exists() else None
        success = exit_code == 0 and bool(metrics and metrics.get("success"))
        rows.append({"episode": i, "success": success, "exit_code": exit_code,
                     "reason": reason or (None if success else "see_episode_log"),
                     "wall_seconds": time.monotonic() - start, "metrics": metrics})
    report = {"controller": "scripted_oracle_baseline", "scene_variation": False,
              "note": "Repeated fixed scene; measures repeatability, not cross-scene generalization",
              "episodes": rows, "success_rate": sum(r["success"] for r in rows) / len(rows)}
    (args.output / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"episodes": len(rows), "success_rate": report["success_rate"]}))
    return 0 if all(row["success"] for row in rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
