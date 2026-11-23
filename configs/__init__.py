from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]


def load(name):
    return yaml.safe_load((ROOT / "configs" / f"{name}.yaml").read_text())
