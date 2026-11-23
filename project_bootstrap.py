"""Expose valid Python package names from numbered team-owned directories.

CLI entrypoints call activate() before importing project modules. Third-party
packages remain installed in the existing virtual environment.
"""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent
TEAM_DIRECTORIES = (
    "01_navigation", "02_manipulation_vla", "03_data_generate", "04_s2r",
    "05_benchmark", "06_data_process", "07_extension",
)


def activate():
    for directory in reversed((".", *TEAM_DIRECTORIES)):
        path = str((ROOT / directory).resolve())
        if path not in sys.path:
            sys.path.insert(0, path)
    return ROOT
