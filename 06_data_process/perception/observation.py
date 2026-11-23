"""TEAM-06: observation contract, shared with TEAM-02 model adapters."""
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class Observation:
    instruction: str
    state: dict
    images: dict[str, np.ndarray]  # RGB uint8 HWC, native unrotated camera frame
    image_time: float  # simulation seconds, state may be newer than last image
    embodiment: str = "stretch3"
    schema_version: str = "1.0"
