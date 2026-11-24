"""Trinary occupancy rasters shared by the Web Console and PGM export."""

from dataclasses import dataclass
from pathlib import Path

from .planning import LOG_ODDS_LIMIT, StaticGrid


@dataclass(frozen=True)
class Raster:
    width: int
    height: int
    origin: tuple[float, float]
    resolution: float
    pixels: bytes


def rasterize(grid: StaticGrid) -> Raster:
    """Convert sparse cells to PGM pixels with +Y at the top."""
    xmin = min(x for x, _ in grid.cells)
    xmax = max(x for x, _ in grid.cells)
    ymin = min(y for _, y in grid.cells)
    ymax = max(y for _, y in grid.cells)
    width, height = xmax - xmin + 1, ymax - ymin + 1
    pixels = bytearray([205]) * (width * height)
    for (x, y), odds in grid.cells.items():
        if odds >= LOG_ODDS_LIMIT:
            pixels[(ymax - y) * width + x - xmin] = 0
        elif odds <= -LOG_ODDS_LIMIT:
            pixels[(ymax - y) * width + x - xmin] = 254
    return Raster(width, height, (xmin * grid.resolution, ymin * grid.resolution),
                  grid.resolution, bytes(pixels))


def export_map(grid: StaticGrid, output: Path):
    raster = rasterize(grid)
    pgm, yaml = output.with_suffix(".pgm"), output.with_suffix(".yaml")
    if pgm.exists() or yaml.exists():
        raise FileExistsError(f"Map export already exists at {output}")
    pgm.parent.mkdir(parents=True, exist_ok=True)
    with pgm.open("xb") as handle:
        handle.write(f"P5\n{raster.width} {raster.height}\n255\n".encode("ascii"))
        handle.write(raster.pixels)
    with yaml.open("x", encoding="utf-8") as handle:
        x, y = raster.origin
        handle.write(f"image: {pgm.name}\nresolution: {raster.resolution:g}\n"
                     f"origin: [{x:g}, {y:g}, 0.0]\nnegate: 0\n"
                     "occupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n")
