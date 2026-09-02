"""Print selected RetroNeighborhood prim transforms in world coordinates."""

import os

from isaacsim import SimulationApp


simulation_app = SimulationApp({"headless": True})

from isaacsim.core.utils.stage import open_stage
from pxr import Usd, UsdGeom


SCENE_USD = os.environ.get(
    "AIRSTACK_SCENE_USD",
    r"D:\AirStackWSL\scenes\RetroNeighborhood\RetroNeighborhood_Export.usd",
)
TARGETS = (
    "/World/stage/Neighborhood/PowerLines/SM_PowerLine_4",
    "/World/stage/Neighborhood/PowerLines/SM_PowerLine_2",
    "/World/stage",
    "/World/Root",
)


def main() -> None:
    open_stage(SCENE_USD)
    for _ in range(10):
        simulation_app.update()
    stage = Usd.Stage.Open(SCENE_USD)
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    for path in TARGETS:
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            print(f"USD_WORLD_MISSING path={path}", flush=True)
            continue
        transform = cache.GetLocalToWorldTransform(prim)
        translation = transform.ExtractTranslation()
        print(
            "USD_WORLD "
            f"path={path} "
            f"translation=({translation[0]:.6f},{translation[1]:.6f},{translation[2]:.6f})",
            flush=True,
        )


try:
    main()
finally:
    simulation_app.close()
