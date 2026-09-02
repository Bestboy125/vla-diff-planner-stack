"""Headless smoke test for the local Isaac Sim source build."""

import json

from isaacsim import SimulationApp


simulation_app = SimulationApp({"headless": True})

import omni.usd
import torch
from pxr import UsdGeom


try:
    context = omni.usd.get_context()
    context.new_stage()
    simulation_app.update()

    stage = context.get_stage()
    cube = UsdGeom.Cube.Define(stage, "/World/SmokeTestCube")
    cube.CreateSizeAttr(1.0)

    for _ in range(10):
        simulation_app.update()

    cuda_available = torch.cuda.is_available()
    result = {
        "stage_created": stage is not None,
        "cube_path": str(cube.GetPath()),
        "cuda_available": cuda_available,
        "cuda_device": torch.cuda.get_device_name(0) if cuda_available else None,
        "torch_version": torch.__version__,
        "torch_cuda_version": torch.version.cuda,
    }
    print("ISAAC_SIM_SMOKE_OK " + json.dumps(result, ensure_ascii=False, sort_keys=True))
finally:
    simulation_app.close()
