"""Read simulated D435 images and COIN-D4 planar scans on the physics clock."""
from pathlib import Path
import argparse
import json

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent
CONFIG = json.loads((ROOT / "sensors.json").read_text())


class SensorReader:
    def __init__(self, model):
        self.model = model
        self.renderer = mujoco.Renderer(model, height=360, width=640)
        self.last_tick = {}
        self.visual = mujoco.MjvOption()
        self.visual.geomgroup[:] = [1, 1, 1, 0, 0, 0]

    def close(self):
        self.renderer.close()

    def read(self, data):
        """Return only newly due streams; call after each physics step."""
        out = {}
        for name, cfg in CONFIG.items():
            tick = int(np.floor(data.time * cfg["rate"] + 1e-9))
            if self.last_tick.get(name) == tick:
                continue
            self.last_tick[name] = tick
            if cfg["kind"] == "lidar":
                out[name] = self.scan(data, cfg)
                continue
            depth = cfg["kind"] == "depth"
            if depth:
                self.renderer.enable_depth_rendering()
            else:
                self.renderer.disable_depth_rendering()
            self.renderer.update_scene(data, name, scene_option=self.visual)
            frame = self.renderer.render()
            if depth:
                frame[(frame < cfg["near"]) | (frame > cfg["far"])] = np.nan
            elif cfg["kind"].startswith("infra"):
                frame = np.rint(frame @ np.array([0.299, 0.587, 0.114])).astype(np.uint8)
            out[name] = frame
        return out

    def scan(self, data, cfg):
        site = data.site(cfg["frame"])
        count = cfg["samples"]
        angles = cfg["angle_min"] + cfg["increment"] * np.arange(count)
        local = np.column_stack((np.cos(angles), np.sin(angles), np.zeros(count)))
        dirs = np.ascontiguousarray(local @ site.xmat.reshape(3, 3).T)
        dist = np.empty(count)

        # The four scanner housings are baked into the chassis visual mesh.
        # Start each ray at its optical blind-zone boundary to avoid the cover.
        mask = np.array([1, 1, 1, 0, 0, 0], dtype=np.uint8)
        hit = np.empty(1, dtype=np.int32)
        for i, direction in enumerate(dirs):
            origin = site.xpos + cfg["near"] * direction
            value = mujoco.mj_ray(self.model, data, origin, direction, mask, 1, -1, hit)
            dist[i] = value + cfg["near"] if value >= 0 else np.inf

        dist[dist > cfg["far"]] = np.inf
        return dist.astype(np.float32)


def snapshot(folder):
    model = mujoco.MjModel.from_xml_path(str(ROOT / "scene.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    reader = SensorReader(model)
    try:
        frames = reader.read(data)
    finally:
        reader.close()
    folder.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(folder / "sensors.npz", **frames)
    from PIL import Image
    for name, frame in frames.items():
        if CONFIG[name]["kind"] == "color":
            Image.fromarray(frame).save(folder / (name + ".png"))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=Path("sensor_sample"))
    snapshot(parser.parse_args().output)

