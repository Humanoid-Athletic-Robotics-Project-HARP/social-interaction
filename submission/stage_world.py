"""Write the MuJoCo scenes for one of our worlds (submission/worlds/NAME.xml).

  python3 submission/stage_world.py NAME MODEL_DIR OUT_DIR

MODEL_DIR holds him-sim2real-management's G1 (g1_robot.xml and its meshes,
from submission/build.sh). OUT_DIR gets a copy of it and:
  NAME.xml          for HIM's simulator (--scene): the G1, its floor and the
                    world's fixed obstacles. Movers are left out: HIM's sim
                    wouldn't move them, so they'd stand still where they start.
  NAME_fixed.xml    the world without its movers, included by NAME.xml
  NAME_all.xml      the G1 in the world with its movers, for scoring (course.py)
  image/            for our image's simulated lidar: the world with its movers
                    (world.xml) and the scene that includes it (lidar_scene.xml)
"""

import re
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

WORLDS = Path(__file__).resolve().parent / "worlds"


def scene(name, world_file):
    return f"""<mujoco model="harp_{name}">
  <include file="g1_robot.xml"/>
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <worldbody>
    <geom name="safety_floor" type="plane" size="0 0 0.05" friction="1.15"/>
  </worldbody>
  <include file="{world_file}"/>
</mujoco>
"""


def main(name, model_dir, out_dir, world=None):
    """world: the world's file, by default submission/worlds/NAME.xml (course.py passes an
    older version of it to rebuild an old run's scene)."""
    world = Path(world) if world else WORLDS / (name + ".xml")
    if not world.is_file():
        raise SystemExit(f"No world {name}: {world} doesn't exist")
    out = Path(out_dir)
    shutil.copytree(model_dir, out, dirs_exist_ok=True)

    # The comments' usage lines have "--", which MuJoCo reads but XML parsers don't.
    root = ET.fromstring(re.sub(r"<!--.*?-->", "", world.read_text(), flags=re.S))
    worldbody = root.find("worldbody")
    for body in worldbody.findall("body"):
        if body.get("mocap") == "true":
            worldbody.remove(body)
    ET.ElementTree(root).write(out / f"{name}_fixed.xml")
    (out / f"{name}.xml").write_text(scene(name, f"{name}_fixed.xml"))

    image = out / "image" / name
    image.mkdir(parents=True, exist_ok=True)
    shutil.copy(world, image / "world.xml")
    (image / "lidar_scene.xml").write_text(scene(name, "world.xml"))
    (out / f"{name}_all.xml").write_text(scene(name, f"image/{name}/world.xml"))
    return out / f"{name}.xml"


if __name__ == "__main__":
    if len(sys.argv) != 4:
        raise SystemExit(__doc__)
    print(main(*sys.argv[1:]))
