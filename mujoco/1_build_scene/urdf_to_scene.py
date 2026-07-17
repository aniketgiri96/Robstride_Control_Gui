"""Convert the Onshape URDF (+ its STL meshes) into a MuJoCo scene.xml.

MuJoCo can parse a URDF, but this Onshape export needs two fixes before it is
usable for teleop:

  * ``package://main_assembly/meshes/*.stl`` paths MuJoCo does not understand -
    we point its compiler at the real ``meshes/`` folder and strip the path, and
  * a URDF carries no actuators, so the dashboard would show no sliders - we add
    one ``<position>`` servo per revolute joint.

Pipeline: wrap the URDF with a ``<mujoco><compiler.../></mujoco>`` block so the
meshes resolve, let MuJoCo compile it, save the result as MJCF, then splice in a
ground plane, a light, and the actuators. The hinge joints keep their URDF names
(``revolute_1``..) so the GUI bridge's joint map lines up one-to-one.

Usage (run from the repo root):
    python mujoco/1_build_scene/urdf_to_scene.py \
        mujoco/1_build_scene/main_assembly.urdf \
        mujoco/1_build_scene/scene.xml
    python mujoco/3_teleop/mujoco_dashboard.py mujoco/1_build_scene/scene.xml
"""

from __future__ import annotations

import os
import sys
import tempfile
import xml.etree.ElementTree as ET

try:
    import mujoco
except ImportError:
    sys.exit("mujoco is not installed - run: pip install -r requirements-sim.txt")


def _find_meshdir(urdf_path: str) -> str:
    """Locate the meshes folder for a ``package://<pkg>/meshes/...`` URDF.

    Onshape lays the package out as ``<pkg>/{urdf,meshes,launch}``; the URDF may
    also sit at the repo root with the package beside it. Try the obvious spots.
    """
    here = os.path.dirname(os.path.abspath(urdf_path))
    candidates = [
        os.path.join(here, "meshes"),           # urdf/ sibling: <pkg>/urdf -> ../meshes? no
        os.path.join(os.path.dirname(here), "meshes"),  # <pkg>/urdf/x.urdf -> <pkg>/meshes
        os.path.join(here, "main_assembly", "meshes"),  # root copy -> ./main_assembly/meshes
    ]
    for c in candidates:
        if os.path.isdir(c):
            return c
    sys.exit("could not find the meshes/ folder near " + urdf_path)


def _wrapped_urdf(urdf_path: str, meshdir: str) -> str:
    """Write a temp copy of the URDF with a MuJoCo compiler block prepended."""
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    mj = ET.Element("mujoco")
    comp = ET.SubElement(mj, "compiler")
    comp.set("meshdir", meshdir)
    comp.set("strippath", "true")        # package://.../Part_1.stl -> Part_1.stl
    comp.set("balanceinertia", "true")   # tolerate Onshape's tiny/degenerate inertias
    comp.set("discardvisual", "false")
    comp.set("fusestatic", "false")      # keep welded links as bodies (clearer tree)
    root.insert(0, mj)
    fd, tmp = tempfile.mkstemp(suffix=".urdf")
    os.close(fd)
    tree.write(tmp)
    return tmp


def build(urdf_path: str, out_path: str) -> None:
    meshdir = _find_meshdir(urdf_path)
    tmp = _wrapped_urdf(urdf_path, meshdir)
    try:
        model = mujoco.MjModel.from_xml_path(tmp)      # compiles + resolves meshes
        mujoco.mj_saveLastXML(out_path, model)         # dump full MJCF
    finally:
        os.unlink(tmp)

    # Collect the hinge (revolute) joint names straight from the compiled model.
    hinges = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
              for j in range(model.njnt)
              if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE]

    tree = ET.parse(out_path)
    root = tree.getroot()

    world = root.find("worldbody")
    light = ET.Element("light", {"pos": "0 0 2", "dir": "0 0 -1",
                                 "diffuse": "0.8 0.8 0.8"})
    floor = ET.Element("geom", {"name": "floor", "type": "plane",
                                "size": "2 2 0.1", "rgba": "0.9 0.9 0.9 1"})
    world.insert(0, floor)
    world.insert(0, light)

    actuator = root.find("actuator")
    if actuator is None:
        actuator = ET.SubElement(root, "actuator")
    for name in hinges:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        lo, hi = model.jnt_range[jid]
        ET.SubElement(actuator, "position", {
            "name": f"{name}_act", "joint": name, "kp": "15",
            "ctrlrange": f"{lo:.6g} {hi:.6g}",
        })

    tree.write(out_path, xml_declaration=True, encoding="unicode")
    print(f"wrote {out_path}")
    print(f"  meshes: {meshdir}")
    print(f"  {len(hinges)} actuated joint(s): {', '.join(hinges)}")


def main() -> None:
    if len(sys.argv) < 2:
        sys.exit("usage: python urdf_to_scene.py <robot.urdf> [scene.xml]")
    urdf = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) > 2 else "scene.xml"
    build(urdf, out)


if __name__ == "__main__":
    main()
