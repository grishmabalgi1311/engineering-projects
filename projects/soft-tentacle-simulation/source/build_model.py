"""Build two copies of the calibrated-in-future planar arm plus a free ball."""
from copy import deepcopy
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parent


def build(config):
    tree = ET.parse(ROOT / "model/arm_reference.xml")
    root = tree.getroot()
    root.set("model", "Project2_reduced_wave_ball_UNCALIBRATED")
    root.find("option").set("timestep", str(config["dt_s"]))
    visual = root.find("visual")
    ET.SubElement(visual, "global", offwidth="1200", offheight="800")
    ET.SubElement(visual, "map", znear="0.002")
    wb = root.find("worldbody")
    arm = deepcopy(wb.find("body"))
    tendons = deepcopy(root.find("tendon"))
    contacts = deepcopy(root.find("contact"))
    root.remove(root.find("tendon"))
    root.remove(root.find("actuator"))
    if root.find("contact") is not None:
        root.remove(root.find("contact"))
    for el in list(wb):
        wb.remove(el)
    L, W = config["tank_length_m"], config["tank_width_m"]
    depth, level = config["water_depth_m"], config["surface_z_m"]
    wall_height = config.get("tank_height_m", depth + .05)
    ET.SubElement(wb, "light", pos="0 0 1", dir="0 0 -1", diffuse="0.8 0.8 0.8")
    ET.SubElement(wb, "geom", name="tank_floor", type="box",
                  pos=f"0 0 {level-depth-.01}", size=f"{L/2} {W/2} .01",
                  rgba="0.12 0.2 0.25 1", contype="1", conaffinity="1")
    for axis, span, other in ((0, L, W), (1, W, L)):
        for sign in (-1, 1):
            pos = [0, 0, level-depth+wall_height/2]
            pos[axis] = sign*(span/2+.005)
            size = [other/2, .005, wall_height/2]
            if axis == 0:
                size = [.005, other/2, wall_height/2]
            ET.SubElement(wb, "geom", name=f"wall_{axis}_{sign}", type="box",
                          pos=" ".join(map(str,pos)), size=" ".join(map(str,size)),
                          rgba="0.5 0.7 0.8 0.25", contype="1", conaffinity="1")
    new_tendons = ET.SubElement(root, "tendon")
    new_contacts = ET.SubElement(root, "contact")
    actuators = ET.SubElement(root, "actuator")
    for i in range(1):
        prefix = f"arm{i+1}_"
        body = deepcopy(arm)
        # Rename locally-owned elements and references, retaining shared mesh/class assets.
        names = {e.get("name") for e in body.iter() if e.get("name")}
        for e in body.iter():
            if e.get("name"):
                e.set("name", prefix+e.get("name"))
            if e.tag == "geom" and e.get("mass"):
                if not config.get("preserve_original_arm_mass", False):
                    e.set("mass", str(float(e.get("mass"))*config["arm_mass_kg"]/.049999))
                # External contacts are enabled; parent-link contacts use existing pairs.
                e.set("contype", "2")
                e.set("conaffinity", "1")
                e.set("rgba", "0.95 0.72 0.16 1" if i == 0 else "0.9 0.35 0.18 1")
        x,y = config["arm_bases_xy_m"][i]
        body.set("pos", f"{x} {y} {config['arm_z_m']}")
        body.set("euler", f"0 0 {config['arm_yaws_deg'][i]}")
        wb.append(body)
        for tendon in tendons:
            t = deepcopy(tendon)
            t.set("name", prefix+t.get("name"))
            for site in t.findall("site"):
                site.set("site", prefix+site.get("site"))
            new_tendons.append(t)
        if contacts is not None:
            for pair in contacts:
                e = deepcopy(pair)
                for attr in ("geom1", "geom2"):
                    if e.get(attr) in names:
                        e.set(attr,prefix+e.get(attr))
                if e.get("name"):
                    e.set("name",prefix+e.get("name"))
                new_contacts.append(e)
        for j in (1,2):
            ET.SubElement(actuators,"motor",name=f"{prefix}pull{j}",
                          tendon=f"{prefix}cable{j}",gear="-1",ctrllimited="true",
                          ctrlrange=f"0 {config['max_tension_N']}",forcelimited="true",
                          forcerange=f"0 {config['max_tension_N']}")
    ball = ET.SubElement(wb,"body",name="ball",pos=f"0 0 {level+.01}")
    ET.SubElement(ball,"freejoint",name="ball_free")
    ET.SubElement(ball,"geom",name="ball_geom",type="sphere",size=str(config["ball_radius_m"]),
                  mass=str(config["ball_mass_kg"]),rgba="0.9 0.1 0.55 1",
                  contype="1",conaffinity="3",friction="0.2 0.005 0.0001")
    # Target marker is visual only.
    tx,ty=config["target_xy_m"]
    ET.SubElement(wb,"site",name="target",type="cylinder",pos=f"{tx} {ty} {level+.002}",
                  size=f"{config['goal_radius_m']} .001",rgba="0.1 1 0.2 0.35")
    ET.indent(tree,space="  ")
    # Use text for in-memory compile. Assets still resolve from model/ using from_xml_path.
    return tree


if __name__ == "__main__":
    cfg=json.loads((ROOT/'config.json').read_text())
    path=ROOT/'model/two_tentacles_ball.xml'
    build(cfg).write(path,encoding="utf-8",xml_declaration=True)
    print(path)
