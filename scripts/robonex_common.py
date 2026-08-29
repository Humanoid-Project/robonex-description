from __future__ import annotations

import math
import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
URDF_PATH = os.path.join(ROOT, "urdf", "robonex.urdf")
LOOPS_PATH = os.path.join(ROOT, "loop_closures.yaml")
MESH_DIR = os.path.join(ROOT, "meshes")



@dataclass
class Geom:
    mesh: str
    xyz: tuple
    rpy: tuple
    scale: tuple
    material: str = ""


@dataclass
class Link:
    name: str
    mass: float = 0.0
    com: tuple = (0.0, 0.0, 0.0)
    inertia: tuple = (0.0,) * 6
    visuals: list = field(default_factory=list)
    collisions: list = field(default_factory=list)


@dataclass
class Joint:
    name: str
    jtype: str
    parent: str
    child: str
    xyz: tuple
    rpy: tuple
    axis: tuple = (1.0, 0.0, 0.0)
    lower: float = -math.pi
    upper: float = math.pi
    effort: float = 0.0
    velocity: float = 0.0



def _triple(node, attr, default=(0.0, 0.0, 0.0)):
    if node is None or node.get(attr) is None:
        return default
    return tuple(float(v) for v in node.get(attr).split())


def _parse_geoms(link_el, tag):
    out = []
    for el in link_el.findall(tag):
        mesh = el.find("geometry/mesh")
        if mesh is None:
            continue
        origin = el.find("origin")
        mat = el.find("material")
        out.append(Geom(
            mesh=os.path.basename(mesh.get("filename")),
            xyz=_triple(origin, "xyz"),
            rpy=_triple(origin, "rpy"),
            scale=_triple(mesh, "scale", (1.0, 1.0, 1.0)),
            material=(mat.get("name") or "") if mat is not None else "",
        ))
    return out


def load_urdf(path=URDF_PATH):
    root = ET.parse(path).getroot()

    links = {}
    for el in root.findall("link"):
        lk = Link(name=el.get("name"))
        inertial = el.find("inertial")
        if inertial is not None:
            lk.mass = float(inertial.find("mass").get("value"))
            lk.com = _triple(inertial.find("origin"), "xyz")
            i = inertial.find("inertia")
            lk.inertia = (
                float(i.get("ixx")), float(i.get("ixy")), float(i.get("ixz")),
                float(i.get("iyy")), float(i.get("iyz")), float(i.get("izz")),
            )
        lk.visuals = _parse_geoms(el, "visual")
        lk.collisions = _parse_geoms(el, "collision")
        links[lk.name] = lk

    joints = {}
    for el in root.findall("joint"):
        origin = el.find("origin")
        j = Joint(
            name=el.get("name"),
            jtype=el.get("type"),
            parent=el.find("parent").get("link"),
            child=el.find("child").get("link"),
            xyz=_triple(origin, "xyz"),
            rpy=_triple(origin, "rpy"),
        )
        ax = el.find("axis")
        if ax is not None:
            j.axis = _triple(ax, "xyz", (1.0, 0.0, 0.0))
        lim = el.find("limit")
        if lim is not None:
            j.lower = float(lim.get("lower", -math.pi))
            j.upper = float(lim.get("upper", math.pi))
            j.effort = float(lim.get("effort", 0.0))
            j.velocity = float(lim.get("velocity", 0.0))
        joints[j.name] = j

    child_to_joint = {j.child: j for j in joints.values()}
    base = [n for n in links if n not in child_to_joint]
    if len(base) != 1:
        raise ValueError("expected exactly one root link, found %s" % base)

    return links, joints, base[0]


def children_of(joints):
    out = {}
    for j in joints.values():
        out.setdefault(j.parent, []).append(j)
    return out



def _mini_yaml(text):
    data, stack = {}, []
    cur_key, cur_list, cur_item = None, None, None

    def coerce(v):
        v = v.strip()
        if v.startswith("[") and v.endswith("]"):
            body = v[1:-1].strip()
            return [float(x) for x in body.split(",")] if body else []
        try:
            return float(v)
        except ValueError:
            return v

    for raw in text.splitlines():
        line = raw.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        indent = len(line) - len(line.lstrip())
        s = line.strip()

        if indent == 0 and s.endswith(":"):
            cur_key = s[:-1]
            cur_list = []
            data[cur_key] = cur_list
            cur_item = None
        elif s.startswith("- "):
            body = s[2:]
            if ":" in body:
                cur_item = {}
                cur_list.append(cur_item)
                k, v = body.split(":", 1)
                cur_item[k.strip()] = coerce(v)
            else:
                cur_list.append(coerce(body))
                cur_item = None
        elif ":" in s and cur_item is not None:
            k, v = s.split(":", 1)
            cur_item[k.strip()] = coerce(v)
    return data


def load_loops(path=LOOPS_PATH):
    with open(path, "r", encoding="utf-8") as f:
        text = f.read()
    try:
        import yaml
        return yaml.safe_load(text)
    except Exception:
        return _mini_yaml(text)



def rpy_to_quat(rpy):
    r, p, y = rpy
    cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)
    cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
    cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def fmt(vals, nd=6):
    return " ".join(("%." + str(nd) + "g") % v for v in vals)
