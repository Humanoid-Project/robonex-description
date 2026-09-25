import xml.etree.ElementTree as ET

import numpy as np


def load(path):
    r = ET.parse(path).getroot()
    links, joints = {}, {}
    for l in r.findall("link"):
        i = l.find("inertial")
        d = {"name": l.get("name"), "meshes": [], "col": []}
        if i is not None:
            I = i.find("inertia").attrib
            d["mass"] = float(i.find("mass").get("value"))
            d["com"] = np.array([float(v) for v in i.find("origin").get("xyz").split()])
            d["com_rpy"] = i.find("origin").get("rpy")
            d["I"] = np.array([[float(I["ixx"]), float(I["ixy"]), float(I["ixz"])],
                               [float(I["ixy"]), float(I["iyy"]), float(I["iyz"])],
                               [float(I["ixz"]), float(I["iyz"]), float(I["izz"])]])
        for tag, key in (("visual", "meshes"), ("collision", "col")):
            for v in l.findall(tag):
                m = v.find("geometry/mesh")
                o = v.find("origin")
                d[key].append((m.get("filename") if m is not None else None,
                               o.get("xyz") if o is not None else None, o.get("rpy") if o is not None else None,
                               m.get("scale") if m is not None else None))
        links[d["name"]] = d
    for j in r.findall("joint"):
        o = j.find("origin")
        a = j.find("axis")
        lim = j.find("limit")
        joints[j.get("name")] = {
            "type": j.get("type"), "parent": j.find("parent").get("link"), "child": j.find("child").get("link"),
            "xyz": np.array([float(v) for v in o.get("xyz").split()]),
            "rpy": np.array([float(v) for v in (o.get("rpy") or "0 0 0").split()]),
            "axis": np.array([float(v) for v in a.get("xyz").split()]) if a is not None else None,
            "limit": {k: float(v) for k, v in lim.attrib.items()} if lim is not None else None,
        }
    return links, joints


def global_origins(joints):
    child = {j["child"]: j for j in joints.values()}
    P = {}

    def g(link):
        if link in P:
            return P[link]
        if link not in child:
            P[link] = np.zeros(3)
            return P[link]
        j = child[link]
        P[link] = g(j["parent"]) + j["xyz"]
        return P[link]

    for j in joints.values():
        g(j["child"])
    return P
