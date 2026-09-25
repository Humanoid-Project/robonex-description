"""Re-express every exported STL in URDF axes (X forward, Y left, Z up) and in the frame of its link.

stl_raw/<part>.stl is in the part's component frame (mm). For each vertex:
    p_world = T_occ * p_local                      (fusion world, mm)
    p_link  = R_FU @ p_world - origin_of_link      (URDF axes, link frame, mm)
Normals are rotated with the same rotation. Output keeps millimetres (URDF scale 0.001).
"""
import os
import struct

import numpy as np

from lower_model import HERE, R_FU, T, LINK_PARTS, build, link_origins_f, mesh_of, to_urdf

RAW = os.path.join(HERE, "stl_raw")
OUT = os.path.join(os.path.dirname(HERE), "meshes")

DT = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])


def read_stl(path):
    b = open(path, "rb").read()
    n = struct.unpack("<I", b[80:84])[0]
    if 84 + 50 * n != len(b):
        raise ValueError("%s is not a binary STL" % path)
    return np.frombuffer(b[84:], dtype=DT, count=n).copy()


def write_stl(path, tris, header):
    with open(path, "wb") as f:
        f.write(header.encode("ascii")[:80].ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        f.write(tris.tobytes())


def main():
    J, _ = build()
    origin_f = link_origins_f(J)
    os.makedirs(OUT, exist_ok=True)
    meshes = {}  # mesh name -> (link, [parts])
    for link, parts in LINK_PARTS.items():
        for p in parts:
            name = mesh_of(p)
            if name in meshes and meshes[name][0] != link:
                raise ValueError("mesh %s spans links %s and %s" % (name, meshes[name][0], link))
            meshes.setdefault(name, (link, []))[1].append(p)
    for name in sorted(meshes):
        link, parts = meshes[name]
        chunks = []
        for part in parts:
            tris = read_stl(os.path.join(RAW, part + ".stl"))
            M = T[part]
            Rw = R_FU @ M[:3, :3]
            tw = R_FU @ M[:3, 3] - to_urdf(origin_f[link])
            out = tris.copy()
            out["v"] = (tris["v"].astype(np.float64).reshape(-1, 3) @ Rw.T + tw).reshape(-1, 3, 3).astype(np.float32)
            out["n"] = (tris["n"].astype(np.float64) @ Rw.T).astype(np.float32)
            chunks.append(out)
        out = np.concatenate(chunks)
        write_stl(os.path.join(OUT, name + ".stl"), out,
                  "RoboNex Ver.2 %s, mm, URDF axes, frame of %s" % (name, link))
        v = out["v"].reshape(-1, 3)
        print("%-24s <- %-40s frame %-22s tris %6d  min %s  max %s" % (
            name, "+".join(parts), link, len(out), np.round(v.min(0), 2), np.round(v.max(0), 2)))


if __name__ == "__main__":
    main()
