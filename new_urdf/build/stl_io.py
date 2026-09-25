import struct

import numpy as np

DT = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])


def read_stl(path):
    b = open(path, "rb").read()
    n = struct.unpack("<I", b[80:84])[0]
    return np.frombuffer(b[84:], dtype=DT, count=n).copy()


def write_stl(path, tris, header):
    with open(path, "wb") as f:
        f.write(header.encode("ascii")[:80].ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        f.write(tris.tobytes())
