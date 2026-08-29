#!/usr/bin/env python3
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from build_isaac_urdf import build

COLLISION = "mesh"


if __name__ == "__main__":
    build(COLLISION)
