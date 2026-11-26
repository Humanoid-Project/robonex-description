#!/usr/bin/env python3
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from build_isaac_urdf import VARIANTS, build

COLLISION = "mesh"


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Build the RoboNex Isaac URDF with mesh collision.")
    parser.add_argument("--variant", choices=VARIANTS, default=VARIANTS[0])
    parser.parse_args()
    build(COLLISION)
