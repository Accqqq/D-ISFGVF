#!/usr/bin/env python3
"""Generate reproducible phase-offset scenario maps as ASCII PCD files.

Usage:
  python3 generate_phase_offset_maps.py --outdir <dir>

Generated maps (all with the header metadata printed alongside):
  phase_offset_open.pcd                  open field, obstacles only near bounds
  phase_offset_split_merge.pcd           central obstacle splitting the flow
  phase_offset_corridor_two_wide.pcd     straight corridor, 2.0 m wide
  phase_offset_corridor_single_wide.pcd  straight corridor, 1.0 m wide
"""

import argparse
import os


def write_pcd(path, points):
    with open(path, "w") as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write("WIDTH %d\n" % len(points))
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write("POINTS %d\n" % len(points))
        f.write("DATA ascii\n")
        for p in points:
            f.write("%.4f %.4f %.4f\n" % (p[0], p[1], p[2]))
    print("wrote %s (%d points)" % (path, len(points)))


def grid_points(x0, x1, y0, y1, step, z=0.5):
    pts = []
    x = x0
    while x <= x1 + 1e-9:
        y = y0
        while y <= y1 + 1e-9:
            pts.append((x, y, z))
            y += step
        x += step
    return pts


def wall_points(x0, x1, y_center, thickness, step, z0=0.0, z1=3.0, zstep=0.1):
    pts = []
    x = x0
    while x <= x1 + 1e-9:
        for dy in (-thickness / 2.0, thickness / 2.0):
            z = z0
            while z <= z1 + 1e-9:
                pts.append((x, y_center + dy, z))
                z += zstep
        x += step
    return pts


def make_open():
    # Sparse floor points (z=0.05, below the z in [0.8, 1.2] flight band) let
    # the SDFMap/ESDF initialize, plus obstacles only in the four corners;
    # the flight corridor along y=0 (x in 0..16) is completely free.
    pts = []
    pts += grid_points(-9.0, 17.0, -14.0, 14.0, 2.0, z=0.05)
    for cx in (-9.5, 9.5):
        for cy in (-14.5, 14.5):
            pts += grid_points(cx - 0.5, cx + 0.5, cy - 0.5, cy + 0.5, 0.25,
                               z=0.5)
            pts += grid_points(cx - 0.5, cx + 0.5, cy - 0.5, cy + 0.5, 0.25,
                               z=2.0)
    return pts


def make_split_merge():
    pts = grid_points(-9.8, 9.8, -14.8, 14.8, 2.0, z=0.5)
    # central box obstacle splitting the flow: x in [-1.5, 1.5], y in [-1, 1]
    x = -1.5
    while x <= 1.5 + 1e-9:
        y = -1.0
        while y <= 1.0 + 1e-9:
            z = 0.0
            while z <= 2.5 + 1e-9:
                pts.append((x, y, z))
                z += 0.5
            y += 0.25
        x += 0.25
    return pts


def make_corridor(width, name):
    # Wall CENTERS at +-(width + thickness)/2 so the FREE gap equals width.
    half = (width + 0.5) / 2.0
    pts = wall_points(-10.0, 28.0, half, 0.5, 0.1)
    pts += wall_points(-10.0, 28.0, -half, 0.5, 0.1)
    return pts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--outdir", default=".")
    args = ap.parse_args()
    os.makedirs(args.outdir, exist_ok=True)

    write_pcd(os.path.join(args.outdir, "phase_offset_open.pcd"), make_open())
    write_pcd(os.path.join(args.outdir, "phase_offset_split_merge.pcd"),
              make_split_merge())
    write_pcd(os.path.join(args.outdir, "phase_offset_corridor_two_wide.pcd"),
              make_corridor(2.0, "two_wide"))
    write_pcd(os.path.join(args.outdir, "phase_offset_corridor_single_wide.pcd"),
              make_corridor(1.0, "single_wide"))

    meta = {
        "open": {"range": "x[-9.8,9.8] y[-14.8,14.8] z[0.5,2.0]",
                 "corridor_free": "y=0, x in [0,16]"},
        "split_merge": {"range": "x[-9.8,9.8] y[-14.8,14.8]",
                        "obstacle": "box [-1.5,1.5]x[-1.0,1.0] z 0..2.5"},
        "corridor_two_wide": {"width_m": 2.0, "x_range": "[-10,28]",
                              "wall_thickness": 0.5},
        "corridor_single_wide": {"width_m": 1.0, "x_range": "[-10,28]",
                                 "wall_thickness": 0.5},
    }
    with open(os.path.join(args.outdir, "phase_offset_maps_meta.yaml"),
              "w") as f:
        import yaml
        yaml.safe_dump(meta, f)
    print("metadata written")


if __name__ == "__main__":
    main()
