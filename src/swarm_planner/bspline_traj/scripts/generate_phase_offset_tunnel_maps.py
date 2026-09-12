#!/usr/bin/env python3
"""Generate "single long narrow tunnel" PCD maps for the swarm gate scenario.

The map is one wall block that spans the whole per-agent map width in x, with a
single rectangular tunnel through it along y:

    |<--- 30 m wall (blocks everything) --->|
                     +----+                    <- tunnel: width w, length L
    y = +L/2  -------|    |-------
    y = -L/2  -------|    |-------
                     +----+

Units fly along y, so they must queue and pass the tunnel; everywhere else the
wall is solid.  Compared with the upstream `narrow.pcd` (a 0.34 m thin slit)
the tunnel can be made arbitrarily long and narrow.

Usage:
  python3 generate_phase_offset_tunnel_maps.py --outdir <resource dir>
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


def grid_xz(x0, x1, y, z0, z1, step, zstep):
    """Grid on the x-z plane at a fixed y (block front/back face)."""
    pts = []
    x = x0
    while x <= x1 + 1e-9:
        z = z0
        while z <= z1 + 1e-9:
            pts.append((x, y, z))
            z += zstep
        x += step
    return pts


def grid_yz(x, y0, y1, z0, z1, step, zstep):
    """Grid on the y-z plane at a fixed x (tunnel wall face)."""
    pts = []
    y = y0
    while y <= y1 + 1e-9:
        z = z0
        while z <= z1 + 1e-9:
            pts.append((x, y, z))
            z += zstep
        y += step
    return pts


def make_tunnel(width, length, span=15.0, height=3.0, step=0.1, zstep=0.1):
    """Wall surfaces sampled at the map resolution (0.1 m).

    The default `step`/`zstep` must not be coarser than the SDFMap resolution:
    the map marks a voxel occupied only where a point falls, and
    `obstacles_inflation` (0.099 m at 0.1 m resolution) inflates by a single
    voxel, so a 0.5 m sampling leaves 0.3 m tall slits the UAVs can fly through.
    """
    half_w = width / 2.0
    y0 = -length / 2.0
    y1 = length / 2.0
    pts = []
    for sign in (-1.0, 1.0):
        x_inner = sign * half_w
        x_outer = sign * span
        xa, xb = (x_inner, x_outer) if sign > 0.0 else (x_outer, x_inner)
        # Tunnel side wall: the surface the tube/preview measures.
        pts += grid_yz(x_inner, y0, y1, 0.0, height, step, zstep)
        # Block front and back faces (block travel around the tunnel).
        pts += grid_xz(xa, xb, y1, 0.0, height, step, zstep)
        pts += grid_xz(xa, xb, y0, 0.0, height, step, zstep)
        # Outer end caps so the block reads as solid from outside too.
        pts += grid_yz(x_outer, y0, y1, 0.0, height, step, zstep)
    return pts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--outdir", default=".")
    args = ap.parse_args()
    os.makedirs(args.outdir, exist_ok=True)

    variants = (
        # (file, tunnel width [m], tunnel length [m])
        ("phase_offset_tunnel_w12_l6.pcd", 1.2, 6.0),
        ("phase_offset_tunnel_w10_l10.pcd", 1.0, 10.0),
    )
    for name, width, length in variants:
        pts = make_tunnel(width, length)
        write_pcd(os.path.join(args.outdir, name), pts)
        print("   tunnel width %.2f m, length %.2f m, wall spans x=+-15 m, "
              "z 0..3 m" % (width, length))


if __name__ == "__main__":
    main()
