#!/usr/bin/env python3
"""Generate the pillar_gap_corridor scenario map.

Scenario (flight is southbound, i.e. along -y, start at the north end):

    y in [ 14.0, -0.6]   pillar field      6 x 5 jittered pillar grid
    y in [ -0.6, -5.6]   reformation gap   5.4 m free, the cluster reforms
    y in [ -5.6,-10.6]   entrance funnel   free half-width 6.0 m -> 1.2 m
    y in [-10.6,-19.6]   narrow corridor   inner half-width 1.2 m, 9 m long
    y in [-19.6,-30.0]   exit bay          goal disk lands here

The funnel and corridor walls reach the map window edge (|x| = 10 m), so the
3 m opening is the ONLY passage: the cluster cannot fly around the walls.  That
requires the map window to keep its default width (map_size_x = 20).

Companion world/launch parameters (the map window is centred on the origin):

    roslaunch phase_offset_sim_bringup phase_offset_world_pillar_gap.launch
    roslaunch phase_offset_sim_bringup phase_offset_swarm.launch \
      agent_count:=15 publish_goals:=true \
      formation_goal_translation_y:=-47.0 map_size_y:=80

    start centre (0, 20, 1)  -> disk y[17, 23], sensing box y[12, 28]
    goal  centre (0, -27.0)  -> disk y[-30, -24] (4.4 m south of the corridor
                                exit at y = -19.6, so no goal sits in a wall)
    map_size_y 80            -> world y[-40, 40] covers both sensing boxes
                                (start box y[12, 28], goal box y[-38, -22])

The map is written on the same 0.1 m lattice and ASCII .PCD format as
pillar.pcd, so map_pub / local_sensing consume it unchanged.
"""

import argparse
import math
import os

import numpy as np

# --- scenario geometry (metres) ------------------------------------------
PILLAR_FIELD_WEST_X = -7.2      # pillar field extents (unchanged outer footprint)
PILLAR_FIELD_EAST_X = 7.2
PILLAR_FIELD_SOUTH_Y = 0.2
PILLAR_FIELD_NORTH_Y = 12.2
# Even jittered grid instead of the authentic pillar.pcd crop: the crop packed
# six pillars into the west edge and left x ~ -3 .. +1 empty.  Spacing 2.4 m
# with 0.6-1.2 m pillars keeps every corridor between pillar faces at least
# 1.2 m wide, and the jitter/skip/size draw keeps the field looking scattered.
PILLAR_SPACING = 2.6
PILLAR_JITTER = 0.30
PILLAR_SKIP_PROBABILITY = 0.12
PILLAR_SIZES = (0.6, 0.8, 1.0, 1.2)
PILLAR_SEED = 20260914
GAP_SOUTH_Y = -5.6              # free reformation gap
FUNNEL_SOUTH_Y = -10.6          # funnel: half-width 6.0 -> 1.5 m
CORRIDOR_SOUTH_Y = -19.6        # narrow corridor end (9 m long)
CORRIDOR_HALF_WIDTH = 1.2       # inner half-width -> 2.4 m clear opening
FUNNEL_ENTRY_HALF_WIDTH = 6.0
# Both walls run out to the map window edge, so the opening is the only way
# through.  map_size_x must stay at its 20 m default for this to hold.
MAP_HALF_WIDTH = 10.0
SKIN_THICKNESS = 0.2            # dense layer on the inner face (blocks ESDF)
INTERIOR_STEP = 5               # coarse lattice steps inside the wall body

PILLAR_Z_MIN, PILLAR_Z_TOP = -0.5, 2.5      # pillar height, on the 0.1 lattice
WALL_Z_MIN, WALL_Z_MAX = -0.9, 2.9         # wall solid on the 0.1 m lattice
STEP = 0.1                                  # lattice step of both sources


def _even_axis(west, east, spacing):
    """Evenly spaced, centred coordinates filling ``[west, east]``."""
    count = int(math.floor((east - west) / spacing)) + 1
    offset = 0.5 * ((east - west) - (count - 1) * spacing)
    return west + offset + np.arange(count) * spacing


def pillar_section():
    """Deterministic jittered pillar field, evenly spread over the footprint.

    The authentic pillar.pcd crop packed six pillars against the west edge and
    left x ~ -3 .. +1 nearly empty.  A centred grid with jitter, mixed pillar
    sizes and a few skipped cells keeps the same "scattered pillar field" look
    while giving every part of the field (and every corridor between pillar
    faces, >= 0.8 m) the same density.
    """
    rng = np.random.default_rng(PILLAR_SEED)
    columns = _even_axis(PILLAR_FIELD_WEST_X, PILLAR_FIELD_EAST_X,
                         PILLAR_SPACING)
    rows = _even_axis(PILLAR_FIELD_SOUTH_Y, PILLAR_FIELD_NORTH_Y,
                      PILLAR_SPACING)
    z_steps = np.arange(_steps(PILLAR_Z_MIN), _steps(PILLAR_Z_TOP) + 1)
    points = []
    for row_y in rows:
        for column_x in columns:
            if rng.random() < PILLAR_SKIP_PROBABILITY:
                continue
            size = float(PILLAR_SIZES[int(rng.integers(len(PILLAR_SIZES)))])
            centre_x = round((column_x + rng.uniform(-PILLAR_JITTER,
                                                     PILLAR_JITTER)) / STEP)
            centre_y = round((row_y + rng.uniform(-PILLAR_JITTER,
                                                  PILLAR_JITTER)) / STEP)
            half = _steps(0.5 * size)
            for x_step in range(centre_x - half, centre_x + half + 1):
                for y_step in range(centre_y - half, centre_y + half + 1):
                    for z_step in z_steps:
                        points.append((x_step * STEP, y_step * STEP,
                                       z_step * STEP))
    return np.asarray(points, dtype=float)


def corridor_half_width(y_value):
    """Inner half-width of the free channel at ``y_value``."""
    if y_value >= GAP_SOUTH_Y:
        return None                      # no wall here (gap / pillar field)
    if y_value >= FUNNEL_SOUTH_Y:
        span = GAP_SOUTH_Y - FUNNEL_SOUTH_Y
        ratio = (GAP_SOUTH_Y - y_value) / span
        return FUNNEL_ENTRY_HALF_WIDTH + ratio * (
            CORRIDOR_HALF_WIDTH - FUNNEL_ENTRY_HALF_WIDTH)
    if y_value >= CORRIDOR_SOUTH_Y:
        return CORRIDOR_HALF_WIDTH
    return None


def wall_section():
    """Funnel + corridor walls that seal the map window.

    The funnel half-width is continuous, so it is snapped *outwards* to the
    nearest lattice column; the free channel is therefore never narrower than
    the nominal geometry.  Each wall has a dense 0.2 m skin on the inner face
    (everything the ESDF / tube builder can ever see) and a coarser filled
    body behind it, which keeps the cloud small enough to publish comfortably.
    """
    y_steps = np.arange(_steps(GAP_SOUTH_Y), _steps(CORRIDOR_SOUTH_Y) - 1, -1)
    z_steps = np.arange(_steps(WALL_Z_MIN), _steps(WALL_Z_MAX) + 1)
    skin_offsets = range(_steps(SKIN_THICKNESS))
    outer_step = _steps(MAP_HALF_WIDTH)
    points = []
    for index, y_step in enumerate(y_steps):
        y_value = y_step * STEP
        half = corridor_half_width(y_value)
        if half is None:
            continue
        inner = int(math.ceil(half / STEP - 1e-9))
        # dense inner skin, both walls
        for offset in skin_offsets:
            for z_step in z_steps:
                points.append(((inner + offset) * STEP, y_value, z_step * STEP))
                points.append((-(inner + offset) * STEP, y_value, z_step * STEP))
        # coarse body behind the skin
        if index % INTERIOR_STEP:
            continue
        body = list(range(inner + _steps(SKIN_THICKNESS), outer_step + 1,
                          INTERIOR_STEP))
        if not body or body[-1] != outer_step:
            body.append(outer_step)
        for x_step in body:
            for z_step in z_steps[::INTERIOR_STEP]:
                points.append((x_step * STEP, y_value, z_step * STEP))
                points.append((-x_step * STEP, y_value, z_step * STEP))
    return np.asarray(points, dtype=float)


def _steps(value):
    """Exact lattice index of ``value`` (the map lives on a 0.1 m grid)."""
    steps = int(round(value / STEP))
    if abs(steps * STEP - value) > 1e-9:
        raise ValueError("%r is not on the %r m lattice" % (value, STEP))
    return steps


def write_pcd(path, points):
    lines = [
        "# .PCD v0.7 - Point Cloud Data file format",
        "VERSION 0.7",
        "FIELDS x y z",
        "SIZE 4 4 4",
        "TYPE F F F",
        "COUNT 1 1 1",
        "WIDTH %d" % len(points),
        "HEIGHT 1",
        "VIEWPOINT 0 0 0 1 0 0 0",
        "POINTS %d" % len(points),
        "DATA ascii",
    ]
    for x_value, y_value, z_value in points:
        lines.append("%.1f %.1f %.1f" % (x_value, y_value, z_value))
    with open(path, "w") as handle:
        handle.write("\n".join(lines))
        handle.write("\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    here = os.path.dirname(os.path.abspath(__file__))
    parser.add_argument("--output", default=os.path.join(
        here, os.pardir, "resource", "pillar_gap_corridor.pcd"))
    args = parser.parse_args()

    pillars = pillar_section()
    walls = wall_section()
    points = np.vstack([pillars, walls])
    write_pcd(args.output, points)

    print("pillar field  : %d points, x in [%.2f, %.2f] y in [%.2f, %.2f]" % (
        len(pillars), pillars[:, 0].min(), pillars[:, 0].max(),
        pillars[:, 1].min(), pillars[:, 1].max()))
    print("corridor walls: %d points" % len(walls))
    print("written       : %s (%d points)" % (args.output, len(points)))
    print("extent        : x[%.2f, %.2f] y[%.2f, %.2f] z[%.2f, %.2f]" % (
        points[:, 0].min(), points[:, 0].max(),
        points[:, 1].min(), points[:, 1].max(),
        points[:, 2].min(), points[:, 2].max()))
    tunnel = points[(points[:, 1] <= FUNNEL_SOUTH_Y)
                    & (points[:, 1] >= CORRIDOR_SOUTH_Y)]
    tunnel_half = float(np.abs(tunnel[:, 0]).min())
    print("corridor skin : |x| >= %.2f m in the straight section (%.1f m clear)" %
          (tunnel_half, 2.0 * tunnel_half))
    print("wall extent   : |x| <= %.2f m (seals the map window)" %
          float(np.abs(points[points[:, 1] < GAP_SOUTH_Y][:, 0]).max()))
    print("free gap      : %.2f m between southmost pillar and funnel mouth" %
          (pillars[:, 1].min() - GAP_SOUTH_Y))
    # The whole point of the layout: the opening is the only passage, and
    # nothing may sit inside the free channel.
    assert math.isclose(tunnel_half, CORRIDOR_HALF_WIDTH, abs_tol=1e-9)
    assert float(np.abs(points[points[:, 1] < GAP_SOUTH_Y][:, 0]).max()) \
        == MAP_HALF_WIDTH


if __name__ == "__main__":
    main()
