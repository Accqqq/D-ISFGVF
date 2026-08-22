#!/usr/bin/env python3
"""Batch 10 analysis: summarize a phase-offset rosbag into JSON + a PNG.

Usage:
  python3 analyze_phase_offset_bag.py /tmp/po_figure8.bag [--outdir DIR]

Reads:
  /uav_i/sim/odom, /sim/odom            per-UAV trajectories
  /phase_offset_swarm/debug             ISF / allocator / continuation stats
  /phase_offset_swarm/conflict_state    tube conflict events
  /phase_offset_swarm/state             swarm state (communication volume)
  /phase_offset_swarm/path_event        path epochs
"""

import argparse
import io
import json
import math
import os
import sys

import rosbag


def collect(bag):
    odoms = {}          # robot_id -> list of (t, x, y, z)
    debug = {}          # robot_id -> list of dict
    conflicts = []
    states = []         # (t, robot_id)
    path_events = []
    msg_bytes = {}      # topic -> (count, bytes)

    for topic, msg, t in bag.read_messages():
        buff = io.BytesIO()
        msg.serialize(buff)
        size = buff.tell()
        c, b = msg_bytes.get(topic, (0, 0))
        msg_bytes[topic] = (c + 1, b + size)

        if topic.endswith("/sim/odom"):
            rid = 0
            if topic.startswith("/uav_"):
                rid = int(topic.split("/")[1].split("_")[1])
            odoms.setdefault(rid, []).append(
                (t.to_sec(), msg.pose.pose.position.x,
                 msg.pose.pose.position.y, msg.pose.pose.position.z))
        elif topic == "/phase_offset_swarm/debug":
            d = {
                "t": t.to_sec(),
                "robot_id": msg.robot_id,
                "phase": msg.phase,
                "delta": msg.delta,
                "e_perp": msg.e_perp_norm,
                "matched_residual": msg.matched_residual,
                "allocator_ms": msg.allocator_ms,
                "mode": msg.mode,
                "channel_beta": msg.channel_beta,
                "delta_jump": msg.delta_jump,
                "delta_r": msg.delta_r_norm,
                "min_neighbor_distance": msg.min_neighbor_distance,
                "min_pair_cbf_margin": msg.min_pair_cbf_margin,
                "min_tube_cbf_margin": msg.min_tube_cbf_margin,
                "curvature": msg.curvature,
            }
            debug.setdefault(msg.robot_id, []).append(d)
        elif topic == "/phase_offset_swarm/conflict_state":
            conflicts.append((t.to_sec(), msg.channel_beta, msg.active,
                              msg.conflict_state))
        elif topic == "/phase_offset_swarm/state":
            states.append((t.to_sec(), msg.robot_id))
        elif topic == "/phase_offset_swarm/path_event":
            path_events.append((t.to_sec(), msg.robot_id,
                                msg.event_sequence, msg.active))
    return odoms, debug, conflicts, states, path_events, msg_bytes


def rate(series, duration):
    if duration <= 0 or len(series) < 2:
        return 0.0
    return (len(series) - 1) / max(1e-6, series[-1] - series[0])


def stats_of(values):
    if not values:
        return {"n": 0}
    return {
        "n": len(values),
        "min": float(min(values)),
        "max": float(max(values)),
        "mean": float(sum(values) / len(values)),
    }


def min_inter_uav_distance(odoms):
    """Pairwise nearest-timestamp inter-UAV distance."""
    ids = sorted(odoms.keys())
    if len(ids) < 2:
        return None, 0, []
    mins = []
    for i in range(len(ids)):
        for j in range(i + 1, len(ids)):
            a, b = odoms[ids[i]], odoms[ids[j]]
            # two-pointer over timestamps
            bi = 0
            for pa in a:
                while bi + 1 < len(b) and b[bi + 1][0] < pa[0]:
                    bi += 1
                best = None
                for k in (bi - 1, bi, bi + 1):
                    if 0 <= k < len(b):
                        d = math.sqrt(
                            (pa[1] - b[k][1]) ** 2 +
                            (pa[2] - b[k][2]) ** 2 +
                            (pa[3] - b[k][3]) ** 2)
                        best = d if best is None else min(best, d)
                if best is not None:
                    mins.append(best)
    if not mins:
        return None, 0, []
    return min(mins), sum(1 for d in mins if d < 0.60), mins


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bagfile")
    ap.add_argument("--outdir", default=None)
    args = ap.parse_args()

    if not os.path.exists(args.bagfile):
        print("bag not found: %s" % args.bagfile)
        return 1
    outdir = args.outdir or os.path.dirname(os.path.abspath(args.bagfile))
    os.makedirs(outdir, exist_ok=True)

    bag = rosbag.Bag(args.bagfile)
    try:
        odoms, debug, conflicts, states, path_events, msg_bytes = collect(bag)
        info = bag.get_type_and_topic_info()
        duration = bag.get_start_time() and (
            bag.get_end_time() - bag.get_start_time())
    finally:
        bag.close()

    result = {
        "bag": os.path.basename(args.bagfile),
        "duration_s": round(duration, 3),
        "topics": {
            topic: {"messages": c, "bytes": b}
            for topic, (c, b) in sorted(msg_bytes.items())
        },
    }

    # Per-UAV odom + debug stats.
    per_uav = {}
    for rid, od in sorted(odoms.items()):
        if len(od) < 2:
            continue
        t0, t1 = od[0][0], od[-1][0]
        d = debug.get(rid, [])
        per_uav[rid] = {
            "odom_rate_hz": round(rate([p[0] for p in od], t1 - t0), 2),
            "distance_travelled_m": round(
                sum(math.dist(od[k][1:4], od[k - 1][1:4])
                    for k in range(1, len(od))), 3),
            "debug_samples": len(d),
            "e_perp_stats": stats_of([x["e_perp"] for x in d]),
            "matched_residual_max": stats_of(
                [x["matched_residual"] for x in d]).get("max", 0.0),
            "allocator_ms": stats_of([x["allocator_ms"] for x in d]),
            "delta": stats_of([x["delta"] for x in d]),
            "delta_jump_max": stats_of([x["delta_jump"] for x in d])
                .get("max", 0.0),
            "delta_r_max": stats_of([x["delta_r"] for x in d])
                .get("max", 0.0),
            "channel_beta_min": stats_of([x["channel_beta"] for x in d])
                .get("min", 1.0),
            "min_tube_cbf_margin": stats_of(
                [x["min_tube_cbf_margin"] for x in d]).get("min", 1e9),
            "min_pair_cbf_margin": stats_of(
                [x["min_pair_cbf_margin"] for x in d]).get("min", 1e9),
            "emergency_samples": sum(1 for x in d if x["mode"] == 4),
            "mode_counts": {
                str(m): sum(1 for x in d if x["mode"] == m)
                for m in sorted({x["mode"] for x in d})
            },
        }
    result["per_uav"] = per_uav

    min_d, n_coll, _ = min_inter_uav_distance(odoms)
    result["swarm"] = {
        "min_inter_uav_distance_m": round(min_d, 4) if min_d else None,
        "collision_count_under_0.6m": n_coll,
        "conflict_events": len(conflicts),
        "conflict_active_ratio": round(
            sum(1 for _, _, a, _ in conflicts if a) /
            max(1, len(conflicts)), 4) if conflicts else 0.0,
        "swarm_state_messages": len(states),
        "path_events": len(path_events),
        "communication_bytes_total": result["topics"]
            .get("/phase_offset_swarm/state", {}).get("bytes", 0),
    }

    summary_path = os.path.join(outdir,
                                os.path.splitext(os.path.basename(args.bagfile))[0] +
                                "_stats.json")
    with open(summary_path, "w") as f:
        json.dump(result, f, indent=2, sort_keys=True)
    print(json.dumps(result, indent=2, sort_keys=True))
    print("summary written to %s" % summary_path)

    # Trajectory PNG (best effort).
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(8, 8))
        for rid, od in sorted(odoms.items()):
            xs = [p[1] for p in od]
            ys = [p[2] for p in od]
            ax.plot(xs, ys, label="uav_%d" % rid, lw=1.5)
            ax.scatter([xs[0]], [ys[0]], marker="o", s=50)
            ax.scatter([xs[-1]], [ys[-1]], marker="*", s=120)
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_title("Phase-offset trajectories (%s)" %
                     os.path.basename(args.bagfile))
        ax.legend()
        ax.grid(alpha=0.3)
        png = os.path.join(outdir,
                           os.path.splitext(os.path.basename(args.bagfile))[0] +
                           "_traj.png")
        fig.savefig(png, dpi=110)
        print("trajectory plot written to %s" % png)
    except Exception as exc:  # pragma: no cover
        print("PNG skipped: %s" % exc)
    return 0


if __name__ == "__main__":
    sys.exit(main())
