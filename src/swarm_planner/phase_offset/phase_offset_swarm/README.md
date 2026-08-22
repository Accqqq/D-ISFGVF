# phase_offset_swarm

This package currently exposes the isolated, ROS-free PhaseOffsetSwarm
Swarm Intent V1 core.  It consumes timestamped two-dimensional neighbour
states and a local `beta_preview`, then computes the bounded world-frame
nominal intent

`g_coord = sat(g_sep + beta_preview * g_coh0 + g_conf)`.

The core has no ROS, map, Tube, allocator, CBF, matched-port, or vehicle
control dependency.  A ROS1 shadow adapter is added only after the pure C++
test gate passes.
