# Phase update ablation experiment design

## Goal

Add a self-contained ablation experiment for the existing obstacle-free GVF
simulation. The experiment compares two closed-reference phase update modes on
the same nominal figure-eight trajectory:

- `phase_continuation`: retain the current phase-continuation workflow. The
  phase is propagated from the previous update and only uses the local
  projection window already present in the controller.
- `phase_reinit`: re-estimate the closest phase from the current position on
  the complete closed reference at every update. The search is global over one
  lap, so a position near the figure-eight crossing may select the other
  branch.

The existing launches and their behavior must remain unchanged.

## Experiment entry point

Create a new launch file, `launch/phase_ablation.launch`. It shall expose:

- `mode`: `phase_continuation` or `phase_reinit`;
- the figure-eight geometry and initial-state arguments used by the current
  obstacle-free simulation;
- `record_bag`, `bag_path`, and `start_rviz` switches;
- a bounded run timeout used as a safety guard.

The launch reuses the current simulator, controller, planner, and empty-map
configuration but does not edit or include the bag-5977 reproduction launch.
The only experimental variable is the phase mode.

## Controller changes

Add a private parameter read by `gvf_manager` for the phase mode. The default
must be `phase_continuation`, preserving current behavior when the parameter is
absent.

Keep the current continuation path unchanged. For `phase_reinit`, after the
closed reference has been initialized, compute the phase candidate by searching
all reference segments over `[0, total_w)` (and selecting the nearest finite
projection). Do not use `closed_ref_w_` as the center of that search. Store the
selected phase in the same `closed_ref_w_` state so downstream lookahead,
planning, and visualization use the selected branch consistently.

Publish a `common_msgs/GvfDebug` message on `/particle0/gvf/debug` at the
controller update rate. The message must contain finite values for the ROS
timestamp, actual position, command position, `progress_w`, `closed_ref_w`,
`w_proj`, `w_dot`, guidance velocity, and raw/filtered odometry velocity.
The existing message definition may be extended only when needed; the default
launch behavior must not depend on the publisher.

## Real-log runner and post-processing

Add a Python runner under `scripts/` that:

1. launches one mode with the new launch file and records a rosbag containing
   `/sim/odom`, `/particle0/gvf/debug`, and `/particle0/circle_reference`;
2. detects completion from the logged, unwrapped closed-reference phase: after
   initialization, the phase increase must reach one `total_w` lap;
3. stops the mode cleanly, then repeats the exact procedure for the other mode;
4. converts each bag into a CSV and generates the final plot only after all
   required topics and a complete lap are present.

The runner must also enforce a minimum flight time and a configurable timeout.
If a topic is missing, a bag is empty, values are non-finite, or a lap is not
completed before timeout, it shall report failure and retain the raw bag. It
must not synthesize samples or curves.

## CSV and figure contract

For each mode, write a CSV with one row per matched debug/odom sample. Rows
must include at least:

`mode`, `ros_time`, `x`, `y`, `z`, `cmd_x`, `cmd_y`, `cmd_z`, `w`,
`w_proj`, `w_dot`, `ref_x`, `ref_y`, `ref_z`, `error_xy`, `error_3d`.

The reference point is evaluated from the nominal figure-eight geometry at the
logged phase. The local reference segment is the finite arc-length window
around that same logged phase and is exported/used for plotting; it is not a
synthetic trajectory unrelated to the logged phase.

Generate one three-panel PNG and PDF:

1. XY plot of the fixed nominal figure-eight, both actual trajectories, and
   sampled local reference segments from each mode;
2. `w(t)` for both modes, using the logged unwrapped phase and marking phase
   discontinuities;
3. local reference tracking error versus time for both modes.

Write a metadata/summary CSV containing bag paths, sample counts, lap status,
phase jump count, and error summary for each mode.

## Verification

Add focused tests for:

- global closest-point selection on both branches of the figure-eight
  crossing;
- continuation mode remaining local to the previous phase;
- bag-to-CSV reference evaluation and error calculations;
- rejection of incomplete/invalid logs.

Run an actual no-RViz simulation smoke test for both modes, verify that both
bags contain the required topics and complete one lap, and inspect the
generated CSV/PNG/PDF. Existing tests and existing launch files must continue
to pass unchanged.

