# phase_offset_core

`phase_offset_core` is a small, pure C++14/Eigen library for evaluating a
gravity-referenced 2.5D differential path and its scalar horizontal-offset
reference. It owns no runtime state and has no side effects.

## Public input and output

`PathDifferentialState` supplies `p`, `p_w`, `p_ww`, and a caller-owned phase
value `w`. `GeometryEvaluator::evaluate` also receives a world position and
the scalar offset `delta`, then returns:

- three-dimensional unit tangent `T`, horizontal left normal `N`, and `N_w`;
- horizontal signed curvature `kappa_xy`;
- offset reference `r` and its fixed-offset derivative `r_w`;
- error `e`, its tangential scalar component, and its normal component.

The base path and its two phase derivatives remain three-dimensional. The
scalar offset `delta` is restricted to the gravity-referenced horizontal normal,
so the candidate reference keeps the base-path height. The evaluator rejects a
zero full tangent or a near-vertical path whose horizontal tangent is too small;
it does not reject a path merely because `p_w.z()` or `p_ww.z()` is nonzero.

For `e_z = (0, 0, 1)`, the evaluator uses

```text
h = e_z x p_w
N = h / ||h||
h_w = e_z x p_ww
N_w = (I - N N^T) h_w / ||h||
kappa_xy = (p_wx p_wwy - p_wy p_wwx) / ||h||^3
r = p + N delta
r_w = p_w + N_w delta
T = r_w / ||r_w||
```

The horizontal regularity check is `1 - kappa_xy * delta >= regularity_margin`.
For a fixed-height path, this strictly reduces to the previous planar formula
`r_w = (1 - kappa_xy * delta) p_w`. The returned state is always finite;
failures are represented by `valid=false` and a non-empty diagnostic string.

## A4 matched ports

`port_types.h`, `MatchedPort`, and `PortProjector` are still pure C++14/Eigen
components with no runtime-system dependencies.

`MatchedPort` accepts only a *final* port and applies that same pair to all
three channels:

```text
x_dot     = f_x + r_w u_w_final + N u_delta_final
w_dot     = f_w + u_w_final
delta_dot = u_delta_final
```

It also reports the complete matched residual formed from these full dynamics.
It neither generates nor projects a port.

`PortProjector` is a stateless local kinematic projector.  Its caller supplies
the previous final port explicitly.  A4 limits only port amplitude and rate,
strictly positive phase speed, strictly positive physical tangential speed,
and the next local regularity margin.

## A5 generic offset envelope

`offset_constraint.h` adds a deliberately generic scalar lower/upper envelope
to the projector input.  It has no map, ESDF, profile, ROS, or multi-UAV
types.  When the constraint is disabled, `PortProjector` retains its A4 scalar
clamp behavior exactly.  When enabled, it projects the two final ports jointly
onto the deterministic convex set formed by the A4 limits, the envelope at the
next step, and its upper/lower forward-invariance half-planes.  The result
reports both invariant residuals and whether the envelope constrained either
port.

This keeps the core responsible only for generic kinematics: callers own
where an envelope comes from and must pass the same projected final port to
the physical, phase, and offset channels.

## Build and test

```bash
catkin_make -j8
catkin_make run_tests_phase_offset_core
```
