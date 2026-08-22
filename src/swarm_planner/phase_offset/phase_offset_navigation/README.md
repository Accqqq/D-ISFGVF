# phase_offset_navigation

`phase_offset_navigation` is the ROS-free single-UAV runtime above
`phase_offset_core`.  It builds conservative phase-indexed offset envelopes,
filters them into C1 profiles, and applies the same final matched port to the
physical, phase, and offset channels.

The standalone `TubeCrossSectionSolver` is a pure C++ categorical-environment
kernel for one normal cross-section. It traces the positive and negative
directions independently, preserves asymmetric feasible intervals, and reports
geometric validity separately from whether zero or a preferred offset is
contained. Unknown, map-boundary, and unavailable cells conservatively stop
only the affected ray. The historical `RawOccupancyQuery` type name remains
for source compatibility, but it means a categorical environment query, not a
requirement to read raw SDFMap log odds.

Production ESDF construction accepts only the immutable cloud-clearance query
and its exact path evaluator.  A missing cloud query, snapshot resolution, or
path evaluator fails closed; it cannot fall back to an SDF-distance query.
`TubeBuilder::build` and `TubeBuilder::buildRawOccupancy` remain isolated
legacy unit-test APIs, not production Candidate, Active, or clearance paths.
Map ownership and ROS adaptation remain outside this package.

Fixed envelopes exercise the profile and port-invariance mechanism but are
explicitly not obstacle certified.  Cloud-clearance ESDF envelopes retain the
full/residual margin accounting and reject incomplete previews conservatively.

Existing callers that provide only `DistanceQuery` retain the legacy
inflated-ESDF builder and its six-field erosion contract for isolated tests.
If an ESDF epoch input provides the categorical query, `TubeBuilder` calls
`TubeCrossSectionSolver` instead: its environmental search never applies the
legacy controller-like `max_offset` clamp. The full physical radius is
`r_full = r_uav + e_map + e_loc + e_track`. If the immutable map backing
already includes a declared part of `e_map`, the solver erodes only
`r_residual = r_full - e_map_preincluded`; both facts are retained with every
cross-section. Samples retain obstacle, curvature, environment, termination,
zero-containment, and preferred-offset facts separately. Their legacy
signed-distance fields stay finite placeholders; they do not claim a raw
Euclidean ESDF measurement.

`TubeFilter` retains its local slope limiting and dense C1 certification but
does not replace a locally uncertifiable profile with a global envelope.  A
bounded local repair failure is reported as filter failure, so a narrow point
cannot silently flatten distant open sections.

`TubeEpochManager` is a separate, ROS-free candidate/active profile state
machine.  A candidate build sequence is not an active tube epoch: only a
material hybrid installation changes the active epoch, while equivalent
refreshes retain it.  R1 does not connect this component to the legacy Runtime
or ROS. Its `map_observation_sequence` input is supplied by the integration
layer and can identify one immutable observation snapshot without becoming a
profile-identity counter.

For the categorical cross-section path, geometric interval validity is
independent of zero or the retained offset.  A valid one-sided candidate is
therefore preserved for diagnostics, but a candidate that excludes the
retained offset cannot overwrite Active ownership.  Raw current-state
validation uses candidate bounds and cloud-clearance evidence; it has no
additional legacy `DistanceQuery` check.

R2 connects that ownership model in the adapter at a deterministic 10 Hz (or
accepted-path event) cadence.  `PhaseOffsetRuntime` is now high-rate execution
only: it consumes an immutable installed active profile, queries its current
and next bounds, projects the final port, and integrates delta.  It neither
constructs candidates nor owns a distance query, builder, filter, or rebuild
counter.  A latest candidate and an installed active profile remain distinct;
the former drives the UNCERTIFIED marker and the latter drives the certified
marker. In A5-G2d the integration supplies one immutable cloud-occupancy
snapshot per real cloud observation, so candidate construction, current
cross-section checks, and diagnostics all read the same sequence. A period
expiry may rebuild from the same sequence; it never fabricates a map revision.

A5-G2d corrects the cloud-map bridge. `obstacle_set_complete=true` is a
producer contract, not a confidence hint: it may be asserted only when the
producer can prove that every obstacle in its declared observed domain is
represented, and that every unoccupied voxel in that same domain is known
free. Inside that domain the query may certify free space; outside it is
unknown. Sparse real depth hits, points outside the sensor FOV, and volumes in
occlusion shadows cannot establish that proof and must publish
`obstacle_set_complete=false`. A sparse depth-hit cloud without this
observed-domain contract remains fail-closed. The adapter never reads SDFMap
raw log odds or applies a self-free seed. The cloud snapshot mirrors the
point-cloud map's one-voxel inflation and declares that preincluded
`e_map=0.10 m`; with the default full radius `0.55 m`, the categorical
cross-section erosion is explicitly the residual `0.45 m`.

The Runtime evaluates live `U+` and, only when necessary, live `U_safe` with
the same projector. `RuntimeExecutionMode::SAFETY_PRIORITY` is the latter
live-port result and remains distinct from epoch ownership. If neither set is
available, or current cloud evidence disproves the certificate, the result is
`CERTIFICATE_DENIED`: it withdraws the offset certificate and selected matched
port, but does not issue an emergency actuator command, introduce a mode, or
claim a replan action.

S6 removes the obsolete retained-forward and dynamic-rollout diagnostics. The
existing tube-epoch payload therefore changes from 60 fields to 50 live fields
and reports schema version `3`; this schema cleanup does not represent a
change to Runtime control evidence or a replay result.

For a categorical candidate, `TubeBuilder` retains the maximal contiguous valid run
that contains the current phase sample.  `preview_start_w`/`preview_end_w`
therefore identify that certified segment, while the separately stored
requested endpoints and truncation facts preserve the full observation
attempt.  A future or past invalid sample only truncates its side; an invalid
current sample remains incomplete.  A one-sample certified profile can be
mathematically valid, but the unchanged Marker helper deliberately DELETEs it
because it cannot form a surface ribbon.

The cloud snapshot intentionally represents only its cloud environment input;
it does not fold in manual-click or static-preinflated layers. G2d evidence
therefore uses the original pillar/local-sensing cloud with an empty manual
obstacle file and the static-preinflated layer disabled. Integrating other
environmental layers is outside this stage.
