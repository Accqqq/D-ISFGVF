#include "phase_offset_core/geometry.h"
#include "phase_offset_core/normal_frame.h"

#include <Eigen/Geometry>

#include <cmath>

namespace phase_offset_core {
namespace {

const char kNoReason[] = "";
const char kInvalidGeometryParameters[] = "invalid geometry parameters";
const char kPathStateInvalid[] = "path state is invalid";
const char kPathInputNotFinite[] = "path input is not finite";
const char kPositionNotFinite[] = "position is not finite";
const char kOffsetNotFinite[] = "offset is not finite";
const char kPathSpeedNotFinite[] = "path speed is not finite";
const char kPathSpeedTooSmall[] = "path speed is too small";
const char kHorizontalPathSpeedNotFinite[] = "horizontal path speed is not finite";
const char kHorizontalPathSpeedTooSmall[] = "horizontal path speed is too small";
const char kCurvatureOrRegularityNotFinite[] =
    "curvature or regularity is not finite";
const char kOffsetRegularityMarginViolated[] =
    "offset regularity margin is violated";
const char kActiveReferenceSpeedNotFinite[] =
    "active reference speed is not finite";
const char kActiveReferenceSpeedTooSmall[] =
    "active reference speed is too small";
const char kComputedGeometryNotFinite[] = "computed geometry is not finite";

struct TerminalGeometry {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d T = Eigen::Vector3d::Zero();
  Eigen::Vector3d r = Eigen::Vector3d::Zero();
  Eigen::Vector3d r_w = Eigen::Vector3d::Zero();
  Eigen::Vector3d error = Eigen::Vector3d::Zero();
  Eigen::Vector3d e_perp = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 3, 2> J = Eigen::Matrix<double, 3, 2>::Zero();
  double regularity = 0.0;
  double e_parallel = 0.0;
};

bool IsFinite(const double value) {
  return std::isfinite(value);
}

bool IsFinite(const Eigen::Vector3d& value) {
  return value.allFinite();
}

void ResetIfNotFinite(Eigen::Vector3d& value) {
  if (!IsFinite(value)) {
    value.setZero();
  }
}

void ResetIfNotFinite(double& value) {
  if (!IsFinite(value)) {
    value = 0.0;
  }
}

void EnsureFiniteOutput(PhaseOffsetGeometryState& output) {
  ResetIfNotFinite(output.p);
  ResetIfNotFinite(output.p_w);
  ResetIfNotFinite(output.p_ww);
  ResetIfNotFinite(output.T);
  ResetIfNotFinite(output.N);
  ResetIfNotFinite(output.N_w);
  ResetIfNotFinite(output.r);
  ResetIfNotFinite(output.r_w);
  ResetIfNotFinite(output.error);
  ResetIfNotFinite(output.e_perp);
  ResetIfNotFinite(output.w);
  ResetIfNotFinite(output.delta);
  ResetIfNotFinite(output.path_speed);
  ResetIfNotFinite(output.horizontal_path_speed);
  ResetIfNotFinite(output.curvature);
  ResetIfNotFinite(output.regularity);
  ResetIfNotFinite(output.e_parallel);
  if (!output.J.allFinite()) output.J.setZero();
}

bool Invalidate(PhaseOffsetGeometryState& output, const char* reason) {
  EnsureFiniteOutput(output);
  output.valid = false;
  output.invalid_reason = reason;
  return false;
}

bool Invalidate(PreparedReferenceResult& output, const char* reason) {
  ResetIfNotFinite(output.r);
  ResetIfNotFinite(output.regularity);
  output.valid = false;
  output.invalid_reason = reason;
  return false;
}

void InitializeFiniteOutput(const PathDifferentialState& path,
                            const double delta,
                            PhaseOffsetGeometryState& output) {
  output = PhaseOffsetGeometryState();
  if (IsFinite(path.p)) {
    output.p = path.p;
  }
  if (IsFinite(path.p_w)) {
    output.p_w = path.p_w;
  }
  if (IsFinite(path.p_ww)) {
    output.p_ww = path.p_ww;
  }
  if (IsFinite(path.w)) {
    output.w = path.w;
  }
  if (IsFinite(delta)) {
    output.delta = delta;
  }
}

bool HasValidParameters(const GeometryParams& params) {
  return IsFinite(params.tangent_epsilon) && params.tangent_epsilon > 0.0 &&
         IsFinite(params.horizontal_tangent_epsilon) &&
         params.horizontal_tangent_epsilon > 0.0 &&
         IsFinite(params.regularity_margin) && params.regularity_margin > 0.0 &&
         IsFinite(params.minimum_reference_speed) &&
         params.minimum_reference_speed > 0.0;
}

const char* PrePointFailure(const PathDifferentialState& path,
                            const GeometryParams& params) {
  if (!HasValidParameters(params)) {
    return kInvalidGeometryParameters;
  }
  if (!path.valid) {
    return kPathStateInvalid;
  }
  if (!IsFinite(path.p) || !IsFinite(path.p_w) || !IsFinite(path.p_ww) ||
      !IsFinite(path.w)) {
    return kPathInputNotFinite;
  }
  return nullptr;
}

void BuildPreparedPathFromValidatedPath(const PathDifferentialState& path,
                                        const GeometryParams& params,
                                        PreparedPathGeometry& output) {
  // All callers hand us a freshly value-initialized PreparedPathGeometry.
  // Do not zero its fixed-size Eigen members a second time on the full path.
  output.p = path.p;
  output.p_w = path.p_w;
  output.p_ww = path.p_ww;
  output.w = path.w;
  output.path_revision = path.path_revision;
  output.frame_revision = path.frame_revision;
  output.frame_bound = path.frame_valid;
  output.frame_provenance = path.frame_provenance;
  output.pre_point_valid = true;
  output.pre_point_reason = kNoReason;
  output.delayed_path_reason = kNoReason;

  output.path_speed = path.p_w.norm();
  if (!IsFinite(output.path_speed)) {
    output.delayed_path_reason = kPathSpeedNotFinite;
    return;
  }
  if (output.path_speed <= params.tangent_epsilon) {
    output.delayed_path_reason = kPathSpeedTooSmall;
    return;
  }

  const Eigen::Vector3d gravity_axis(0.0, 0.0, 1.0);
  const Eigen::Vector3d horizontal_tangent = gravity_axis.cross(path.p_w);
  output.horizontal_path_speed = horizontal_tangent.norm();
  if (!IsFinite(output.horizontal_path_speed)) {
    output.delayed_path_reason = kHorizontalPathSpeedNotFinite;
    return;
  }
  output.T = path.p_w / output.path_speed;
  if (path.frame_valid && IsFinite(path.T) && IsFinite(path.N) &&
      IsFinite(path.N_w) && path.T.norm() > params.tangent_epsilon &&
      path.N.norm() > params.tangent_epsilon) {
    output.T = path.T.normalized();
    output.N = path.N.normalized();
    output.N_w = path.N_w;
    output.frame_bound = true;
  } else {
    // Compatibility fallback for synthetic callers. Production adapters pass
    // frame_valid=true, so no production consumer reconstructs N/N_w.
    if (!IsFinite(output.horizontal_path_speed) ||
        output.horizontal_path_speed <= params.horizontal_tangent_epsilon) {
      output.delayed_path_reason = kHorizontalPathSpeedTooSmall;
      return;
    }
    output.N = horizontal_tangent / output.horizontal_path_speed;
    const Eigen::Vector3d horizontal_tangent_derivative =
        gravity_axis.cross(path.p_ww);
    const Eigen::Vector3d projected_horizontal_tangent_derivative =
        horizontal_tangent_derivative -
        output.N * output.N.dot(horizontal_tangent_derivative);
    output.N_w = projected_horizontal_tangent_derivative /
        output.horizontal_path_speed;
  }
  if (!IsFinite(output.T) || !IsFinite(output.N) || !IsFinite(output.N_w) ||
      std::abs(output.T.norm() - 1.0) > 1e-6 ||
      std::abs(output.N.norm() - 1.0) > 1e-6 ||
      std::abs(output.T.dot(output.N)) > 1e-6) {
    output.delayed_path_reason = kComputedGeometryNotFinite;
    return;
  }
  const double curvature_numerator =
      path.p_w.x() * path.p_ww.y() - path.p_w.y() * path.p_ww.x();
  const double speed_cubed =
      output.horizontal_path_speed * output.horizontal_path_speed *
      output.horizontal_path_speed;
  output.curvature = output.horizontal_path_speed >
          params.horizontal_tangent_epsilon
      ? curvature_numerator / speed_cubed : 0.0;
  output.delayed_path_valid = true;
}

void CopyPreparedPrefix(const PreparedPathGeometry& prepared,
                        const double delta,
                        PhaseOffsetGeometryState& output) {
  output.p = prepared.p;
  output.p_w = prepared.p_w;
  output.p_ww = prepared.p_ww;
  output.w = prepared.w;
  output.delta = delta;
  output.path_speed = prepared.path_speed;
  output.horizontal_path_speed = prepared.horizontal_path_speed;
  output.N = prepared.N;
  output.N_w = prepared.N_w;
  output.T = prepared.T;
  output.curvature = prepared.curvature;
  output.path_revision = prepared.path_revision;
  output.frame_revision = prepared.frame_revision;
  output.provenance = prepared.frame_provenance;
}

template <typename TerminalOutput>
bool HasFiniteTerminal(const PreparedPathGeometry& prepared,
                       const double delta,
                       const TerminalOutput& output) {
  return IsFinite(prepared.p) && IsFinite(prepared.p_w) &&
         IsFinite(prepared.p_ww) && IsFinite(output.T) &&
         IsFinite(prepared.N) && IsFinite(prepared.N_w) && IsFinite(output.r) &&
         IsFinite(output.r_w) && IsFinite(output.error) &&
         IsFinite(output.e_perp) && IsFinite(prepared.w) && IsFinite(delta) &&
         IsFinite(prepared.path_speed) &&
         IsFinite(prepared.horizontal_path_speed) && IsFinite(prepared.curvature) &&
         IsFinite(output.regularity) && IsFinite(output.e_parallel);
}

template <typename TerminalOutput>
const char* EvaluateTerminal(const PreparedPathGeometry& prepared,
                             const Eigen::Vector3d& position,
                             const double delta,
                             const GeometryParams& params,
                             TerminalOutput& output) {
  // All callers hand us a freshly value-initialized TerminalGeometry.
  // Preserving that initial zero state retains failure diagnostics while
  // avoiding a second fixed-size zero fill for every evaluated delta.
  output.r = prepared.p + prepared.N * delta;
  output.r_w = prepared.p_w + prepared.N_w * delta;
  const double legacy_regularity = 1.0 - prepared.curvature * delta;
  output.regularity = prepared.frame_bound ? output.r_w.norm()
                                           : legacy_regularity;
  if (!IsFinite(prepared.curvature) || !IsFinite(output.regularity)) {
    return kCurvatureOrRegularityNotFinite;
  }
  if ((!prepared.frame_bound &&
       output.regularity < params.regularity_margin) ||
      (prepared.frame_bound &&
       output.regularity < params.minimum_reference_speed)) {
    if (!prepared.frame_bound) return kOffsetRegularityMarginViolated;
    return kActiveReferenceSpeedTooSmall;
  }
  const double active_path_speed = output.r_w.norm();
  if (!IsFinite(active_path_speed)) {
    return kActiveReferenceSpeedNotFinite;
  }
  if (active_path_speed <= params.tangent_epsilon) {
    return kActiveReferenceSpeedTooSmall;
  }
  output.T = output.r_w / active_path_speed;
  output.error = position - output.r;
  output.e_parallel = output.T.dot(output.error);
  output.e_perp = output.error - output.e_parallel * output.T;
  output.J.col(0) = output.r_w;
  output.J.col(1) = prepared.N;

  if (!HasFiniteTerminal(prepared, delta, output)) {
    return kComputedGeometryNotFinite;
  }
  return nullptr;
}

}  // namespace

GeometryEvaluator::GeometryEvaluator(const GeometryParams& params) : params_(params) {}

void GeometryEvaluator::preparePath(const PathDifferentialState& path,
                                    PreparedPathGeometry& output) const {
  output = PreparedPathGeometry();
  const char* const pre_point_failure = PrePointFailure(path, params_);
  if (pre_point_failure != nullptr) {
    output.pre_point_reason = pre_point_failure;
    return;
  }
  BuildPreparedPathFromValidatedPath(path, params_, output);
}

bool GeometryEvaluator::evaluate(const PathDifferentialState& path,
                                 const Eigen::Vector3d& position,
                                 const double delta,
                                 PhaseOffsetGeometryState& output) const {
  InitializeFiniteOutput(path, delta, output);

  const char* const pre_point_failure = PrePointFailure(path, params_);
  if (pre_point_failure != nullptr) {
    return Invalidate(output, pre_point_failure);
  }
  if (!IsFinite(position)) {
    return Invalidate(output, kPositionNotFinite);
  }
  if (!IsFinite(delta)) {
    return Invalidate(output, kOffsetNotFinite);
  }

  PreparedPathGeometry prepared;
  BuildPreparedPathFromValidatedPath(path, params_, prepared);
  CopyPreparedPrefix(prepared, delta, output);
  if (!prepared.delayed_path_valid) {
    return Invalidate(output, prepared.delayed_path_reason);
  }

  const char* const terminal_failure =
      EvaluateTerminal(prepared, position, delta, params_, output);
  if (terminal_failure != nullptr) {
    return Invalidate(output, terminal_failure);
  }

  output.valid = true;
  output.invalid_reason.clear();
  return true;
}

bool GeometryEvaluator::evaluatePreparedReference(
    const PreparedPathGeometry& prepared,
    const Eigen::Vector3d& position,
    const double delta,
    PreparedReferenceResult& output) const {
  output = PreparedReferenceResult();
  if (!prepared.pre_point_valid) {
    return Invalidate(output, prepared.pre_point_reason);
  }
  if (!IsFinite(position)) {
    return Invalidate(output, kPositionNotFinite);
  }
  if (!IsFinite(delta)) {
    return Invalidate(output, kOffsetNotFinite);
  }
  if (!prepared.delayed_path_valid) {
    return Invalidate(output, prepared.delayed_path_reason);
  }

  TerminalGeometry terminal;
  const char* const terminal_failure =
      EvaluateTerminal(prepared, position, delta, params_, terminal);
  output.r = terminal.r;
  output.regularity = terminal.regularity;
  if (terminal_failure != nullptr) {
    return Invalidate(output, terminal_failure);
  }

  output.valid = true;
  output.invalid_reason = kNoReason;
  return true;
}

}  // namespace phase_offset_core
