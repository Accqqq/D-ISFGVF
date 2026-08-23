#include "bspline_race/integration/phase_offset_active_adapter.h"

#include <cmath>
#include <utility>

namespace FLAG_Race {
namespace {

bool IsFinite(const double value) {
  return std::isfinite(value);
}

bool IsFinite(const Eigen::Vector3d& value) {
  return value.allFinite();
}

bool HasFiniteGuidance(const guidance::IsfGuidance& guidance) {
  return IsFinite(guidance.v_cmd) && IsFinite(guidance.w_dot) &&
         IsFinite(guidance.e_parallel) && IsFinite(guidance.e_perp) &&
         IsFinite(guidance.ref_pt) && IsFinite(guidance.tangent);
}

bool HasFiniteLegacy(const LegacyGuidanceSnapshot& legacy) {
  return IsFinite(legacy.v_cmd) && IsFinite(legacy.w_dot) &&
         IsFinite(legacy.e_parallel) && IsFinite(legacy.e_perp) &&
         IsFinite(legacy.ref_pt) && IsFinite(legacy.tangent);
}

void EnsureFinite(ZeroPortGeometryResiduals& residuals) {
  if (!IsFinite(residuals.r_minus_p_norm)) {
    residuals.r_minus_p_norm = 0.0;
  }
  if (!IsFinite(residuals.r_w_minus_p_w_norm)) {
    residuals.r_w_minus_p_w_norm = 0.0;
  }
  if (!IsFinite(residuals.tangent_residual)) {
    residuals.tangent_residual = 0.0;
  }
}

void EnsureFinite(ActiveEquivalenceResult& comparison) {
  if (!IsFinite(comparison.v_cmd_residual)) {
    comparison.v_cmd_residual = 0.0;
  }
  if (!IsFinite(comparison.w_dot_residual)) {
    comparison.w_dot_residual = 0.0;
  }
  if (!IsFinite(comparison.e_parallel_residual)) {
    comparison.e_parallel_residual = 0.0;
  }
  if (!IsFinite(comparison.e_perp_residual)) {
    comparison.e_perp_residual = 0.0;
  }
  if (!IsFinite(comparison.reference_residual)) {
    comparison.reference_residual = 0.0;
  }
  if (!IsFinite(comparison.tangent_residual)) {
    comparison.tangent_residual = 0.0;
  }
}

bool Invalidate(ActiveAdapterOutput& output, std::string reason) {
  EnsureFinite(output.zero_port);
  output.valid = false;
  output.invalid_reason = std::move(reason);
  return false;
}

bool Invalidate(ActiveEquivalenceResult& comparison, std::string reason) {
  EnsureFinite(comparison);
  comparison.valid = false;
  comparison.equivalent = false;
  comparison.invalid_reason = std::move(reason);
  return false;
}

}  // namespace

phase_offset_core::PathDifferentialState ConvertContinuousPhasePathStateForActive(
    const ContinuousPhasePathState& source,
    const double w) {
  phase_offset_core::PathDifferentialState converted;
  converted.p = source.p;
  converted.p_w = source.dp_dw;
  converted.p_ww = source.d2p_dw2;
  converted.w = w;
  converted.T = source.T;
  converted.N = source.N;
  converted.N_w = source.N_w;
  converted.path_revision = source.path_revision;
  converted.frame_revision = source.frame_revision;
  converted.frame_valid = source.frame_valid;
  converted.frame_provenance = source.frame_provenance;
  converted.valid = source.valid;
  return converted;
}

ActiveAdapterInput MakeActiveAdapterInput(
    const ContinuousPhasePathState& source,
    const double w,
    const Eigen::Vector3d& position,
    const guidance::IsfGains& gains) {
  ActiveAdapterInput input;
  input.path = ConvertContinuousPhasePathStateForActive(source, w);
  input.position = position;
  input.gains = gains;
  return input;
}

bool PhaseOffsetActiveAdapter::evaluate(const ActiveAdapterInput& input,
                                        ActiveAdapterOutput& output) const {
  output = ActiveAdapterOutput();

  if (!geometry_evaluator_.evaluate(input.path, input.position, 0.0,
                                    output.geometry)) {
    return Invalidate(output, output.geometry.invalid_reason.empty()
                                  ? "zero-port geometry is invalid"
                                  : output.geometry.invalid_reason);
  }

  guidance::ReferenceGeometry reference;
  reference.point = output.geometry.r;
  reference.tangent = output.geometry.T;
  reference.derivative_norm = output.geometry.r_w.norm();
  reference.valid = output.geometry.valid;
  if (!guidance::IsfReferenceKernel::evaluate(
          input.position, reference, input.gains, output.guidance)) {
    return Invalidate(output, output.guidance.invalid_reason.empty()
                                  ? "zero-port guidance is invalid"
                                  : output.guidance.invalid_reason);
  }

  const double path_speed = input.path.p_w.norm();
  if (!IsFinite(path_speed) || path_speed <= 0.0) {
    return Invalidate(output, "base path speed is invalid");
  }
  output.zero_port.r_minus_p_norm = (output.geometry.r - input.path.p).norm();
  output.zero_port.r_w_minus_p_w_norm =
      (output.geometry.r_w - input.path.p_w).norm();
  output.zero_port.tangent_residual =
      (output.geometry.T - input.path.p_w / path_speed).norm();
  if (!IsFinite(output.zero_port.r_minus_p_norm) ||
      !IsFinite(output.zero_port.r_w_minus_p_w_norm) ||
      !IsFinite(output.zero_port.tangent_residual) ||
      !HasFiniteGuidance(output.guidance)) {
    return Invalidate(output, "zero-port output is not finite");
  }

  output.valid = true;
  output.invalid_reason.clear();
  return true;
}

bool PhaseOffsetActiveAdapter::compareWithLegacy(
    const ActiveAdapterOutput& active,
    const LegacyGuidanceSnapshot& legacy,
    const double tolerance,
    ActiveEquivalenceResult& comparison) {
  comparison = ActiveEquivalenceResult();
  if (!IsFinite(tolerance) || tolerance < 0.0) {
    return Invalidate(comparison, "equivalence tolerance is invalid");
  }
  if (!active.valid || !active.guidance.valid || !legacy.valid) {
    return Invalidate(comparison, "active or legacy guidance is invalid");
  }
  if (!HasFiniteGuidance(active.guidance) || !HasFiniteLegacy(legacy)) {
    return Invalidate(comparison, "active or legacy guidance is not finite");
  }

  comparison.v_cmd_residual = (active.guidance.v_cmd - legacy.v_cmd).norm();
  comparison.w_dot_residual = std::abs(active.guidance.w_dot - legacy.w_dot);
  comparison.e_parallel_residual =
      std::abs(active.guidance.e_parallel - legacy.e_parallel);
  comparison.e_perp_residual = (active.guidance.e_perp - legacy.e_perp).norm();
  comparison.reference_residual = (active.guidance.ref_pt - legacy.ref_pt).norm();
  comparison.tangent_residual = (active.guidance.tangent - legacy.tangent).norm();
  if (!IsFinite(comparison.v_cmd_residual) ||
      !IsFinite(comparison.w_dot_residual) ||
      !IsFinite(comparison.e_parallel_residual) ||
      !IsFinite(comparison.e_perp_residual) ||
      !IsFinite(comparison.reference_residual) ||
      !IsFinite(comparison.tangent_residual)) {
    return Invalidate(comparison, "equivalence residual is not finite");
  }

  comparison.valid = true;
  comparison.equivalent = comparison.v_cmd_residual <= tolerance &&
      comparison.w_dot_residual <= tolerance &&
      comparison.e_parallel_residual <= tolerance &&
      comparison.e_perp_residual <= tolerance &&
      comparison.reference_residual <= tolerance &&
      comparison.tangent_residual <= tolerance;
  if (!comparison.equivalent) {
    comparison.invalid_reason = "zero-port equivalence tolerance is exceeded";
  }
  return comparison.equivalent;
}

std::array<double, 14> PhaseOffsetActiveAdapter::makeDiagnostics(
    const ActiveAdapterOutput& active,
    const ActiveEquivalenceResult& comparison,
    const bool gate_open,
    const int consecutive_count,
    const bool failure_latched,
    const bool selected_active) {
  // Field order: mode_active, gate_open, gate_consecutive_count,
  // active_failure_latched, v_cmd_residual, w_dot_residual,
  // e_parallel_residual, e_perp_residual, reference_residual,
  // tangent_residual, r_minus_p_norm, r_w_minus_p_w_norm,
  // geometry_tangent_residual, selected_source.
  std::array<double, 14> values = {{
      1.0,
      gate_open ? 1.0 : 0.0,
      static_cast<double>(consecutive_count),
      failure_latched ? 1.0 : 0.0,
      comparison.v_cmd_residual,
      comparison.w_dot_residual,
      comparison.e_parallel_residual,
      comparison.e_perp_residual,
      comparison.reference_residual,
      comparison.tangent_residual,
      active.zero_port.r_minus_p_norm,
      active.zero_port.r_w_minus_p_w_norm,
      active.zero_port.tangent_residual,
      selected_active ? 1.0 : 0.0}};
  for (double& value : values) {
    if (!IsFinite(value)) {
      value = 0.0;
    }
  }
  return values;
}

}  // namespace FLAG_Race
