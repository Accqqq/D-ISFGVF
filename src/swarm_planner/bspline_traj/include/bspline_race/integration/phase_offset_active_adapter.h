#pragma once

#include <Eigen/Core>

#include <array>
#include <string>

#include <bspline_race/continuous_phase_path.h>
#include <bspline_race/guidance/isf_reference_kernel.h>
#include <phase_offset_core/geometry.h>

namespace FLAG_Race {

struct ActiveAdapterInput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  phase_offset_core::PathDifferentialState path;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  guidance::IsfGains gains;
};

struct ZeroPortGeometryResiduals {
  double r_minus_p_norm = 0.0;
  double r_w_minus_p_w_norm = 0.0;
  double tangent_residual = 0.0;
};

struct ActiveAdapterOutput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  guidance::IsfGuidance guidance;
  phase_offset_core::PhaseOffsetGeometryState geometry;
  ZeroPortGeometryResiduals zero_port;
  bool valid = false;
  std::string invalid_reason;
};

struct LegacyGuidanceSnapshot {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d v_cmd = Eigen::Vector3d::Zero();
  double w_dot = 0.0;
  double e_parallel = 0.0;
  Eigen::Vector3d e_perp = Eigen::Vector3d::Zero();
  Eigen::Vector3d ref_pt = Eigen::Vector3d::Zero();
  Eigen::Vector3d tangent = Eigen::Vector3d::Zero();
  bool valid = false;

  LegacyGuidanceSnapshot() = default;
  LegacyGuidanceSnapshot(const Eigen::Vector3d& v_cmd_value,
                         const double w_dot_value,
                         const double e_parallel_value,
                         const Eigen::Vector3d& e_perp_value,
                         const Eigen::Vector3d& ref_pt_value,
                         const Eigen::Vector3d& tangent_value,
                         const bool valid_value)
      : v_cmd(v_cmd_value),
        w_dot(w_dot_value),
        e_parallel(e_parallel_value),
        e_perp(e_perp_value),
        ref_pt(ref_pt_value),
        tangent(tangent_value),
        valid(valid_value) {}
};

struct ActiveEquivalenceResult {
  double v_cmd_residual = 0.0;
  double w_dot_residual = 0.0;
  double e_parallel_residual = 0.0;
  double e_perp_residual = 0.0;
  double reference_residual = 0.0;
  double tangent_residual = 0.0;
  bool valid = false;
  bool equivalent = false;
  std::string invalid_reason;
};

phase_offset_core::PathDifferentialState ConvertContinuousPhasePathStateForActive(
    const ContinuousPhasePathState& source,
    double w);

ActiveAdapterInput MakeActiveAdapterInput(
    const ContinuousPhasePathState& source,
    double w,
    const Eigen::Vector3d& position,
    const guidance::IsfGains& gains);

class PhaseOffsetActiveAdapter {
 public:
  bool evaluate(const ActiveAdapterInput& input,
                ActiveAdapterOutput& output) const;

  static bool compareWithLegacy(const ActiveAdapterOutput& active,
                                const LegacyGuidanceSnapshot& legacy,
                                double tolerance,
                                ActiveEquivalenceResult& comparison);

  template <typename LegacyGuidance>
  static bool compareWithLegacyResult(const ActiveAdapterOutput& active,
                                      const LegacyGuidance& legacy,
                                      const double tolerance,
                                      ActiveEquivalenceResult& comparison) {
    return compareWithLegacy(
        active,
        LegacyGuidanceSnapshot(
            legacy.v_cmd, legacy.w_dot, legacy.e_parallel, legacy.e_perp,
            legacy.ref_pt, legacy.tangent, legacy.valid),
        tolerance, comparison);
  }

  static std::array<double, 14> makeDiagnostics(
      const ActiveAdapterOutput& active,
      const ActiveEquivalenceResult& comparison,
      bool gate_open,
      int consecutive_count,
      bool failure_latched,
      bool selected_active);

 private:
  phase_offset_core::GeometryEvaluator geometry_evaluator_;
};

}  // namespace FLAG_Race
