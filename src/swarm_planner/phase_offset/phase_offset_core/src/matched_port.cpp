#include "phase_offset_core/matched_port.h"

#include <cmath>
#include <utility>

namespace phase_offset_core {
namespace {

bool IsFinite(const double value) {
  return std::isfinite(value);
}

bool IsFinite(const Eigen::Vector3d& value) {
  return value.allFinite();
}

void EnsureFinite(MatchedPortOutput& output) {
  if (!IsFinite(output.physical_port)) output.physical_port.setZero();
  if (!IsFinite(output.v_cmd)) output.v_cmd.setZero();
  if (!IsFinite(output.w_dot)) output.w_dot = 0.0;
  if (!IsFinite(output.delta_dot)) output.delta_dot = 0.0;
  if (!IsFinite(output.matched_residual)) output.matched_residual.setZero();
  if (!IsFinite(output.matched_residual_norm)) output.matched_residual_norm = 0.0;
}

bool Invalidate(MatchedPortOutput& output, std::string reason) {
  EnsureFinite(output);
  output.valid = false;
  output.invalid_reason = std::move(reason);
  return false;
}

}  // namespace

bool MatchedPort::evaluate(const MatchedPortInput& input,
                           MatchedPortOutput& output) {
  output = MatchedPortOutput();
  if (!input.geometry.valid) {
    return Invalidate(output, "geometry is invalid");
  }
  if (!IsFinite(input.geometry.r_w) || !IsFinite(input.geometry.N) ||
      !IsFinite(input.base_v_cmd) || !IsFinite(input.base_w_dot) ||
      !IsFinite(input.final_port.u_w) || !IsFinite(input.final_port.u_delta)) {
    return Invalidate(output, "matched-port input is not finite");
  }

  output.physical_port = input.geometry.r_w * input.final_port.u_w +
                         input.geometry.N * input.final_port.u_delta;
  output.v_cmd = input.base_v_cmd + output.physical_port;
  output.w_dot = input.base_w_dot + input.final_port.u_w;
  output.delta_dot = input.final_port.u_delta;

  const Eigen::Vector3d matched_dynamics =
      output.v_cmd - input.geometry.r_w * output.w_dot -
      input.geometry.N * output.delta_dot;
  const Eigen::Vector3d base_dynamics =
      input.base_v_cmd - input.geometry.r_w * input.base_w_dot;
  output.matched_residual = matched_dynamics - base_dynamics;
  output.matched_residual_norm = output.matched_residual.norm();
  if (!IsFinite(output.physical_port) || !IsFinite(output.v_cmd) ||
      !IsFinite(output.w_dot) || !IsFinite(output.delta_dot) ||
      !IsFinite(output.matched_residual) ||
      !IsFinite(output.matched_residual_norm)) {
    return Invalidate(output, "matched-port output is not finite");
  }

  output.valid = true;
  output.invalid_reason.clear();
  return true;
}

}  // namespace phase_offset_core
