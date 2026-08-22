#pragma once

#include <cmath>
#include <stdexcept>
#include <string>

namespace phase_offset_swarm {

struct SwarmParameters {
  // Distances are metres, gains/intent limits are metres per second, and
  // time values are seconds unless the field name says otherwise.
  double d_safe = 0.60;
  double d_minus = 0.80;
  double d_plus = 1.20;
  double r_comm = 2.50;
  double r_conf = 1.80;
  double r_safe = 1.50;
  double k_sep = 1.20;
  double k_coh = 0.25;
  double k_conf = 0.80;
  double ttc_activation = 2.00;
  double closing_speed_activation = 0.50;
  double fresh_timeout = 0.30;
  double stale_timeout = 0.60;
  double lost_retention_timeout = 1.50;
  double g_max = 1.50;
  double distance_epsilon = 1e-9;
  double ttc_epsilon = 1e-6;
  double neighbor_hysteresis = 0.05;
  double future_stamp_tolerance = 0.02;

  bool validate(std::string* error = nullptr) const {
    const auto fail = [error](const std::string& message) {
      if (error != nullptr) {
        *error = message;
      }
      return false;
    };
    const auto finite = [](double value) { return std::isfinite(value); };
    const auto nonnegative = [&finite](double value) {
      return finite(value) && value >= 0.0;
    };

    if (!finite(d_safe) || !finite(d_minus) || !finite(d_plus) ||
        !finite(r_comm) || !(0.0 < d_safe && d_safe < d_minus &&
                             d_minus < d_plus && d_plus < r_comm)) {
      return fail("require 0 < d_safe < d_minus < d_plus < r_comm");
    }
    if (!finite(r_conf) || !finite(r_safe) || !(d_safe < r_conf) ||
        !(d_safe < r_safe)) {
      return fail("require d_safe < r_conf and d_safe < r_safe");
    }
    if (!finite(fresh_timeout) || !finite(stale_timeout) ||
        !finite(lost_retention_timeout) ||
        !(0.0 < fresh_timeout && fresh_timeout < stale_timeout &&
          stale_timeout < lost_retention_timeout)) {
      return fail("require 0 < fresh_timeout < stale_timeout < retention");
    }
    if (!nonnegative(r_comm) || !nonnegative(r_conf) || !nonnegative(r_safe) ||
        !nonnegative(k_sep) || !nonnegative(k_coh) || !nonnegative(k_conf) ||
        !nonnegative(ttc_activation) ||
        !nonnegative(closing_speed_activation) ||
        !nonnegative(neighbor_hysteresis) ||
        !nonnegative(future_stamp_tolerance)) {
      return fail("radii, gains, activations, and hysteresis must be finite");
    }
    if (!finite(g_max) || !(g_max > 0.0)) {
      return fail("g_max must be strictly positive and finite");
    }
    if (!finite(distance_epsilon) || !(distance_epsilon > 0.0) ||
        !finite(ttc_epsilon) || !(ttc_epsilon > 0.0) ||
        !finite(closing_speed_activation) ||
        !(closing_speed_activation > 0.0) ||
        !finite(ttc_activation) || !(ttc_activation > 0.0)) {
      return fail("distance, TTC, and closing-speed divisors must be positive");
    }
    if (!finite(r_comm) || !finite(r_conf) || !finite(r_safe) ||
        !finite(k_sep) || !finite(k_coh) || !finite(k_conf) ||
        !finite(fresh_timeout) || !finite(stale_timeout) ||
        !finite(lost_retention_timeout)) {
      return fail("all parameters must be finite");
    }
    return true;
  }

  void validateOrThrow() const {
    std::string error;
    if (!validate(&error)) {
      throw std::invalid_argument("invalid SwarmParameters: " + error);
    }
  }
};

}  // namespace phase_offset_swarm
