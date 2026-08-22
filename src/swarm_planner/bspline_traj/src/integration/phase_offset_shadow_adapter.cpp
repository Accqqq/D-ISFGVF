#include "bspline_race/integration/phase_offset_shadow_adapter.h"

#include <algorithm>
#include <cmath>

namespace FLAG_Race {
namespace {

constexpr double kFrameVectorLength = 0.8;

bool IsFinite(const double value) {
  return std::isfinite(value);
}

bool IsFinite(const Eigen::Vector3d& value) {
  return value.allFinite();
}

double FiniteOrZero(const double value) {
  return IsFinite(value) ? value : 0.0;
}

struct PathDiagnosticsScanState {
  bool has_path_z = false;
  bool has_p_w_z = false;
  bool has_p_ww_z = false;
  bool has_horizontal_path_speed = false;
  bool has_horizontal_to_total_speed_ratio = false;
};

void AccumulateRawPathDiagnostics(
    const phase_offset_core::PathDifferentialState& sample,
    ShadowDiagnostics& diagnostics,
    PathDiagnosticsScanState& scan_state) {
  ++diagnostics.sample_count;

  if (IsFinite(sample.p.z())) {
    if (!scan_state.has_path_z) {
      diagnostics.path_z_min = sample.p.z();
      diagnostics.path_z_max = sample.p.z();
      scan_state.has_path_z = true;
    } else {
      diagnostics.path_z_min = std::min(diagnostics.path_z_min, sample.p.z());
      diagnostics.path_z_max = std::max(diagnostics.path_z_max, sample.p.z());
    }
    diagnostics.path_z_span = diagnostics.path_z_max - diagnostics.path_z_min;
  }

  if (IsFinite(sample.p_w.z())) {
    const double abs_p_w_z = std::abs(sample.p_w.z());
    if (!scan_state.has_p_w_z || abs_p_w_z > diagnostics.max_abs_p_w_z) {
      diagnostics.max_abs_p_w_z = abs_p_w_z;
      diagnostics.max_abs_p_w_z_at_w = FiniteOrZero(sample.w);
      scan_state.has_p_w_z = true;
    }
  }

  if (IsFinite(sample.p_ww.z())) {
    const double abs_p_ww_z = std::abs(sample.p_ww.z());
    if (!scan_state.has_p_ww_z || abs_p_ww_z > diagnostics.max_abs_p_ww_z) {
      diagnostics.max_abs_p_ww_z = abs_p_ww_z;
      diagnostics.max_abs_p_ww_z_at_w = FiniteOrZero(sample.w);
      scan_state.has_p_ww_z = true;
    }
  }

  const double horizontal_path_speed =
      std::hypot(sample.p_w.x(), sample.p_w.y());
  const double total_path_speed = sample.p_w.norm();
  if (!IsFinite(horizontal_path_speed) || !IsFinite(total_path_speed)) {
    return;
  }
  if (!scan_state.has_horizontal_path_speed ||
      horizontal_path_speed < diagnostics.min_horizontal_path_speed) {
    diagnostics.min_horizontal_path_speed = horizontal_path_speed;
    diagnostics.min_horizontal_path_speed_at_w = FiniteOrZero(sample.w);
    scan_state.has_horizontal_path_speed = true;
  }

  const double horizontal_to_total_speed_ratio =
      total_path_speed > 0.0 ? horizontal_path_speed / total_path_speed : 0.0;
  if (!scan_state.has_horizontal_to_total_speed_ratio ||
      horizontal_to_total_speed_ratio <
          diagnostics.min_horizontal_to_total_speed_ratio) {
    diagnostics.min_horizontal_to_total_speed_ratio =
        horizontal_to_total_speed_ratio;
    diagnostics.min_horizontal_to_total_speed_ratio_at_w =
        FiniteOrZero(sample.w);
    scan_state.has_horizontal_to_total_speed_ratio = true;
  }
}

geometry_msgs::Point ToPoint(const Eigen::Vector3d& value) {
  geometry_msgs::Point point;
  point.x = value.x();
  point.y = value.y();
  point.z = value.z();
  return point;
}

visualization_msgs::Marker MakeLineMarker(const ros::Time& stamp,
                                          const std::string& frame_id,
                                          const std::string& name,
                                          const int id,
                                          const float red,
                                          const float green,
                                          const float blue) {
  visualization_msgs::Marker marker;
  marker.header.stamp = stamp;
  marker.header.frame_id = frame_id;
  marker.ns = name;
  marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.04;
  marker.color.r = red;
  marker.color.g = green;
  marker.color.b = blue;
  marker.color.a = 1.0;
  return marker;
}

visualization_msgs::Marker MakeDeleteMarker(const ros::Time& stamp,
                                            const std::string& frame_id,
                                            const std::string& name,
                                            const int id) {
  visualization_msgs::Marker marker;
  marker.header.stamp = stamp;
  marker.header.frame_id = frame_id;
  marker.ns = name;
  marker.id = id;
  marker.action = visualization_msgs::Marker::DELETE;
  return marker;
}

void AppendFrameDeletes(const ros::Time& stamp,
                        const std::string& frame_id,
                        visualization_msgs::MarkerArray& frame_markers) {
  for (int id = 0; id < 4; ++id) {
    frame_markers.markers.push_back(MakeDeleteMarker(
        stamp, frame_id, "phase_offset_shadow_frame", id));
  }
}

visualization_msgs::Marker MakeSphereMarker(const ros::Time& stamp,
                                            const std::string& frame_id,
                                            const int id,
                                            const Eigen::Vector3d& point,
                                            const float red,
                                            const float green,
                                            const float blue) {
  visualization_msgs::Marker marker;
  marker.header.stamp = stamp;
  marker.header.frame_id = frame_id;
  marker.ns = "phase_offset_shadow_frame";
  marker.id = id;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position = ToPoint(point);
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.16;
  marker.scale.y = 0.16;
  marker.scale.z = 0.16;
  marker.color.r = red;
  marker.color.g = green;
  marker.color.b = blue;
  marker.color.a = 1.0;
  return marker;
}

visualization_msgs::Marker MakeArrowMarker(const ros::Time& stamp,
                                           const std::string& frame_id,
                                           const int id,
                                           const Eigen::Vector3d& start,
                                           const Eigen::Vector3d& direction,
                                           const float red,
                                           const float green,
                                           const float blue) {
  visualization_msgs::Marker marker;
  marker.header.stamp = stamp;
  marker.header.frame_id = frame_id;
  marker.ns = "phase_offset_shadow_frame";
  marker.id = id;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.04;
  marker.scale.y = 0.08;
  marker.scale.z = 0.08;
  marker.color.r = red;
  marker.color.g = green;
  marker.color.b = blue;
  marker.color.a = 1.0;
  marker.points.push_back(ToPoint(start));
  marker.points.push_back(ToPoint(start + kFrameVectorLength * direction));
  return marker;
}

std_msgs::Float64MultiArray MakeDiagnosticsMessage(
    const ShadowDiagnostics& diagnostics) {
  std_msgs::Float64MultiArray message;
  // data layout: w, delta, curvature, regularity, e_parallel, e_perp_norm,
  // valid, sample_count, invalid_sample_count, path_z_min, path_z_max,
  // path_z_span, max_abs_p_w_z, max_abs_p_w_z_at_w, max_abs_p_ww_z,
  // max_abs_p_ww_z_at_w, candidate_path_complete,
  // min_horizontal_path_speed, min_horizontal_path_speed_at_w,
  // min_horizontal_to_total_speed_ratio,
  // min_horizontal_to_total_speed_ratio_at_w, max_abs_candidate_z_offset.
  message.data = {FiniteOrZero(diagnostics.w),
                  FiniteOrZero(diagnostics.delta),
                  FiniteOrZero(diagnostics.curvature),
                  FiniteOrZero(diagnostics.regularity),
                  FiniteOrZero(diagnostics.e_parallel),
                  FiniteOrZero(diagnostics.e_perp_norm),
                  diagnostics.valid ? 1.0 : 0.0};
  message.data.push_back(static_cast<double>(diagnostics.sample_count));
  message.data.push_back(static_cast<double>(diagnostics.invalid_sample_count));
  message.data.push_back(FiniteOrZero(diagnostics.path_z_min));
  message.data.push_back(FiniteOrZero(diagnostics.path_z_max));
  message.data.push_back(FiniteOrZero(diagnostics.path_z_span));
  message.data.push_back(FiniteOrZero(diagnostics.max_abs_p_w_z));
  message.data.push_back(FiniteOrZero(diagnostics.max_abs_p_w_z_at_w));
  message.data.push_back(FiniteOrZero(diagnostics.max_abs_p_ww_z));
  message.data.push_back(FiniteOrZero(diagnostics.max_abs_p_ww_z_at_w));
  message.data.push_back(diagnostics.candidate_path_complete ? 1.0 : 0.0);
  message.data.push_back(FiniteOrZero(diagnostics.min_horizontal_path_speed));
  message.data.push_back(
      FiniteOrZero(diagnostics.min_horizontal_path_speed_at_w));
  message.data.push_back(
      FiniteOrZero(diagnostics.min_horizontal_to_total_speed_ratio));
  message.data.push_back(FiniteOrZero(
      diagnostics.min_horizontal_to_total_speed_ratio_at_w));
  message.data.push_back(FiniteOrZero(diagnostics.max_abs_candidate_z_offset));
  return message;
}

}  // namespace

phase_offset_core::PathDifferentialState ConvertContinuousPhasePathState(
    const ContinuousPhasePathState& source,
    const double w) {
  phase_offset_core::PathDifferentialState converted;
  converted.p = source.p;
  converted.p_w = source.dp_dw;
  converted.p_ww = source.d2p_dw2;
  converted.w = w;
  converted.valid = source.valid;
  return converted;
}

PhaseOffsetShadowAdapter::PhaseOffsetShadowAdapter(const ShadowConfig& config)
    : config_(config), geometry_evaluator_() {
  if (!IsFinite(config_.delta)) {
    config_.delta = 0.0;
  }
  if (!IsFinite(config_.sample_step_w) || config_.sample_step_w <= 0.0) {
    config_.sample_step_w = 0.10;
  }
  if (!IsFinite(config_.publish_rate) || config_.publish_rate <= 0.0) {
    config_.publish_rate = 5.0;
  }
  if (config_.frame_id.empty()) {
    config_.frame_id = "world";
  }
}

void PhaseOffsetShadowAdapter::advertise(ros::NodeHandle& private_nh) {
  if (!enabled() || advertised_) {
    return;
  }
  base_path_pub_ = private_nh.advertise<visualization_msgs::Marker>(
      "phase_offset_shadow/base_path", 1, true);
  candidate_path_pub_ = private_nh.advertise<visualization_msgs::Marker>(
      "phase_offset_shadow/candidate_path", 1, true);
  frame_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>(
      "phase_offset_shadow/frame", 1);
  diagnostics_pub_ = private_nh.advertise<std_msgs::Float64MultiArray>(
      "phase_offset_shadow/diagnostics", 1);
  advertised_ = true;
}

bool PhaseOffsetShadowAdapter::enabled() const {
  return config_.mode == PhaseOffsetShadowMode::SHADOW;
}

double PhaseOffsetShadowAdapter::sampleStepW() const {
  return config_.sample_step_w;
}

bool PhaseOffsetShadowAdapter::isPublishDue(const ros::Time& stamp) const {
  if (!enabled()) {
    return false;
  }
  if (!has_published_) {
    return true;
  }
  return (stamp - last_publish_time_).toSec() >= 1.0 / config_.publish_rate;
}

bool PhaseOffsetShadowAdapter::buildMarkers(const ShadowUpdateInput& input,
                                            ShadowMarkerBundle& markers,
                                            ShadowDiagnostics& diagnostics) const {
  markers = ShadowMarkerBundle();
  markers.base_path = MakeLineMarker(
      input.stamp, config_.frame_id, "phase_offset_shadow_base", 0, 0.0F, 0.3F, 1.0F);
  markers.candidate_path = MakeLineMarker(input.stamp,
                                          config_.frame_id,
                                          "phase_offset_shadow_candidate",
                                          0,
                                          1.0F,
                                          0.0F,
                                          1.0F);
  diagnostics = ShadowDiagnostics();
  diagnostics.w = input.current_path.w;
  diagnostics.delta = config_.delta;

  if (!enabled()) {
    diagnostics.invalid_reason = "shadow mode is disabled";
    return false;
  }
  if (!IsFinite(input.position)) {
    diagnostics.invalid_reason = "position is not finite";
    markers.candidate_path = MakeDeleteMarker(
        input.stamp, config_.frame_id, "phase_offset_shadow_candidate", 0);
    AppendFrameDeletes(input.stamp, config_.frame_id, markers.frame);
    return false;
  }

  PathDiagnosticsScanState scan_state;
  std::string first_invalid_sample_reason;
  for (const phase_offset_core::PathDifferentialState& sample : input.sampled_path) {
    AccumulateRawPathDiagnostics(sample, diagnostics, scan_state);
    if (IsFinite(sample.p)) {
      markers.base_path.points.push_back(ToPoint(sample.p));
    }

    phase_offset_core::PhaseOffsetGeometryState sample_geometry;
    if (!geometry_evaluator_.evaluate(sample, input.position, config_.delta,
                                      sample_geometry) ||
        !IsFinite(sample_geometry.p) || !IsFinite(sample_geometry.r)) {
      ++diagnostics.invalid_sample_count;
      if (first_invalid_sample_reason.empty()) {
        first_invalid_sample_reason = sample_geometry.invalid_reason.empty()
                                          ? "sample geometry is not finite"
                                          : sample_geometry.invalid_reason;
      }
      continue;
    }
    const double candidate_z_offset =
        std::abs(sample_geometry.r.z() - sample_geometry.p.z());
    if (IsFinite(candidate_z_offset)) {
      diagnostics.max_abs_candidate_z_offset =
          std::max(diagnostics.max_abs_candidate_z_offset, candidate_z_offset);
    }
    markers.candidate_path.points.push_back(ToPoint(sample_geometry.r));
  }
  diagnostics.candidate_path_complete =
      diagnostics.sample_count > 0U && diagnostics.invalid_sample_count == 0U;
  if (!diagnostics.candidate_path_complete) {
    markers.candidate_path = MakeDeleteMarker(
        input.stamp, config_.frame_id, "phase_offset_shadow_candidate", 0);
  }

  phase_offset_core::PhaseOffsetGeometryState current_geometry;
  if (!geometry_evaluator_.evaluate(
          input.current_path, input.position, config_.delta, current_geometry)) {
    diagnostics.invalid_reason = current_geometry.invalid_reason;
    diagnostics.candidate_path_complete = false;
    markers.candidate_path = MakeDeleteMarker(
        input.stamp, config_.frame_id, "phase_offset_shadow_candidate", 0);
    AppendFrameDeletes(input.stamp, config_.frame_id, markers.frame);
    return false;
  }

  diagnostics.valid = true;
  diagnostics.w = current_geometry.w;
  diagnostics.delta = current_geometry.delta;
  diagnostics.curvature = current_geometry.curvature;
  diagnostics.regularity = current_geometry.regularity;
  diagnostics.e_parallel = current_geometry.e_parallel;
  diagnostics.e_perp_norm = current_geometry.e_perp.norm();

  if (!IsFinite(current_geometry.p) || !IsFinite(current_geometry.r) ||
      !IsFinite(current_geometry.T) || !IsFinite(current_geometry.N)) {
    diagnostics.valid = false;
    diagnostics.invalid_reason = "current frame is not finite";
    diagnostics.candidate_path_complete = false;
    markers.candidate_path = MakeDeleteMarker(
        input.stamp, config_.frame_id, "phase_offset_shadow_candidate", 0);
    AppendFrameDeletes(input.stamp, config_.frame_id, markers.frame);
    return false;
  }

  if (!diagnostics.candidate_path_complete) {
    diagnostics.invalid_reason = first_invalid_sample_reason.empty()
                                     ? "sampled path is empty"
                                     : first_invalid_sample_reason;
  }

  markers.frame.markers.push_back(MakeSphereMarker(input.stamp,
                                                    config_.frame_id,
                                                    0,
                                                    current_geometry.p,
                                                    0.0F,
                                                    0.3F,
                                                    1.0F));
  markers.frame.markers.push_back(MakeSphereMarker(input.stamp,
                                                    config_.frame_id,
                                                    1,
                                                    current_geometry.r,
                                                    1.0F,
                                                    0.0F,
                                                    1.0F));
  markers.frame.markers.push_back(MakeArrowMarker(input.stamp,
                                                   config_.frame_id,
                                                   2,
                                                   current_geometry.p,
                                                   current_geometry.T,
                                                   0.0F,
                                                   1.0F,
                                                   0.0F));
  markers.frame.markers.push_back(MakeArrowMarker(input.stamp,
                                                   config_.frame_id,
                                                   3,
                                                   current_geometry.p,
                                                   current_geometry.N,
                                                   1.0F,
                                                   0.5F,
                                                   0.0F));
  return true;
}

bool PhaseOffsetShadowAdapter::update(const ShadowUpdateInput& input,
                                      ShadowDiagnostics* diagnostics) {
  if (!enabled() || !advertised_ || !isPublishDue(input.stamp)) {
    return false;
  }

  ShadowMarkerBundle markers;
  ShadowDiagnostics local_diagnostics;
  const bool valid = buildMarkers(input, markers, local_diagnostics);
  base_path_pub_.publish(markers.base_path);
  candidate_path_pub_.publish(markers.candidate_path);
  frame_pub_.publish(markers.frame);
  diagnostics_pub_.publish(MakeDiagnosticsMessage(local_diagnostics));
  last_publish_time_ = input.stamp;
  has_published_ = true;

  if (!valid || !local_diagnostics.candidate_path_complete) {
    ROS_WARN_THROTTLE(1.0,
                      "[phase_offset_shadow] invalid geometry: %s "
                      "(p_w.z=%.9g, p_ww.z=%.9g, samples=%zu, "
                      "invalid_samples=%zu, max_abs_p_w.z=%.9g at_w=%.9g, "
                      "max_abs_p_ww.z=%.9g at_w=%.9g)",
                      local_diagnostics.invalid_reason.c_str(),
                      input.current_path.p_w.z(),
                      input.current_path.p_ww.z(),
                      local_diagnostics.sample_count,
                      local_diagnostics.invalid_sample_count,
                      local_diagnostics.max_abs_p_w_z,
                      local_diagnostics.max_abs_p_w_z_at_w,
                      local_diagnostics.max_abs_p_ww_z,
                      local_diagnostics.max_abs_p_ww_z_at_w);
  }
  if (diagnostics != nullptr) {
    *diagnostics = local_diagnostics;
  }
  return valid;
}

}  // namespace FLAG_Race
