#pragma once

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <cstddef>
#include <string>
#include <vector>

#include <bspline_race/continuous_phase_path.h>
#include <phase_offset_core/geometry.h>

namespace FLAG_Race {

enum class PhaseOffsetShadowMode {
  DISABLED = 0,
  SHADOW = 1,
};

struct ShadowConfig {
  PhaseOffsetShadowMode mode = PhaseOffsetShadowMode::DISABLED;
  double delta = 0.0;
  double sample_step_w = 0.10;
  double publish_rate = 5.0;
  std::string frame_id = "world";
};

struct ShadowUpdateInput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  phase_offset_core::PathDifferentialState current_path;
  std::vector<phase_offset_core::PathDifferentialState,
              Eigen::aligned_allocator<phase_offset_core::PathDifferentialState>>
      sampled_path;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  ros::Time stamp;
};

struct ShadowDiagnostics {
  bool valid = false;
  double w = 0.0;
  double delta = 0.0;
  double curvature = 0.0;
  double regularity = 0.0;
  double e_parallel = 0.0;
  double e_perp_norm = 0.0;
  std::size_t sample_count = 0U;
  std::size_t invalid_sample_count = 0U;
  double path_z_min = 0.0;
  double path_z_max = 0.0;
  double path_z_span = 0.0;
  double max_abs_p_w_z = 0.0;
  double max_abs_p_w_z_at_w = 0.0;
  double max_abs_p_ww_z = 0.0;
  double max_abs_p_ww_z_at_w = 0.0;
  double min_horizontal_path_speed = 0.0;
  double min_horizontal_path_speed_at_w = 0.0;
  double min_horizontal_to_total_speed_ratio = 0.0;
  double min_horizontal_to_total_speed_ratio_at_w = 0.0;
  double max_abs_candidate_z_offset = 0.0;
  bool candidate_path_complete = false;
  std::string invalid_reason;
};

struct ShadowMarkerBundle {
  visualization_msgs::Marker base_path;
  visualization_msgs::Marker candidate_path;
  visualization_msgs::MarkerArray frame;
};

phase_offset_core::PathDifferentialState ConvertContinuousPhasePathState(
    const ContinuousPhasePathState& source,
    double w);

class PhaseOffsetShadowAdapter {
 public:
  explicit PhaseOffsetShadowAdapter(const ShadowConfig& config);

  void advertise(ros::NodeHandle& private_nh);

  bool enabled() const;
  double sampleStepW() const;
  bool isPublishDue(const ros::Time& stamp) const;

  bool buildMarkers(const ShadowUpdateInput& input,
                    ShadowMarkerBundle& markers,
                    ShadowDiagnostics& diagnostics) const;

  bool update(const ShadowUpdateInput& input,
              ShadowDiagnostics* diagnostics = nullptr);

 private:
  ShadowConfig config_;
  phase_offset_core::GeometryEvaluator geometry_evaluator_;
  ros::Publisher base_path_pub_;
  ros::Publisher candidate_path_pub_;
  ros::Publisher frame_pub_;
  ros::Publisher diagnostics_pub_;
  ros::Time last_publish_time_;
  bool has_published_ = false;
  bool advertised_ = false;
};

}  // namespace FLAG_Race
