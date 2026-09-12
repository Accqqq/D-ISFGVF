#pragma once

#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include <bspline_race/continuous_phase_path.h>
#include <bspline_race/continuous_phase_normal_frame.h>
#include <bspline_race/guidance/isf_reference_kernel.h>
#include <bspline_race/integration/phase_offset_active_adapter.h>
#include <bspline_race/integration/phase_offset_section_input.h>
#include <bspline_race/integration/phase_offset_sph_ros_bridge.h>
#include <phase_offset_navigation/phase_offset_allocator.h>
#include <phase_offset_navigation/phase_offset_runtime.h>
#include <phase_offset_navigation/preview_feasibility.h>
#include <phase_offset_navigation/tube_filter.h>
#include <phase_offset_navigation/tube_types.h>
#include <phase_offset_navigation/tube_viability.h>

namespace FLAG_Race {

enum class PhaseOffsetMatchedMode { ACTIVE = 0, MANUAL = 1 };

enum class PhaseOffsetCoordinationBackend {
  DISABLED = 0,
  D1B = 1,
  SPH = 2,
  kDisabled = DISABLED,
  kD1B = D1B,
  kSph = SPH
};

const char* phaseOffsetCoordinationBackendName(
    PhaseOffsetCoordinationBackend backend);

// ROS/configuration value copied into the adapter.  MANUAL production consumes
// only the SectionBuildConfig and the numeric Preview/Allocator policy below;
// no worker, certificate identity, map identity, or reserve is constructed.
struct PhaseOffsetMatchedAdapterConfig {
  PhaseOffsetMatchedMode mode = PhaseOffsetMatchedMode::ACTIVE;
  PhaseOffsetCoordinationBackend coordination_backend =
      PhaseOffsetCoordinationBackend::D1B;
  double equivalence_tolerance = 1e-10;
  int warmup_cycles = 100;
  double amplitude = 0.10;
  bool observe_only = true;
  double profile_period = 10.0;
  double delta_tracking_gain = 3.0;
  double u_w_amplitude = 0.05;
  double u_w_abs_max = 0.12;
  double u_delta_abs_max = 0.25;
  double u_w_rate_max = 0.60;
  double u_delta_rate_max = 1.20;
  double phase_dot_min = 0.02;
  double tangent_speed_min = 0.02;
  double preflight_sample_step_w = 0.10;

  phase_offset_navigation::NormalPreviewProductionPolicy
      normal_preview_policy;
  bool normal_preview_policy_explicit = false;
  phase_offset_navigation::TubeSource tube_source =
      phase_offset_navigation::TubeSource::NONE;
  phase_offset_navigation::TubeBuilderConfig tube;
  phase_offset_navigation::TubeFilterConfig filter;
  phase_offset_navigation::SectionBuildConfig section_build;
  double tube_update_period = 0.10;
  bool cloud_obstacle_set_complete = false;
  std::string frame_id = "world";
  bool measurement_enabled = false;
  std::string measurement_tube_due_csv_path;

  PhaseOffsetMatchedAdapterConfig() {
    tube.cross_section.search_extent = 3.0;
    tube.cross_section.ray_step = 0.05;
    tube.cross_section.boundary_tolerance = 0.01;
    tube.cross_section.regularity_margin = 0.10;
    tube.cross_section.curvature_epsilon = 1e-9;
    tube.cross_section.margins.uav_radius = 0.25;
    tube.cross_section.margins.map_uncertainty = 0.10;
    tube.cross_section.margins.localization_uncertainty = 0.05;
    tube.cross_section.margins.tracking_error_bound = 0.15;
    tube.cross_section.margins.preincluded_map_uncertainty = 0.0;
    section_build.clearance = tube.cross_section.planner_safe_distance;
    section_build.half_width = tube.cross_section.search_extent;
    section_build.minimum_reference_speed =
        tube.cross_section.minimum_reference_speed;
    section_build.min_regularity_ratio = 0.35;
    section_build.curvature_epsilon = tube.cross_section.curvature_epsilon;
    section_build.max_step_w = tube.sample_step_w;
  }
};

struct MatchedAdapterInput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  phase_offset_core::PathDifferentialState path;
  double semantic_path_start_w = 0.0;
  double semantic_path_end_w = 0.0;
  std::shared_ptr<const ContinuousPhasePath> semantic_path_owner;
  SectionPathBundlePtr section_bundle;

  const bspline_race::integration::PhaseOffsetSphRosBridge* sph_bridge =
      nullptr;
  bspline_race::integration::GCoordCapture captured_gcoord;
  Eigen::Vector3d g_des = Eigen::Vector3d::Zero();
  bool g_des_valid = false;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  guidance::IsfGains gains;
  LegacyGuidanceSnapshot legacy;
  double dt = 0.0;
  ros::Time stamp;
};

struct MatchedAdapterOutput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  guidance::IsfGuidance guidance;
  guidance::IsfGuidance base_guidance;
  phase_offset_core::PhaseOffsetGeometryState geometry;
  phase_offset_core::PortCommand raw_port;
  phase_offset_core::PortProjectionResult projection;
  phase_offset_core::MatchedPortOutput matched;
  ActiveAdapterOutput zero_port;
  ActiveEquivalenceResult zero_comparison;
  SectionPathBundlePtr section_bundle;
  std::shared_ptr<const phase_offset_navigation::RuntimeSectionPreparedStep>
      section_prepared_step;
  phase_offset_navigation::RuntimeExecutionStatus runtime_execution;
  Eigen::Vector3d g_des = Eigen::Vector3d::Zero();
  bool g_des_valid = false;
  phase_offset_navigation::NormalPreviewResult normal_preview;
  phase_offset_navigation::PhaseOffsetAllocatorResult allocator;
  bool allocator_evaluated = false;
  bool allocator_value_failure = false;
  double delta_ref = 0.0;
  double delta = 0.0;
  bool zero_gate_open = false;
  int zero_gate_consecutive_count = 0;
  bool failure_latched = false;
  phase_offset_navigation::ControlFailureReason control_failure_reason =
      phase_offset_navigation::ControlFailureReason::NONE;
  bool profile_active = false;
  bool selected = false;
  bool valid = false;
  phase_offset_navigation::RecoveryStepStatus recovery_status =
      phase_offset_navigation::RecoveryStepStatus::NONE;
  bool recovery_replan_required = false;
  std::string invalid_reason;
};

struct PendingPositionCommandCapture {
  bool pending = false;
  bool valid = false;
  std::uint64_t identity = 0U;
  phase_offset_navigation::ImmutableExecutedReferenceQueryPtr reference_query;
  bool section_transition = false;
  SectionPathBundlePtr section_bundle;
  std::shared_ptr<const ContinuousPhasePath> section_source_path;
  double section_copied_prefix_start_w =
      std::numeric_limits<double>::quiet_NaN();
  double section_copied_prefix_end_w =
      std::numeric_limits<double>::quiet_NaN();
  double section_current_w = std::numeric_limits<double>::quiet_NaN();
  double section_next_w = std::numeric_limits<double>::quiet_NaN();
};

struct SectionBundleCandidateCapture {
  SectionPathBundlePtr bundle;
  std::shared_ptr<const ContinuousPhasePath> source_path;
  double copied_prefix_start_w = std::numeric_limits<double>::quiet_NaN();
  double copied_prefix_end_w = std::numeric_limits<double>::quiet_NaN();
};

class PhaseOffsetMatchedAdapter {
 public:
  explicit PhaseOffsetMatchedAdapter(
      const PhaseOffsetMatchedAdapterConfig& config);
  ~PhaseOffsetMatchedAdapter();

  static PhaseOffsetMatchedAdapterConfig loadConfig(
      ros::NodeHandle& private_nh, PhaseOffsetMatchedMode mode);

  bool configurationValid() const;
  bool requiresPathSamples() const;
  double sampleStepW() const;
  bool requiresTubeTimer() const;
  bool requiresAuthoritativeOffsetHandoff() const;
  bool requestRecenter();
  bool recenterRequested() const;
  bool resetForNewNavigationTask();

  void advertise(ros::NodeHandle& private_nh);
  bool update(const MatchedAdapterInput& input, MatchedAdapterOutput& output);

  bool stageSectionBundle(
      const SectionPathBundlePtr& bundle,
      const std::shared_ptr<const ContinuousPhasePath>& source_path,
      double copied_prefix_start_w,
      double copied_prefix_end_w);
  SectionPathBundlePtr captureSectionBundle() const;
  SectionPathBundlePtr capturePendingSectionBundle() const;
  SectionBundleCandidateCapture capturePendingSectionCandidate() const;

  void deactivate(const ros::Time& stamp = ros::Time(0));
  void requestShutdown();
  void shutdown();

  bool hasPendingPositionCommand() const;
  void discardPendingPositionCommand();
  bool validatePendingPositionCommand() const;
  PendingPositionCommandCapture capturePendingPositionCommand() const;
  phase_offset_navigation::ImmutableExecutedReferenceQueryPtr
  pendingExecutedReferenceQuery() const;
  bool publishPendingPositionCommand(
      const std::function<bool()>& local_publish,
      std::uint64_t expected_identity = 0U,
      bool cancel_pending_for_goal_override = false,
      const std::function<void()>& post_publish_no_fail =
          std::function<void()>());

  // Monotonic task-generation witness used to bind manager-owned Section
  // bundles.  It is not a map/profile identity and never enters navigation.
  std::uint64_t executionGeneration() const;

 private:
  bool requiresAuthoritativeOffsetHandoffLocked() const;
  void clearPendingPositionCommandLocked();
  bool validatePendingPositionCommandLocked(std::string* reason) const;
  bool updateGate(const MatchedAdapterInput& input,
                  MatchedAdapterOutput& output);
  bool updateSectionLocked(const MatchedAdapterInput& input,
                           MatchedAdapterOutput& output);
  void latchFailure(phase_offset_navigation::ControlFailureReason reason);

  friend class gvf_manager;
  friend class GvfManagerS4AnchorTestAccess;

  PhaseOffsetMatchedAdapterConfig config_;
  bool configuration_valid_ = false;
  PhaseOffsetActiveAdapter zero_port_adapter_;
  std::unique_ptr<phase_offset_navigation::PhaseOffsetRuntime> runtime_;

  mutable std::mutex runtime_command_mutex_;
  mutable std::mutex task_publication_mutex_;
  std::atomic<std::uint64_t> task_generation_{1U};
  std::atomic<bool> shutdown_requested_{false};

  int zero_gate_consecutive_count_ = 0;
  bool zero_gate_open_ = false;
  bool failure_latched_ = false;
  phase_offset_navigation::ControlFailureReason control_failure_reason_ =
      phase_offset_navigation::ControlFailureReason::NONE;

  SectionPathBundlePtr staged_section_bundle_;
  std::shared_ptr<const ContinuousPhasePath> staged_section_source_path_;
  double staged_section_copied_prefix_start_w_ =
      std::numeric_limits<double>::quiet_NaN();
  double staged_section_copied_prefix_end_w_ =
      std::numeric_limits<double>::quiet_NaN();

  SectionPathBundlePtr section_bundle_;
  SectionPathBundlePtr pending_section_bundle_;
  std::shared_ptr<const ContinuousPhasePath> pending_section_source_path_;
  double pending_section_copied_prefix_start_w_ =
      std::numeric_limits<double>::quiet_NaN();
  double pending_section_copied_prefix_end_w_ =
      std::numeric_limits<double>::quiet_NaN();
  double pending_section_current_w_ =
      std::numeric_limits<double>::quiet_NaN();
  std::shared_ptr<const phase_offset_navigation::RuntimeSectionPreparedStep>
      pending_section_prepared_step_;
  phase_offset_navigation::ImmutableExecutedReferenceQueryPtr
      pending_section_reference_query_;
  std::uint64_t next_section_pending_identity_ = 0U;
  std::uint64_t section_pending_identity_ = 0U;

  ros::Publisher active_diagnostics_pub_;
};

}  // namespace FLAG_Race
