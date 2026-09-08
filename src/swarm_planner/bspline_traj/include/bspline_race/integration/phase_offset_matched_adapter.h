#pragma once
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <atomic>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include <bspline_race/continuous_phase_path.h>
#include <bspline_race/continuous_phase_normal_frame.h>
#include <bspline_race/integration/phase_offset_active_adapter.h>
#include <bspline_race/integration/phase_offset_sph_ros_bridge.h>
#include <bspline_race/integration/phase_offset_tube_runtime_v2.h>
#include <phase_offset_navigation/phase_offset_runtime.h>
#include <phase_offset_navigation/phase_offset_allocator.h>
#include <phase_offset_navigation/immutable_executed_reference_query.h>
#include <phase_offset_navigation/active_reference_snapshot.h>
#include <phase_offset_navigation/active_reference_authority.h>
#include <phase_offset_navigation/phase_offset_recovery_owner.h>
#include <phase_offset_navigation/preview_feasibility.h>

namespace FLAG_Race {

class gvf_manager;

enum class PhaseOffsetMatchedMode { ACTIVE = 0, MANUAL = 1 };

// Main-side coordination selector.  The manager resolves the configured
// string to one effective value before constructing the adapter.  D1B is the
// compatibility default for direct adapter fixtures; production manager
// configuration explicitly supplies disabled/d1b/sph as required.
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

struct PhaseOffsetMatchedAdapterConfig {
  PhaseOffsetMatchedMode mode = PhaseOffsetMatchedMode::ACTIVE;
  PhaseOffsetCoordinationBackend coordination_backend =
      PhaseOffsetCoordinationBackend::D1B;
  double equivalence_tolerance = 1e-10;
  int warmup_cycles = 100;
  double amplitude = 0.10;
  // Candidate construction and exact-port evaluation remain enabled, while
  // Runtime gate-closed semantics prevent control selection and state commit.
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
  // Required immutable NORMAL Preview production policy.  Its values are
  // captured by the integration boundary and passed through unchanged to C1;
  // an absent policy is invalid and never replaced with an adapter default.
  phase_offset_navigation::NormalPreviewProductionPolicy
      normal_preview_policy;
  bool normal_preview_policy_explicit = false;
  phase_offset_navigation::TubeSource tube_source =
      phase_offset_navigation::TubeSource::NONE;
  phase_offset_navigation::TubeBuilderConfig tube;
  // Explicit V2 certificate configuration.  A shadow adapter never derives
  // or fabricates this value from the legacy TubeBuilderConfig.
  phase_offset_navigation::TubeCertificateConfigV2 tube_certificate_v2;
  phase_offset_navigation::TubeFilterConfig filter;
  double tube_update_period = 0.10;
  // ESDF-selected tube construction accepts the local occupycloud snapshot
  // only when its producer promises a complete local obstacle set.  The
  // default remains fail-closed until an explicit launch enables it.
  bool cloud_obstacle_set_complete = false;
  std::string frame_id = "world";
  // S3-T0-only sidecar timing is opt-in and never changes tube construction.
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
  }
};

// Immutable command-side view of the manager-owned copied-prefix handoff.
// The manager remains the sole owner of the canonical handoff and frontend
// mirror; this value carries only the facts needed for side-effect-free live
// admission of the already completed SUCCESSOR profile.
struct TubeV2SuccessorHandoffEvidence {
  bool valid = false;
  bool structurally_copied_prefix = false;
  std::uint64_t expected_execution_generation = 0U;
  phase_offset_navigation::TubePathKey source_path_key;
  std::shared_ptr<const ContinuousPhasePath> source_path_owner;
  phase_offset_navigation::TubePathKey successor_path_key;
  std::shared_ptr<const ContinuousPhasePath> successor_path_owner;
  double phase_after_w = std::numeric_limits<double>::quiet_NaN();
  double copied_prefix_start_w = std::numeric_limits<double>::quiet_NaN();
  double copied_prefix_end_w = std::numeric_limits<double>::quiet_NaN();
  std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2>
      successor_request;
  std::string provenance;
};

struct MatchedAdapterInput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Optional value-only V2 admission evidence.  It is captured with the
  // command input for NORMAL admission and incumbent finite-reserve
  // applicability.  It never carries authority, publication permission, or
  // a mutable Runtime reference.
  struct TubeV2AdmissionEvidence {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool valid = false;
    std::uint64_t binding_sequence = 0U;
    std::uint64_t accepted_state_sequence = 0U;
    std::uint64_t accepted_state_notification_sequence = 0U;
    std::uint64_t accepted_time_ticks = 0U;
    std::uint64_t support_expiry_ticks = 0U;
    bool support_expiry_timeless = false;
    std::uint64_t map_instance_id = 0U;
    std::uint64_t configuration_generation = 0U;
    std::uint64_t configuration_key = 0U;
    std::uint64_t support_provenance_id = 0U;
    std::string frame_provenance;
    std::uint64_t latest_accepted_state_sequence = 0U;
    std::uint64_t latest_accepted_state_notification_sequence = 0U;
    std::uint64_t latest_accepted_time_ticks = 0U;
    std::uint64_t latest_map_instance_id = 0U;
    std::uint64_t latest_configuration_generation = 0U;
    std::uint64_t latest_configuration_key = 0U;
    std::uint64_t latest_support_provenance_id = 0U;
    std::string latest_frame_provenance;
    phase_offset_core::PortCommand selected_u;
    std::string selected_u_owner = "PhaseOffsetAllocator";
    double base_phase_rate = std::numeric_limits<double>::quiet_NaN();
    double phase_rate_lower = std::numeric_limits<double>::quiet_NaN();
    double phase_rate_upper = std::numeric_limits<double>::quiet_NaN();
    double upper_u_delta = std::numeric_limits<double>::quiet_NaN();
    double now = std::numeric_limits<double>::quiet_NaN();
    double applicability_deadline = std::numeric_limits<double>::quiet_NaN();
    bool applicability_deadline_valid = false;
    phase_offset_navigation::NormalPreviewProductionPolicy preview_policy;
    phase_offset_navigation::TubeExecutionLimitsV2 limits;
    phase_offset_navigation::TubeExecutionTrackingEvidenceV2 tracking;
    std::size_t max_work = 0U;
    std::string provenance;
  } tube_v2_admission;

  phase_offset_core::PathDifferentialState path;
  double semantic_path_start_w = 0.0;
  double semantic_path_end_w = 0.0;
  // Production ownership handoff.  The manager captures these immutable
  // owners in the command callback before storing a build request.
  std::shared_ptr<const ContinuousPhasePath> semantic_path_owner;
  // Optional immutable V2 worker transport captured by the command owner.
  // The adapter copies this one coherent builder input into the selected
  // worker; absent input leaves V2 execution unavailable without
  // fabricating map/path authority from legacy fields.
  std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2>
      tube_worker_input_v2;
  TubeWorkerPurposeV2 tube_worker_purpose_v2 = TubeWorkerPurposeV2::CURRENT;
  // Optional exact manager handoff captured under its existing mailbox lock.
  // It is not a READY flag and cannot install the successor or its mirror.
  std::shared_ptr<const TubeV2SuccessorHandoffEvidence>
      tube_v2_successor_handoff;
  // SPH is a value-semantic ROS boundary.  The manager captures one immutable
  // bridge sample before calling update(); evaluateNormalAllocator resolves
  // its source/receipt freshness only after successful NORMAL Preview.
  const bspline_race::integration::PhaseOffsetSphRosBridge* sph_bridge =
      nullptr;
  bspline_race::integration::GCoordCapture captured_gcoord;
  // Value-semantic NORMAL interaction boundary.  A producer may provide an
  // exact desired active-reference motion for this tick.  When no producer is
  // connected (single-UAV operation), the adapter supplies the frozen
  // recenter-only value from the current delta and immutable normal.
  Eigen::Vector3d g_des = Eigen::Vector3d::Zero();
  bool g_des_valid = false;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  guidance::IsfGains gains;
  LegacyGuidanceSnapshot legacy;
  double dt = 0.0;
  ros::Time stamp;
};

// The single committed V2 execution binding.  Profile, exact immutable
// builder input and Runtime identity are prepared together and exchanged only
// after a successful PositionCommand publication.
struct TubeV2ExecutionBinding {
  std::shared_ptr<const phase_offset_navigation::TubeProfileV2> profile;
  std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2> source_input;
  std::shared_ptr<const ContinuousPhaseNormalFrame> frame_owner;
  phase_offset_navigation::TubeExecutionIdentityV2 identity;
  bool valid = false;

  bool complete() const;
};

// Immutable result of preparing V2 Runtime work against one already-built
// CURRENT profile.  NORMAL admission becomes selectable only through the
// publish-first transaction; when an incumbent NORMAL command is denied, the
// same value may instead carry one
// exact finite-reserve step and its publish-gated Runtime successor token.
// Neither form is authority before successful PositionCommand publication.
struct TubeV2ShadowAdmissionCandidate {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool applicable = false;
  bool prepared = false;
  bool nonselecting = true;
  phase_offset_navigation::TubeExecutionStatusV2 status =
      phase_offset_navigation::TubeExecutionStatusV2::UNAVAILABLE;
  std::uint64_t request_id = 0U;
  std::uint64_t binding_sequence = 0U;
  std::uint64_t execution_generation = 0U;
  std::uint64_t accepted_state_demand = 0U;
  TubeWorkerPurposeV2 purpose = TubeWorkerPurposeV2::CURRENT;
  phase_offset_navigation::TubePathKey source_path_key;
  double copied_prefix_start_w = std::numeric_limits<double>::quiet_NaN();
  double copied_prefix_end_w = std::numeric_limits<double>::quiet_NaN();
  bool copied_prefix_continuity_valid = false;
  double path_position_residual = std::numeric_limits<double>::quiet_NaN();
  double path_derivative_residual = std::numeric_limits<double>::quiet_NaN();
  double reference_position_residual = std::numeric_limits<double>::quiet_NaN();
  double reference_derivative_residual = std::numeric_limits<double>::quiet_NaN();
  std::uint64_t latest_accepted_state_sequence = 0U;
  std::uint64_t latest_accepted_state_notification_sequence = 0U;
  std::uint64_t latest_accepted_time_ticks = 0U;
  std::uint64_t latest_map_instance_id = 0U;
  std::uint64_t latest_configuration_generation = 0U;
  std::uint64_t latest_configuration_key = 0U;
  std::uint64_t latest_support_provenance_id = 0U;
  std::string latest_frame_provenance;
  bool accepted_update_visible = false;
  bool accepted_update_compatible = false;
  phase_offset_navigation::TubePathKey path_key;
  phase_offset_navigation::TubeConfigurationKey configuration_key;
  phase_offset_navigation::TubeMapCaptureKey map_capture_key;
  std::shared_ptr<const phase_offset_navigation::TubeProfileV2> profile;
  std::shared_ptr<const TubeV2ExecutionBinding> proposed_binding;
  Eigen::Vector3d g_des = Eigen::Vector3d::Zero();
  bool g_des_valid = false;
  phase_offset_navigation::NormalPreviewResult normal_preview;
  phase_offset_navigation::PhaseOffsetAllocatorResult allocator;
  bool allocator_evaluated = false;
  phase_offset_navigation::RuntimeV2PreparedStep prepared_step;
  phase_offset_navigation::RuntimeV2CommitToken commit_token;
  phase_offset_navigation::AuthorityPreparedStep authority_prepared;
  // A failed refresh may prepare the next step of the already committed V2
  // finite reserve.  Cursor and Runtime state advance only through the
  // existing publish-first PositionCommand seam.
  phase_offset_navigation::RecoveryPreparedStep recovery_step;
  std::string reason;
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
  // V2 NORMAL admission is exposed as immutable evidence only.  An incumbent
  // finite-reserve candidate may also describe the exact recovery command
  // selected through the existing publish-first transaction; neither form
  // populates legacy Candidate/Active ownership.
  std::shared_ptr<const TubeV2ShadowAdmissionCandidate>
      v2_shadow_admission_candidate;
  // Value-only execution classification consumed by the existing manager
  // beta/publication seam.  It is not a legacy Pair/Epoch authority.
  phase_offset_navigation::RuntimeExecutionStatus runtime_execution;
  // NORMAL value-core evidence.  These are immutable copies for diagnostics
  // and tests; they do not add a second owner or execution authority.
  Eigen::Vector3d g_des = Eigen::Vector3d::Zero();
  bool g_des_valid = false;
  phase_offset_navigation::NormalPreviewResult normal_preview;
  phase_offset_navigation::PhaseOffsetAllocatorResult allocator;
  bool allocator_evaluated = false;
  // Preview/allocator failure is value-only for neutral planner operation;
  // an active authoritative nonzero tick is fail-closed by the manager while
  // retaining its existing authority snapshot.
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

// Lean command-to-worker transport. It carries no legacy Candidate/Active/
// Epoch state and grants no publication authority.
struct TubeBuildRequestV2 {
  bool active = false;
  std::uint64_t task_generation = 0U;
  std::uint64_t control_sequence = 0U;
  double current_w = std::numeric_limits<double>::quiet_NaN();
  std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2>
      tube_worker_input_v2;
  TubeWorkerPurposeV2 tube_worker_purpose_v2 = TubeWorkerPurposeV2::CURRENT;
};

struct DeactivationCommitToken {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::uint64_t task_generation = 0U;
  std::uint64_t next_control_sequence = 0U;
  bool command_active_before = false;
  bool valid = false;
  std::shared_ptr<const TubeBuildRequestV2> inactive_request;
};

// Immutable command-side capture used to bind the governor's reference query
// to the exact staged PositionCommand transaction that may publish it.  A
// capture with pending=false/valid=true means no transaction was present at
// capture time; pending=true requires the identity to remain unchanged until
// publish.
struct PendingPositionCommandCapture {
  bool pending = false;
  bool valid = false;
  std::uint64_t identity = 0U;
  phase_offset_navigation::ImmutableExecutedReferenceQueryPtr reference_query;
  bool v2_binding_transition = false;
  std::shared_ptr<const TubeV2ExecutionBinding> proposed_v2_binding;
};

class PhaseOffsetMatchedAdapter {
 public:
  explicit PhaseOffsetMatchedAdapter(const PhaseOffsetMatchedAdapterConfig& config);
  ~PhaseOffsetMatchedAdapter();

  static PhaseOffsetMatchedAdapterConfig loadConfig(
      ros::NodeHandle& private_nh, PhaseOffsetMatchedMode mode);
  bool configurationValid() const;
  bool requiresPathSamples() const;
  double sampleStepW() const;
  bool requiresTubeTimer() const;
  // Tube observation remains enabled for MANUAL mode.  This predicate is only
  // true after an offset profile has executed (or a nonzero retained delta is
  // live); pending configuration alone must not force terminal H2 denial.
  bool requiresAuthoritativeOffsetHandoff() const;
  // Begin bounded in-owner recentering after a successor denial.  This does
  // not clear the pair, delta, or planner authority; Runtime retires the
  // profile only after an exact accepted step reaches neutral.
  bool requestRecenter();
  bool recenterRequested() const;
  // A user-issued navigation goal starts a new V2 execution generation and
  // retires every request, completion, binding and pending publication token.
  bool resetForNewNavigationTask();
  void advertise(ros::NodeHandle& private_nh);
  bool update(const MatchedAdapterInput& input, MatchedAdapterOutput& output);
  // Discard only a staged production transaction.  Authority/Runtime commit
  // is intentionally unreachable through a public boolean bypass; production
  // commit occurs only inside publishPendingPositionCommand() after the local
  // PositionCommand publication succeeds.
  void discardPendingPositionCommand();
  bool hasPendingPositionCommand() const;
  bool validatePendingPositionCommand() const;
  PendingPositionCommandCapture capturePendingPositionCommand() const;
  // Returns the immutable executed-reference query staged for the pending
  // PositionCommand transaction.  The governor may consume this value only
  // after validatePendingPositionCommand(); publication revalidates it again
  // under the serialized task/runtime boundary.
  phase_offset_navigation::ImmutableExecutedReferenceQueryPtr
  pendingExecutedReferenceQuery() const;
  // Serializes final validation, local PositionCommand publication and the
  // no-fail authority/token commit against task reset and pair retirement.
  bool publishPendingPositionCommand(
      const std::function<bool()>& local_publish,
      std::uint64_t expected_identity = 0U,
      bool cancel_pending_for_goal_override = false,
      const std::function<void()>& post_publish_no_fail =
          std::function<void()>());
  // Compatibility entry point for the manager timer; it is scheduler-only.
  bool timerTick();
  // Scheduler-only production entry point.  One call represents one ROS Tube
  // timer event and creates at most one pending worker permit.  It never
  // performs heavy Tube construction on the caller thread.
  bool scheduleTubeBuild();
  // Current task generation used to bind immutable manager-built requests.
  // The value is monotonic and grants no execution or publication authority.
  std::uint64_t executionGenerationV2() const;
  // Capture the exact installed V2 source identity for a copied-prefix
  // successor request.  This is immutable transport evidence only; it does
  // not expose or mutate Runtime delta/port state.
  bool captureV2SuccessorSource(
      phase_offset_navigation::TubePathKey& source_path_key,
      std::uint64_t& execution_generation,
      std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2>&
          source_input) const;
  // Captures the one committed immutable V2 binding.  It is value/owner
  // evidence only and never grants a second commit path.
  std::shared_ptr<const TubeV2ExecutionBinding>
  captureV2ExecutionBinding() const;
  // Queue one immutable successor proof on the adapter-owned V2 worker.
  // The source key must still name the installed incumbent.  Submission is
  // nonblocking and never invokes TubeCertificateBuilderV2 on the caller.
  bool enqueueV2SuccessorRequest(
      const phase_offset_navigation::TubePathKey& source_path_key,
      const phase_offset_navigation::TubeBuildInputV2& successor_input);
  // Command-thread transition used when no goal/path is active.  It clears
  // only async request/tube exposure and queues one timer-side DELETE; Runtime
  // execution state (delta, previous port, profile lifecycle) is preserved.
  void deactivate(const ros::Time& stamp = ros::Time(0));
  // Manager shutdown order is: set this flag, stop the ROS timer, wait for an
  // in-flight tick, clear the atomic snapshots, then destroy this adapter.
  void requestShutdown();
 void shutdown();
 private:
  bool requiresAuthoritativeOffsetHandoffLocked() const;
  void deactivateLocked(const ros::Time& stamp,
                        std::uint64_t expected_task_generation);
  bool prepareDeactivationLocked(
      const ros::Time& stamp, std::uint64_t expected_task_generation,
      DeactivationCommitToken& token) const;
  void commitDeactivationNoFailLocked(
      const DeactivationCommitToken& token) noexcept;
  std::shared_ptr<const TubeBuildRequestV2> makeBuildRequestV2(
      const MatchedAdapterInput& input);
  bool updateGate(const MatchedAdapterInput& input, MatchedAdapterOutput& output);
  void clearPendingPositionCommandLocked();
  bool validatePendingPositionCommandLocked(std::string* reason) const;
  void latchFailure(phase_offset_navigation::ControlFailureReason reason);

  // V2 worker deduplication is value based.  In particular this identity
  // must not retain TubeBuildRequestV2 (or its immutable input owner)
  // after an in-flight submit/completion no longer needs that object.  The
  // complete immutable keys are retained so allocator ABA/reused addresses
  // cannot make an unrelated request look like a duplicate.
  struct V2ShadowRequestIdentity {
    TubeWorkerPurposeV2 purpose = TubeWorkerPurposeV2::CURRENT;
    std::uint64_t request_id = 0U;
    std::uint64_t execution_generation = 0U;
    std::uint64_t accepted_state_demand = 0U;
    double useful_start = 0.0;
    double useful_end = 0.0;
    phase_offset_navigation::TubePathKey path_key;
    phase_offset_navigation::TubeConfigurationKey configuration_key;
    phase_offset_navigation::TubeMapCaptureKey map_capture_key;

    bool valid() const { return request_id != 0U; }
    bool operator==(const V2ShadowRequestIdentity& other) const {
      return purpose == other.purpose && request_id == other.request_id &&
          execution_generation == other.execution_generation &&
          accepted_state_demand == other.accepted_state_demand &&
          useful_start == other.useful_start && useful_end == other.useful_end &&
          path_key == other.path_key &&
          configuration_key == other.configuration_key &&
          map_capture_key == other.map_capture_key;
    }
    bool operator!=(const V2ShadowRequestIdentity& other) const {
      return !(*this == other);
    }
  };

  V2ShadowRequestIdentity makeV2ShadowRequestIdentity(
      const TubeWorkerPurposeV2 purpose,
      const phase_offset_navigation::TubeBuildInputV2& input) const;
  void retainV2ShadowCompletionLocked(
      const TubeWorkerCompletionV2& completion);
  static bool candidateMarkerApplicableV2(
      const TubeWorkerCompletionV2& completion, const TubeBuildRequestV2& request,
      std::uint64_t generation, std::uint64_t now_ticks);
  bool prepareV2ShadowAdmission(
      const MatchedAdapterInput& input,
      const std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2>&
          source,
      const std::shared_ptr<const TubeWorkerCompletionV2>& completion,
      std::shared_ptr<const TubeV2ShadowAdmissionCandidate>& candidate,
      TubeWorkerPurposeV2 purpose = TubeWorkerPurposeV2::CURRENT);
  bool prepareV2ShadowSuccessorAdmission(
      const MatchedAdapterInput& input,
      const TubeV2SuccessorHandoffEvidence& handoff,
      const std::shared_ptr<const TubeWorkerCompletionV2>& completion,
      std::shared_ptr<const TubeV2ShadowAdmissionCandidate>& candidate);
  bool prepareV2ShadowIncumbentApplicability(
      const MatchedAdapterInput& input,
      const std::shared_ptr<const phase_offset_navigation::TubeProfileV2>&
          incumbent,
      std::shared_ptr<const TubeV2ShadowAdmissionCandidate>& candidate,
      phase_offset_navigation::RecoveryPreparedStep* recovery_step = nullptr,
      bool force_reserve = false);
  bool evaluateV2NormalAllocator(
      const MatchedAdapterInput& input,
      const std::shared_ptr<const phase_offset_navigation::TubeProfileV2>&
          profile,
      phase_offset_core::PhaseOffsetGeometryState& geometry,
      guidance::IsfGuidance& base,
      phase_offset_navigation::NormalPreviewResult& preview,
      phase_offset_navigation::PhaseOffsetAllocatorResult& allocator,
      Eigen::Vector3d& g_des,
      std::string& reason) const;
  bool stageV2ShadowBootstrapLocked(
      const MatchedAdapterInput& input,
      const phase_offset_navigation::TubeBuildInputV2& source,
      const std::shared_ptr<const TubeV2ShadowAdmissionCandidate>& candidate);
  bool stageV2ShadowRecoveryLocked(
      const MatchedAdapterInput& input,
      const std::shared_ptr<const TubeV2ShadowAdmissionCandidate>& candidate);
  bool stageV2ShadowSuccessorLocked(
      const MatchedAdapterInput& input,
      const TubeV2SuccessorHandoffEvidence& handoff,
      const std::shared_ptr<const TubeV2ShadowAdmissionCandidate>& candidate);
  bool prepareV2ExecutionAuthorityLocked(
      const MatchedAdapterInput& input,
      const MatchedAdapterOutput& output,
      phase_offset_navigation::ActiveReferenceOwnerMode owner_mode,
      const std::shared_ptr<const TubeV2ExecutionBinding>& binding,
      TubeV2ShadowAdmissionCandidate& candidate,
      std::string* reason);
  bool validatePendingV2ShadowBootstrapLocked(std::string* reason) const;
  bool scheduleV2ShadowBuild();
  void consumeV2ShadowCompletions();
  void publishV2TubeMarkers(const ros::Time& stamp);
  void joinTubeWorker();

  struct TubeMarkerIdentityV2 {
    std::uint64_t execution_generation = 0U;
    std::uint64_t path_instance_id = 0U;
    std::uint64_t profile_id = 0U;
    std::uint64_t request_id = 0U;
    std::uint64_t map_state_id = 0U;
    bool valid = false;

    bool operator==(const TubeMarkerIdentityV2& other) const {
      return execution_generation == other.execution_generation &&
          path_instance_id == other.path_instance_id &&
          profile_id == other.profile_id && request_id == other.request_id &&
          map_state_id == other.map_state_id && valid == other.valid;
    }
    bool operator!=(const TubeMarkerIdentityV2& other) const {
      return !(*this == other);
    }
  };

  friend class gvf_manager;
  friend class GvfManagerS4AnchorTestAccess;

  PhaseOffsetMatchedAdapterConfig config_;
  bool configuration_valid_ = false;
  PhaseOffsetActiveAdapter zero_port_adapter_;
  std::unique_ptr<phase_offset_navigation::PhaseOffsetRuntime> runtime_;
  // Runtime/recovery own V2 execution state; this retained authority publishes
  // the task-level execution identity at the existing command boundary.
  phase_offset_navigation::PhaseOffsetExecutionAuthority execution_authority_;
  phase_offset_navigation::PhaseOffsetRecoveryOwner recovery_owner_;
  mutable std::mutex runtime_command_mutex_;
  int zero_gate_consecutive_count_ = 0;
  bool zero_gate_open_ = false;
  bool failure_latched_ = false;
  phase_offset_navigation::ControlFailureReason control_failure_reason_ =
      phase_offset_navigation::ControlFailureReason::NONE;
  std::uint64_t control_sequence_ = 0U;
  bool command_active_ = false;
  std::shared_ptr<const TubeBuildRequestV2> latest_build_request_;
  // worker_state_mutex_: one CURRENT proof cohort, not a queue of demands.
  std::shared_ptr<const TubeBuildRequestV2> pinned_current_cohort_;
  void releaseCurrentCohort(std::uint64_t request_id);
  mutable std::mutex task_publication_mutex_;
  std::atomic<std::uint64_t> task_generation_ {1U};
  std::atomic<bool> shutdown_requested_ {false};

  // The mutex protects the single V2 worker lifecycle and completion slots.
  mutable std::mutex worker_state_mutex_;
  std::unique_ptr<PhaseOffsetTubeWorkerV2> v2_shadow_worker_;
  std::shared_ptr<const TubeWorkerCompletionV2>
      latest_v2_shadow_current_completion_;
  std::shared_ptr<const TubeWorkerCompletionV2>
      latest_v2_shadow_successor_completion_;
  std::shared_ptr<const TubeV2ShadowAdmissionCandidate>
      latest_v2_shadow_admission_candidate_;
  // Retain only the last successful binding identity for the exact immutable
  // completion cohort.  A failed retry still replaces the diagnostic
  // candidate above, but it must not erase the binding against which a later
  // substitution is checked.
  std::shared_ptr<const TubeV2ShadowAdmissionCandidate>
      retained_v2_shadow_admission_candidate_;
  // One pending publish-first V2 transaction: bootstrap, successor, or one
  // exact finite-reserve step.
  std::shared_ptr<const TubeV2ShadowAdmissionCandidate>
      pending_v2_shadow_bootstrap_candidate_;
  std::shared_ptr<const TubeV2ExecutionBinding> v2_execution_binding_;
  V2ShadowRequestIdentity v2_shadow_last_current_identity_;
  V2ShadowRequestIdentity v2_shadow_last_successor_identity_;
  phase_offset_navigation::TubePathKey v2_shadow_last_current_path_key_;
  phase_offset_navigation::TubePathKey v2_shadow_last_successor_path_key_;
  bool v2_shadow_have_current_path_key_ = false;
  bool v2_shadow_have_successor_path_key_ = false;
  bool worker_started_ = false;
  bool worker_stop_requested_ = false;
  ros::Publisher active_diagnostics_pub_;
  // Visualization is a latched, timer-side observer of immutable V2 values.
  // It neither owns nor admits a profile and never runs on the 50 Hz command
  // path.  Identity caching avoids repeatedly serializing unchanged geometry.
  mutable std::mutex marker_publication_mutex_;
  ros::Publisher manual_tube_pub_;
  ros::Publisher manual_tube_candidate_pub_;
  TubeMarkerIdentityV2 published_active_marker_identity_;
  TubeMarkerIdentityV2 published_candidate_marker_identity_;
  bool marker_snapshot_published_ = false;
  visualization_msgs::MarkerArray cached_active_markers_v2_;
  visualization_msgs::MarkerArray cached_candidate_markers_v2_;
  std::size_t cached_active_first_knot_v2_ = 0U;
  std::size_t cached_candidate_first_knot_v2_ = 0U;
};

}  // namespace FLAG_Race
