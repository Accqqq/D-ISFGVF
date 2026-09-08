#ifndef  _GVF_MANAGER_H
#define  _GVF_MANAGER_H   

//standard
#include <fstream>
#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <regex>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <math.h>
#include <numeric>
#include <memory>
#include <thread>
#include <mutex>
#include <vector>
#include <deque>
#include <sstream>
#include <iomanip>
#include <limits>
#include <Eigen/Dense>
//ros
#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/PointCloud2.h> 
#include <sensor_msgs/Imu.h> 
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>
#include <ros/topic_manager.h>
#include <std_msgs/String.h>

//自定义
#include <bspline_race/UniformBspline_3d.h>
#include <bspline_race/bspline_opt_3d.h>
#include "common_msgs/common_msgs.h"
#include <plan_env/edt_environment.h>
#include <path_searching/astar_topo.h>
#include <path_searching/kinodynamic_astar.h>
#include "bspline_race/gvf.h"
#include "bspline_race/integration/phase_offset_cloud_occupancy_query.h"
#include "bspline_race/integration/phase_offset_matched_adapter.h"
#include "bspline_race/integration/phase_offset_shadow_adapter.h"

using namespace std;

#define PI acos(-1)
#define INF 999.9
double delta_T = 0.02;
double last_yaw;
double last_yaw_dot;
double roll, pitch, yaw;//定义存储r\p\y的容器
double D_YAW_MAX = PI/2;
double output_yaw;
double output_d_yaw;
double YAW_MAX = D_YAW_MAX * delta_T;

namespace FLAG_Race
{

// Preallocated planner/display values belonging to one immutable successor
// owner.  This is payload only: retaining it neither installs a path nor
// grants command-publication authority.
struct PathReferenceFrontendMirrorV2
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::MatrixXd traj;
    Eigen::MatrixXd vel;
    Eigen::VectorXd time;
    std::vector<double> w;
    int anchor_idx = 0;

    bool complete() const;
};

// Small copied-prefix handoff evidence prescribed by the frozen V2 design.
// S6-A populates the immutable SUCCESSOR request and leaves successor_profile
// empty; later Stage-6 batches alone may admit a completion and commit the
// binding/mirror at the publication boundary.
struct PathReferenceHandoffV2
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::uint64_t expected_execution_generation = 0U;
    phase_offset_navigation::TubePathKey source_path_key;
    std::shared_ptr<const ContinuousPhasePath> successor_path_owner;
    phase_offset_navigation::TubePathKey successor_path_key;
    double successor_phase_after_w = 0.0;
    double copied_prefix_start_w = 0.0;
    double copied_prefix_end_w = 0.0;
    std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2>
        successor_request;
    // Precomputed adapter-facing view of this same immutable handoff.  The
    // command callback copies only this shared owner; it never reconstructs
    // copied-prefix authority or allocates a new admission payload.
    std::shared_ptr<const TubeV2SuccessorHandoffEvidence>
        admission_evidence;
    std::shared_ptr<const phase_offset_navigation::TubeProfileV2>
        successor_profile;
    std::shared_ptr<const PathReferenceFrontendMirrorV2> frontend_mirror;

    bool requestComplete() const;
    bool committedComplete() const;
};

class gvf_manager
{
    public:
        double init_bias_x, init_bias_y;
        double planInterval;
        double safe_distance_;  // 安全距离参数
        double collision_threshold_;  // 碰撞检测阈值参数
        ros::Time last_replan_time_;  // 上次重规划时间
        ros::Time last_switch_time_;  // 上次接受新轨迹时间（抗抖：最小保持时间）
        int current_traj_index_;  // 当前轨迹执行索引
        double last_yaw;  // 上次yaw角度
        std::string cloud_topic_, odom_topic_, cmd_topic_;
        Eigen::Vector3d odom_;
        bool use_kinopath_ ;
        int num_points_to_take_;  // 获取的路径点数量
        double exec_timer_interval;  // exec_timer的重规划时间间隔
        double kino_timer_interval;  // kino_timer的重规划时间间隔
        double kino_sample_ts_ = 0.2;
        double kino_sample_ts_min_ = 0.05;

        double slow_radius = 1.0;//开始减速半径
        double stop_radius = 0.3;//判定到达目标点半径  
        double goal_reach_radius_ = 2.0;// m，判定到达目标点半径
        double start_pt_change_threshold_ = 1.0; // m，起点变化阈值

        double cmd_vel_max_ = 1.25;          // m/s，GVF 输出速度限幅
        double cmd_acc_max_ = 1.5;           // m/s^2，仅用于 governor 输出诊断
        double cmd_pos_gain_equiv_ = 1.10;   // 位置环等效增益：v_actual ≈ K * position_error
        double cmd_switch_motion_limit_time_ = 0.25; // s，仅用于切换窗口诊断
        double cmd_tangent_vel_max_ = 2.0;
        double cmd_governor_l_min_ = 0.0;
        double cmd_governor_l_max_ = 1.6;
        double cmd_governor_l_step_ = 0.05;
        double cmd_governor_l_rate_max_ = 4.0;
        double cmd_governor_l_ff_weight_ = 0.8;
        double cmd_governor_lead_max_ = 1.6;
        double cmd_governor_normal_cross_max_ = 0.08;
        double cmd_governor_normal_deadband_ = 0.05;
        double cmd_governor_normal_full_error_ = 0.35;
        double cmd_governor_normal_max_ = 0.0;
        double cmd_governor_normal_rate_max_ = 0.6;
        double cmd_governor_l_rate_weight_ = 0.005;
        double cmd_governor_normal_weight_ = 0.60;
        double cmd_governor_normal_rate_weight_ = 0.10;
        double cmd_governor_tau_vel_weight_ = 1.0;
        double cmd_governor_normal_vel_weight_ = 1.0;
        double cmd_governor_normal_vel_error_cap_ = 2.0;
        double switch_governor_path_margin_w_ = 0.2;

        bool cmd_gain_test_enable_ = false;
        double cmd_gain_test_lead_ = 0.4;
        int cmd_gain_test_axis_ = 0;  // 0 表示 x 方向，1 表示 y 方向

        Eigen::Vector3d ref_pos;//上一条命令位置
        bool ref_initialized = false;//是否已经初始化命令状态
        Eigen::Vector3d last_cmd_pos_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d last_curve_vel_ = Eigen::Vector3d::Zero();
        bool has_last_curve_vel_ = false;
        Eigen::Vector3d last_governor_cmd_pos_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d last_governor_cmd_vel_ = Eigen::Vector3d::Zero();
        bool has_last_governor_cmd_ = false;
        // Yaw history is part of the same command transaction as the
        // PositionCommand publication.  The callback computes a candidate;
        // this value is installed only after the adapter/authority commit.
        double pending_last_yaw_ = 0.0;
        bool pending_last_yaw_valid_ = false;
        Eigen::Vector3d cmd_governor_normal_state_ = Eigen::Vector3d::Zero();
        bool cmd_governor_initialized_ = false;
        double cmd_governor_last_l_ = 0.0;
        ros::Time cmd_switch_motion_limit_until_;
        struct OdomPosSample { ros::Time t; Eigen::Vector3d p; };
        std::deque<OdomPosSample> odom_pos_history_;
        Eigen::Vector3d odom_vel_est_ = Eigen::Vector3d::Zero();//由 odom 位置差分估计的真实速度
        Eigen::Vector3d odom_vel_lpf_ = Eigen::Vector3d::Zero();
        bool odom_vel_initialized_ = false;
        double odom_vel_est_window_ = 0.3;
        double odom_vel_lpf_hz_ = 2.0;
        Eigen::Vector3d last_odom_pos_ = Eigen::Vector3d::Zero();
        ros::Time last_odom_time_;
        bool has_last_odom_ = false;

        // 碰撞触发重规划的去抖
        int collision_check_horizon_pts_ = 120;       // 只检查未来 N 个轨迹点
        int collision_consecutive_hits_ = 3;         // 连续 K 个点触发才算碰撞风险

        // 轨迹切换的去抖（连续满足 K 次才允许触发）
        int switch_confirm_goal_progress_cnt_ = 0;
        int switch_confirm_track_error_cnt_ = 0;
        int switch_confirm_near_end_cnt_ = 0;

        // 新轨迹最近点索引去抖：避免 new_i0=nearestIdxInTraj() 在对称/平坦段来回跳导致 topo_dev/topo_side 抖动
        int last_new_i0_ = 0;
        bool has_last_new_i0_ = false;
        
        // 轨迹拼接参数
        int max_trajectory_concatenation_points_;  // 最大轨迹拼接点数
        bool enable_trajectory_concatenation_;     // 是否启用轨迹拼接


        double progress_w_ = 0.0;
        bool progress_initialized_ = false;
        std::string closed_tracking_mode_ = "legacy";
        bool point_phase_v2_enabled_ = false;
        bool point_phase_c2_enabled_ = false;
        double point_phase_endpoint_margin_w_ = 0.05;
        // The H2 phase tuple is shared by the 50 Hz command callback and the
        // asynchronous FSM/replan callback.  Direct access is prohibited:
        // capture/publish it only through the short private helpers below so
        // a future-seam transaction receives one coherent w/init/acquired
        // snapshot without holding a lock during planning or tube work.
        double phase_w_ = 0.0;
        bool phase_initialized_ = false;
        bool closed_phase_acquired_ = false;
        double closed_phase_acquire_distance_ = 0.5;
        double closed_phase_pending_path_end_w_ = 0.0;
        bool closed_phase_has_pending_path_end_w_ = false;
        bool closed_phase_c2_enabled_ = false;
        double closed_phase_c2_join_min_w_ = 0.5;
        double closed_phase_c2_join_max_w_ = 1.0;
        double closed_phase_c2_join_step_w_ = 0.25;
        double closed_phase_c2_sample_step_w_ = 0.05;
        double closed_phase_back_margin_w_ = 1.0;
        bool enable_circle_reference_test_ = false;
        bool circle_reference_auto_start_ = false;
        bool circle_reference_auto_started_ = false;
        bool circle_reference_ready_ = false;
        std::string reference_shape_ = "circle";
        double circle_reference_radius_ = 4.0;
        double figure8_reference_radius_ = 4.0;
        double circle_reference_height_ = 1.0;
        int circle_reference_points_ = 240;
        int circle_reference_lookahead_pts_ = 30;
        double circle_reference_realign_min_progress_ = 3.0;

        double circle_reference_center_x_ = 0.0;
        double circle_reference_center_y_ = 0.0;
        double circle_reference_center_z_ = 0.0;
        Eigen::Vector3d circle_reference_center_ = Eigen::Vector3d::Zero();
        Eigen::MatrixXd circle_reference_traj_;
        Eigen::MatrixXd circle_reference_vel_;
        std::vector<double> circle_reference_w_;
        double circle_reference_total_w_ = 0.0;
        double circle_reference_progress_anchor_w_ = 0.0;
        int circle_reference_index_ = 0;
        double closed_ref_w_ = 0.0;
        bool closed_ref_initialized_ = false;
        bool closed_ref_recover_ = false;
        double closed_ref_search_back_w_ = 0.3;
        double closed_ref_search_forward_w_ = 1.5;
        double closed_ref_lookahead_w_ = 1.5;
        double closed_ref_lookahead_min_w_ = 1.0;
        double closed_ref_lookahead_max_w_ = 3.0;
        double closed_ref_lookahead_step_w_ = 0.5;
        bool closed_ref_enable_global_realign_ = false;
        bool closed_ref_enable_recover_ = false;
        double closed_ref_lost_radius_ = 1.5;
        double closed_ref_recover_radius_ = 1.0;
        double closed_ref_initial_phase_w_ = -1.0;
        double ref_phase_k1_ = 2.0;
        double ref_alpha_rho_ = 1.0;
        double ref_sigma_scale_ = 1.0;
        double ref_wdot_forward_max_ = 3.0;
        double ref_wdot_backward_max_ = 1.5;
        double ref_project_blend_ = 0.2;
        double ref_project_snap_max_ = 0.5;
        double ref_project_boundary_eps_ = 0.03;
        double closed_goal_full_success_tol_ = 0.3;
        double closed_goal_prefer_lookahead_w_ = 2.0;
        double closed_goal_progressive_lookahead_extra_w_ = 0.75;
        double closed_goal_progressive_min_progress_w_ = 0.6;
        double closed_goal_progressive_max_progress_w_ = 0.8;
        double closed_goal_progressive_progress_time_ = 1.0;
        double closed_goal_lookahead_weight_ = 5.0;
        double closed_goal_end_dist_weight_ = 20.0;
        bool closed_goal_push_past_obstacle_ = false;
        double closed_goal_obstacle_check_step_w_ = 0.1;
        double closed_goal_obstacle_pass_margin_w_ = 0.8;
        ros::Time closed_ref_last_update_time_;
        double closed_ref_dbg_e_parallel_ = 0.0;
        double closed_ref_dbg_rho_ = 0.0;
        double closed_ref_dbg_alpha_ = 0.0;
        double closed_ref_dbg_sigma_ = 0.0;
        double closed_ref_dbg_w_dot_ = 0.0;
        double closed_ref_dbg_dt_ = 0.0;
        double closed_ref_dbg_w_dyn_ = 0.0;
        double closed_ref_dbg_w_proj_ = 0.0;
        double closed_ref_dbg_project_delta_ = 0.0;
        double closed_ref_dbg_phase_delta_ = 0.0;
        bool closed_ref_dbg_used_project_blend_ = false;
        bool closed_ref_dbg_projected_on_boundary_ = false;
        double closed_ref_dbg_boundary_eps_ = 0.03;
        std::string closed_ref_dbg_project_blend_skipped_reason_ = "none";
        double last_selected_goal_w_ = 0.0;
        double last_selected_lookahead_w_ = 0.0;
        int last_selected_goal_idx_ = -1;
        int last_failed_goal_idx_ = -1;
        bool has_last_selected_goal_ = false;
        bool last_closed_goal_plan_success_ = true;
        double closed_ref_last_selected_goal_w_ = 0.0;
        double closed_ref_last_selected_lookahead_w_ = 0.0;
        bool closed_ref_has_selected_goal_ = false;
        double closed_ref_pending_goal_w_ = 0.0;
        double closed_ref_pending_lookahead_w_ = 0.0;
        bool closed_ref_has_pending_goal_ = false;
        bool closed_ref_pending_from_bypass_ = false;
        double closed_ref_accepted_goal_w_ = 0.0;
        double closed_ref_accepted_lookahead_w_ = 0.0;
        bool closed_ref_has_accepted_goal_ = false;
        bool closed_ref_accepted_from_bypass_ = false;
        Eigen::Vector3d closed_ref_last_goal_pos_ = Eigen::Vector3d::Zero();
        double closed_ref_last_goal_dist_xy_ = 0.0;
        int closed_ref_last_candidate_idx_ = -1;

        struct gvfManager {
            std::string index;
            std::shared_ptr<SDFMap> sdf_map_;
            std::shared_ptr<EDTEnvironment> edt_environment_;
            std::shared_ptr<AstarTopo> geo_path_finder_;
            std::shared_ptr<KinodynamicAstar> kino_path_finder_;
            std::shared_ptr<bspline_optimizer> bspline_opt_;
            std::shared_ptr<UniformBspline> spline_;
            std::shared_ptr<gvf>  gvf_;
            ros::Time curr_time;  // 当前时间定时器
            ros::Time last_time;  // 上一次时间定时器
            bool is_initialized = false; 
            bool receive_startpt = false;
            bool is_first_goal = false;  // 添加标志位
            bool receive_goal = false;
            bool is_first_kinogoal = true;
            std::vector<Eigen::Vector3d> last_path;  // 存储上一次的轨迹
            Eigen::MatrixXd last_traj;  // 存储上一次的轨迹矩阵
            Eigen::MatrixXd last_vel;  
            Eigen::VectorXd last_traj_time_;
            // ros::Subscriber odom_sub;
            Eigen::Vector3d start_pt, goal_pt, odom;

            
        };

        std::vector<gvfManager> swarmParticlesManager;

    public:
        //ROS
        ros::Publisher  force_pub;
        ros::Timer      exec_timer;
        ros::Publisher  cmd_pub;
        ros::Timer      cmd_timer;
        ros::Timer      test_cmd_timer;  // 新增测试命令定时器
        ros::Subscriber goal_sub;
        ros::Publisher  path_vis;
        ros::Subscriber odom_sub;
        ros::Publisher  path_pub; 
        ros::Publisher  kino_path_pub;  // 新增发布者
        ros::Timer      kino_timer;     // 新增定时器
        ros::Publisher  goal_vis_pub;   // 新增目标点可视化发布者
        ros::Timer      exec_fsm_timer;      // 新增 FSM 状态机定时器

        ros::Subscriber cmd_enable_sub; // 新增订阅者

        ros::Publisher  circle_ref_pub_;

    private:
        struct AuthoritativePhaseSnapshot {
            double w = 0.0;
            bool initialized = false;
            bool closed_acquired = false;
            // Synchronization-only sequence: it is neither a control state
            // nor a published diagnostic.  It prevents a stale command
            // completion from overwriting an FSM reset/initialization.
            std::uint64_t generation = 0U;
        };
        struct AuthoritativePhaseCommitToken {
            AuthoritativePhaseSnapshot expected;
            AuthoritativePhaseSnapshot committed;
            bool valid = false;
        };
        AuthoritativePhaseSnapshot captureAuthoritativePhase() const;
        void publishAuthoritativePhaseLocked(
            double w, bool initialized, bool closed_acquired);
        void publishAuthoritativePhase(
            double w, bool initialized, bool closed_acquired);
        bool commitAuthoritativePhase(
            const AuthoritativePhaseSnapshot& captured,
            double w, bool acquire_closed_phase,
            AuthoritativePhaseSnapshot& committed);
        bool prepareAuthoritativePhaseCommitLocked(
            const AuthoritativePhaseSnapshot& captured,
            double w, bool acquire_closed_phase,
            AuthoritativePhaseCommitToken& token) const;
        void commitAuthoritativePhaseNoFailLocked(
            const AuthoritativePhaseCommitToken& token) noexcept;
        struct CallbackTimingSample {
            std::uint64_t sequence = 0U;
            std::uint64_t steady_duration_ns = 0U;
            std::uint64_t ros_stamp_ns = 0U;
        };
        std::unique_ptr<PhaseOffsetMatchedAdapter> phase_offset_matched_adapter_;
        std::unique_ptr<PhaseOffsetShadowAdapter> phase_offset_shadow_adapter_;
        phase_offset_navigation::RecoveryStepStatus last_recovery_status_ =
            phase_offset_navigation::RecoveryStepStatus::NONE;
        std::mutex frontend_apply_mutex_;
        mutable std::mutex path_reference_handoff_mutex_;
        // One immutable successor slot.  Before publication it carries the
        // request with a null profile; after the publish-first commit the
        // same slot carries the enriched binding/mirror until FSM applies the
        // display payload.  It is not a READY latch or a second lifecycle.
        std::shared_ptr<const PathReferenceHandoffV2>
            pending_path_reference_handoff_v2_;
        // Monotonic process-lifetime identities.  Task reset deliberately
        // does not rewind them, preventing stale successor-key reuse.
        std::uint64_t next_tube_request_id_v2_ = 0U;
        std::uint64_t next_path_identity_v2_ = 0U;
        // The 10 Hz timer owns coherent map capture and CURRENT request
        // construction.  The command callback copies only this immutable
        // owner plus bounded accepted-state visibility metadata.
        mutable std::mutex current_tube_request_mutex_v2_;
        std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2>
            current_tube_request_v2_;
        // This mutex protects exactly {phase_w_, phase_initialized_,
        // closed_phase_acquired_, generation}.  Never hold it across path
        // construction, A*, tube construction, publication, or Runtime work.
        mutable std::mutex authoritative_phase_mutex_;
        std::uint64_t authoritative_phase_generation_ = 0U;
        // S3: MANUAL tube construction/visualization only.  The 50 Hz command
        // callback never calls TubeEpochManager.
        ros::Timer phase_offset_tube_timer_;
        PhaseOffsetMatchedAdapterConfig matched_config_;
        // Exactly one main-side coordination selector is effective for this
        // manager.  Provider/transport flags are validated against it before
        // any SPH bridge is constructed; invalid combinations fail closed to
        // DISABLED and leave the legacy complete-g_des APIs untouched.
        PhaseOffsetCoordinationBackend coordination_backend_ =
            PhaseOffsetCoordinationBackend::D1B;
        bool coordination_backend_config_valid_ = true;
        bool enable_neighbor_transport_ = false;
        bool enable_sph_provider_ = false;
        int phase_offset_robot_id_ = -1;
        std::unique_ptr<bspline_race::integration::PhaseOffsetSphRosBridge>
            phase_offset_sph_ros_bridge_;
        // Value-semantic boundary from the parallel interaction workstream.
        // No producer is connected in the single-UAV baseline, so the
        // adapter's frozen recenter-only fallback remains authoritative.
        mutable std::mutex phase_offset_g_des_mutex_;
        Eigen::Vector3d phase_offset_g_des_ = Eigen::Vector3d::Zero();
        bool phase_offset_g_des_valid_ = false;
        bool phase_offset_measurement_callback_enabled_ = false;
        std::string phase_offset_measurement_callback_csv_path_;
        std::mutex phase_offset_measurement_callback_mutex_;
        std::vector<CallbackTimingSample> phase_offset_measurement_callback_samples_;
        std::uint64_t phase_offset_measurement_callback_sequence_ = 0U;
        int test_traj_index_;  // 测试轨迹执行索引
        bool use_test_cmd_;    // 是否使用测试命令模式
        bool enable_gvfcmd_control; //标记是否收到 gvf 控制指令

        enum FSM_EXEC_STATE { INIT, WAIT_TARGET, GEN_NEW_TRAJ, REPLAN_TRAJ, EXEC_TRAJ };
        FSM_EXEC_STATE exec_state_;

        struct KinoPlanSamples {
            double ts = 0.2;
            std::vector<Eigen::Vector3d> point_set;
            std::vector<Eigen::Vector3d> start_end_derivatives;
        };

        struct PhasePathState {
            Eigen::Vector3d p = Eigen::Vector3d::Zero();
            Eigen::Vector3d dp_dw = Eigen::Vector3d::Zero();
            Eigen::Vector3d d2p_dw2 = Eigen::Vector3d::Zero();
            Eigen::Vector3d vel = Eigen::Vector3d::Zero();
            bool valid = false;
        };

        struct GovernorCandidate {
            bool have = false;
            double cost = 0.0;
            double l = 0.0;
            double query_w = 0.0;
            double path_w_start = 0.0;
            double path_w_end = 0.0;
            Eigen::Vector3d cmd = Eigen::Vector3d::Zero();
            Eigen::Vector3d n = Eigen::Vector3d::Zero();
            Eigen::Vector3d n_raw = Eigen::Vector3d::Zero();
            Eigen::Vector3d base_delta = Eigen::Vector3d::Zero();
            Eigen::Vector3d v_model = Eigen::Vector3d::Zero();
            double vel_cost = 0.0;
            double l_ff_cost = 0.0;
            double l_rate_cost = 0.0;
            double normal_cost = 0.0;
            double normal_rate_cost = 0.0;
            double tau_vel_error = 0.0;
            double normal_vel_error_norm = 0.0;
            bool normal_vel_error_capped = false;
            bool normal_rate_limited = false;
        };

        struct GovernorCommandDebug {
            bool guidance_valid = false;
            bool fallback_hold_pos = false;
            bool final_cmd_overridden = false;
            bool state_reset_due_to_override = false;
            bool state_reset_due_to_lead_limit = false;
            bool normal_rate_limited = false;
            bool lead_limit_violation = false;
            bool normal_vel_error_capped = false;
            bool best_clamped_to_end = false;
            int candidate_count = 0;
            int valid_count = 0;
            int path_end_clamped_count = 0;
            int skipped_lead_count = 0;
            double raw_v_norm = 0.0;
            double raw_v_tau = 0.0;
            double raw_v_normal_norm = 0.0;
            double v_tau_intent = 0.0;
            double v_n_intent_norm = 0.0;
            double l_ff = 0.0;
            double best_l = 0.0;
            double best_query_w = 0.0;
            double best_cost = 0.0;
            double base_delta_norm = 0.0;
            double normal_raw_norm = 0.0;
            double normal_state_norm = 0.0;
            double active_normal_max = 0.0;
            double selected_v_model_norm = 0.0;
            double vel_error_norm = 0.0;
            double tau_vel_error = 0.0;
            double normal_vel_error_norm = 0.0;
            double vel_cost = 0.0;
            double l_ff_cost = 0.0;
            double l_rate_cost = 0.0;
            double normal_cost = 0.0;
            double normal_rate_cost = 0.0;
            double cmd_dist = 0.0;
            double path_w_start = 0.0;
            double path_w_end = 0.0;
            double cmd_delta_rate = 0.0;
            double estimated_acc = 0.0;
            double e_perp_norm = 0.0;
        };

        struct GovernorCommandResult {
            Eigen::Vector3d cmd_pos = Eigen::Vector3d::Zero();
            Eigen::Vector3d yaw_cmd_vec = Eigen::Vector3d::Zero();
            Eigen::Vector3d selected_n = Eigen::Vector3d::Zero();
            double selected_l = 0.0;
            bool command_valid = false;
            bool selected_valid_for_state = false;
            bool reset_state_after_publish = false;
            std::string final_cmd_source = "GOVERNOR_INVALID_HOLD";
            std::string fallback_reason = "none";
        };

        bool planKinoToGoal(gvfManager& pm,
                            const Eigen::Vector3d& start_pt,
                            const Eigen::Vector3d& start_vel,
                            const Eigen::Vector3d& start_acc,
                            const Eigen::Vector3d& goal_pt,
                            const Eigen::Vector3d& end_vel,
                            KinoPlanSamples& samples);
        bool selectClosedGoalCandidate(gvfManager& pm,
                                       const Eigen::Vector3d& curr_pos,
                                       const Eigen::Vector3d& start_pt,
                                       const Eigen::Vector3d& start_vel,
                                       const Eigen::Vector3d& start_acc,
                                       Eigen::Vector3d& goal_pt,
                                       Eigen::Vector3d& end_vel,
                                       KinoPlanSamples& samples);
        bool selectClosedPhaseV2Goal(gvfManager& pm,
                                     const Eigen::Vector3d& curr_pos,
                                     const Eigen::Vector3d& start_pt,
                                     const Eigen::Vector3d& start_vel,
                                     const Eigen::Vector3d& start_acc,
                                     Eigen::Vector3d& goal_pt,
                                     Eigen::Vector3d& end_vel,
                                     KinoPlanSamples& samples);
        bool evaluateSampledPhasePathState(const Eigen::MatrixXd& traj,
                                           const Eigen::MatrixXd& vel,
                                           const std::vector<double>& global_w,
                                           double query_w,
                                           PhasePathState& state) const;
        bool buildPhaseV2C2Frontend(gvfManager& pm,
                                    double phase_at_switch,
                                    double future_switch_w,
                                    const std::shared_ptr<const ContinuousPhasePath>&
                                        old_path_owner,
                                    double path_end_w,
                                    int candidate_anchor_idx,
                                    const UniformBspline& candidate_spline,
                                    const Eigen::MatrixXd& candidate_traj,
                                    const Eigen::MatrixXd& candidate_vel,
                                    const Eigen::VectorXd& candidate_time,
                                    const std::vector<double>& candidate_w,
                                    Eigen::MatrixXd& stitched_traj,
                                    Eigen::MatrixXd& stitched_vel,
                                    Eigen::VectorXd& stitched_time,
                                    std::vector<double>& stitched_w,
                                    std::shared_ptr<const ContinuousPhasePath>& continuous_path) const;
        static bool plannerOnlyFutureSeamV2(
            const std::shared_ptr<const ContinuousPhasePath>& source_path,
            double phase_at_switch,
            double construction_lead_w,
            double& seam_w);
        bool stageFutureSeamPathReferenceHandoffV2(
            const AuthoritativePhaseSnapshot& phase_after,
            gvfManager& pm,
            double path_end_w,
            int candidate_anchor_idx,
            const UniformBspline& candidate_spline,
            const Eigen::MatrixXd& candidate_traj,
            const Eigen::MatrixXd& candidate_vel,
            const Eigen::VectorXd& candidate_time,
            const std::vector<double>& candidate_w,
            Eigen::MatrixXd& staged_traj,
            Eigen::MatrixXd& staged_vel,
            Eigen::VectorXd& staged_time,
            std::vector<double>& staged_w,
            std::shared_ptr<const ContinuousPhasePath>& staged_path);
        bool assignPathIdentityV2(
            const std::shared_ptr<const ContinuousPhasePath>& candidate,
            std::shared_ptr<const ContinuousPhasePath>& assigned);
        bool buildCurrentTubeRequestV2(
            gvfManager& pm,
            const AuthoritativePhaseSnapshot& phase,
            std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2>&
                request);
        bool captureCurrentTubeCommandEvidenceV2(
            gvfManager& pm,
            const std::shared_ptr<const ContinuousPhasePath>& command_path,
            double current_w,
            MatchedAdapterInput& input) const;
        bool installPlannerOnlyFrontendV2(
            gvfManager& pm,
            const Eigen::MatrixXd& traj,
            const Eigen::MatrixXd& vel,
            const Eigen::VectorXd& time,
            const std::vector<double>& w,
            const std::shared_ptr<const ContinuousPhasePath>& path_owner,
            const ros::Time& now,
            const AuthoritativePhaseSnapshot& expected_phase,
            const std::shared_ptr<const ContinuousPhasePath>&
                copied_prefix_source,
            double copied_prefix_end_w,
            std::uint64_t expected_execution_generation,
            nav_msgs::Path& path_msg);
        bool consumeCommittedPathReferenceHandoffV2(
            gvfManager& pm, const ros::Time& now);
        std::shared_ptr<const ContinuousPhasePath>
            captureCommandPathForV2Binding(
                const gvfManager& pm,
                const std::shared_ptr<const TubeV2ExecutionBinding>&
                    binding);
        bool installAuthoritativePathMirrorLocked(
            const Eigen::MatrixXd& traj,
            const Eigen::MatrixXd& vel,
            const std::vector<double>& w,
            nav_msgs::Path& path_msg);
        bool buildMappedPhaseFrontend(
            double phase_anchor,
            double path_end_w,
            bool trim_start_endpoint,
            int candidate_anchor_idx,
            const UniformBspline& candidate_spline,
            const Eigen::MatrixXd& candidate_traj,
            const Eigen::VectorXd& candidate_time,
            Eigen::MatrixXd& mapped_traj,
            Eigen::MatrixXd& mapped_vel,
            Eigen::VectorXd& mapped_time,
            std::vector<double>& mapped_w,
            std::shared_ptr<const ContinuousPhasePath>& continuous_path) const;
        bool buildNominalContinuousPhasePath(
            double start_w,
            double end_w,
            std::shared_ptr<const ContinuousPhasePath>& path) const;
        bool sampleContinuousPhasePath(
            const std::shared_ptr<const ContinuousPhasePath>& path,
            Eigen::MatrixXd& traj,
            Eigen::MatrixXd& vel,
            Eigen::VectorXd& time,
            std::vector<double>& global_w) const;
        friend class GvfManagerS4AnchorTestAccess;
        bool pathPointAtW(const std::shared_ptr<const ContinuousPhasePath>& path,
                          double query_w,
                          double reference_delta,
                          Eigen::Vector3d& point,
                          bool& clamped_to_end,
                          double& path_w_start,
                          double& path_w_end) const;
        bool pathTangentAtW(const std::shared_ptr<const ContinuousPhasePath>& path,
                            double query_w,
                            double reference_delta,
                            Eigen::Vector3d& tangent) const;
        void resetGovernorState();
        void clearActiveTrajectory(bool publish_empty_path,
                                   bool clear_gvf_path = true);
        GovernorCommandResult makeGovernorInvalidHold(const Eigen::Vector3d& pos,
                                                      const std::string& reason,
                                                      GovernorCommandDebug& dbg);
        GovernorCommandResult runVelocityMatchingGovernor(
                                                          const std::shared_ptr<const ContinuousPhasePath>& path,
                                                          const gvf::LiftedGuidanceResult& out,
                                                          const Eigen::Vector3d& pos,
                                                          double progress_w_after,
                                                          double reference_delta,
                                                          double dt,
                                                          double kp_equiv,
                                                          GovernorCommandDebug& dbg,
                                                          const phase_offset_navigation::ImmutableExecutedReferenceQueryPtr&
                                                              executed_reference_query =
                                                                  phase_offset_navigation::ImmutableExecutedReferenceQueryPtr());
        void updateGovernorCommandHistory(const Eigen::Vector3d& pos,
                                          const Eigen::Vector3d& cmd_pos,
                                          double dt,
                                          GovernorCommandDebug& dbg);
        void logGovernorCommand(const GovernorCommandResult& result,
                                const GovernorCommandDebug& dbg,
                                const Eigen::Vector3d& cmd_pos,
                                double real_dis_to_goal,
                                double kp_equiv,
                                bool switch_active) const;
        bool publishGovernorPositionCommand(const Eigen::Vector3d& cmd_pos,
                                            const Eigen::Vector3d& yaw_cmd_vec);
        void recordCallbackTiming(std::uint64_t steady_duration_ns,
                                  std::uint64_t ros_stamp_ns);
        void flushCallbackTiming();

    public:
        gvf_manager(){};  
        gvf_manager(ros::NodeHandle& nh); 
        ~gvf_manager();
        void initCallback(ros::NodeHandle &nh);
        void InitGvf(ros::NodeHandle &nh);
        void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg); 
        void execTimerCallback(const ros::TimerEvent& event);

        bool astaropt(const Eigen::Vector3d& curr_pos, Eigen::MatrixXd& pos_out, Eigen::MatrixXd& vel_out,
                      int& new_i0_out, Eigen::VectorXd& time,
                      UniformBspline* continuous_spline_out = nullptr);
                      
        bool checkCollision();
        void cmdCallback(const ros::TimerEvent& event);
        void phaseOffsetTubeTimerCallback(const ros::TimerEvent& event);
        // Value-semantic NORMAL workspace-intent seam. An upstream producer
        // may publish one final Eigen::Vector3d per control tick; clearing the
        // value returns the adapter to its frozen single-UAV recenter fallback.
        bool setPhaseOffsetGDes(const Eigen::Vector3d& g_des);
        void clearPhaseOffsetGDes();
        bool capturePhaseOffsetGDes(Eigen::Vector3d& g_des) const;
        void test_cmdCallback(const ros::TimerEvent& event);  // 新增测试命令回调函数
        void KinoPathCallback(const ros::TimerEvent& event);  // 新增回调函数
        void publishCorridorMarker(double C_thresh = -1.0);
        void visualizePath(const std::vector<Eigen::Vector3d>& path_points, 
                            ros::Publisher& marker_pub, const std::string& particle_index);
        std::vector<Eigen::Vector3d> correctPathToCenter(
                const std::vector<Eigen::Vector3d>& raw_path);
                
        void cmdEnableCallback(const std_msgs::Bool::ConstPtr& msg);  // 新增：回调函数声明  

        void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
        void FSMCallback(const ros::TimerEvent& event);
        double cul_score(const Eigen::MatrixXd& traj, const Eigen::MatrixXd& vel, const Eigen::Vector3d& goal_pt, int i0,
                         const Eigen::VectorXd& time);

                // --------- Replan helpers (candidate + switch decision) ---------
        bool shouldAcceptCandidate(const Eigen::MatrixXd& old_traj, const Eigen::MatrixXd& old_vel, const Eigen::VectorXd& old_time, int old_i0,
                        const Eigen::MatrixXd& new_traj, const Eigen::MatrixXd& new_vel, const Eigen::VectorXd& new_time, int new_i0,
                        const Eigen::Vector3d& goal_pt, std::string& reason_out,
                        double accepted_progress_w, double accepted_path_w_end);
        static bool shouldForceAcceptForGovernorPathShort(double progress_w,
                                                          double path_w_end,
                                                          double governor_l_max,
                                                          double margin_w)
        {
            if (!std::isfinite(progress_w) || !std::isfinite(path_w_end)) {
                return false;
            }
            const double remaining_w = path_w_end - progress_w;
            const double required_w = std::max(0.0, governor_l_max) + std::max(0.0, margin_w);
            return remaining_w <= required_w;
        }
        static Eigen::Vector3d boundedInitialAcquisitionDelta(
            const Eigen::Vector3d& guidance_velocity,
            double velocity_limit,
            double position_gain,
            double lead_limit)
        {
            if (!guidance_velocity.allFinite() ||
                !std::isfinite(velocity_limit) ||
                !std::isfinite(position_gain) ||
                !std::isfinite(lead_limit) || position_gain <= 0.0) {
                return Eigen::Vector3d::Zero();
            }

            Eigen::Vector3d velocity = guidance_velocity;
            const double safe_velocity_limit = std::max(0.0, velocity_limit);
            if (safe_velocity_limit > 1e-9 &&
                velocity.norm() > safe_velocity_limit) {
                velocity *= safe_velocity_limit / velocity.norm();
            }

            Eigen::Vector3d delta = velocity / position_gain;
            const double safe_lead_limit = std::max(0.0, lead_limit);
            if (safe_lead_limit > 1e-9 && delta.norm() > safe_lead_limit) {
                delta *= safe_lead_limit / delta.norm();
            }
            return delta;
        }
        // The authoritative v2 phase is a transaction: form a candidate before
        // command selection, then publish it only after a final governor command
        // has survived validation and the goal-position override.  Initial
        // closed-phase acquisition moves the vehicle, but is not that port.
        struct AuthoritativePhaseCommitDecision
        {
            bool commit = false;
            bool acquire_closed_phase = false;
            double phase_after = 0.0;
            double applied_delta_w = 0.0;
        };
        static double clampPointPhaseCandidate(double requested_phase,
                                               double path_start_w,
                                               double path_end_w)
        {
            if (path_end_w <= path_start_w + 1e-6) {
                return requested_phase;
            }
            return std::max(path_start_w,
                            std::min(requested_phase, path_end_w));
        }
        static AuthoritativePhaseCommitDecision decideAuthoritativePhaseCommit(
            bool have_phase_candidate,
            bool command_valid,
            bool force_goal_position,
            bool initial_closed_phase_acquisition_used,
            double phase_before,
            double phase_candidate,
            bool closed_phase_acquired_before,
            bool will_acquire_closed_phase)
        {
            AuthoritativePhaseCommitDecision decision;
            decision.phase_after = phase_before;
            decision.commit = have_phase_candidate && command_valid &&
                              !force_goal_position &&
                              !initial_closed_phase_acquisition_used;
            if (decision.commit) {
                decision.phase_after = phase_candidate;
                decision.applied_delta_w = phase_candidate - phase_before;
                decision.acquire_closed_phase =
                    !closed_phase_acquired_before && will_acquire_closed_phase;
            }
            return decision;
        }
        static bool shouldRunInitialClosedPhaseAcquisition(
            bool closed_phase_active,
            bool closed_phase_acquired_before,
            bool will_acquire_closed_phase)
        {
            return closed_phase_active && !closed_phase_acquired_before &&
                   !will_acquire_closed_phase;
        }
        static bool shouldDeclarePointGoalReached(bool circle_mode_active,
                                                  double dist_xy,
                                                  double final_tolerance = 0.2)
        {
            if (circle_mode_active || !std::isfinite(dist_xy) ||
                !std::isfinite(final_tolerance) || dist_xy < 0.0 ||
                final_tolerance <= 0.0) {
                return false;
            }
            return dist_xy < final_tolerance;
        }
        enum class ReplanTriggerDecision
        {
            NONE = 0,
            COLLISION,
            PERIODIC
        };
        static ReplanTriggerDecision selectReplanTrigger(
            bool collision_detected,
            bool periodic_due,
            bool periodic_phase_ready,
            bool terminal_nonzero_retry_suppressed)
        {
            // The H2 terminal latch suppresses duplicate periodic attempts
            // only.  A newly observed collision is a higher-priority safety
            // event and must always remain able to request replanning.
            if (collision_detected) return ReplanTriggerDecision::COLLISION;
            if (periodic_due && periodic_phase_ready &&
                !terminal_nonzero_retry_suppressed) {
                return ReplanTriggerDecision::PERIODIC;
            }
            return ReplanTriggerDecision::NONE;
        }
        struct ClosedGoalCandidateProgress
        {
            bool valid = false;
            bool passed_obstacle = false;
            double end_delta_w = 0.0;
            double end_to_goal_dist = std::numeric_limits<double>::infinity();
            double lookahead = 0.0;
        };

        struct ClosedGoalProgressiveCandidate
        {
            bool valid = false;
            int idx = -1;
            double lookahead = 0.0;
            double end_delta_w = 0.0;
            double kino_path_length = std::numeric_limits<double>::infinity();
            double end_to_goal_dist = std::numeric_limits<double>::infinity();
        };

        static double closedGoalRequiredProgress(double speed_xy,
                                                 double progress_time,
                                                 double min_progress,
                                                 double max_progress)
        {
            const double lo = std::max(0.0, std::min(min_progress, max_progress));
            const double hi = std::max(lo, std::max(min_progress, max_progress));
            const double raw = std::max(0.0, speed_xy) * std::max(0.0, progress_time);
            return std::max(lo, std::min(raw, hi));
        }

        static bool closedGoalProgressSufficient(double end_delta_w,
                                                 double required_progress)
        {
            return std::isfinite(end_delta_w) && std::isfinite(required_progress) &&
                   end_delta_w >= required_progress;
        }

        static double closedPhaseV2SelectedDeltaW(double base_delta_w,
                                                  double max_delta_w,
                                                  bool obstacle_end_found,
                                                  double obstacle_end_delta_w,
                                                  double pass_margin_w)
        {
            const double hi = std::max(0.0, max_delta_w);
            double selected = std::max(0.0, std::min(base_delta_w, hi));
            if (obstacle_end_found && std::isfinite(obstacle_end_delta_w)) {
                selected = std::max(
                    selected,
                    obstacle_end_delta_w + std::max(0.0, pass_margin_w));
            }
            return std::min(selected, hi);
        }

        static bool evaluateC2QuinticHermite(
            const Eigen::Vector3d& p0,
            const Eigen::Vector3d& dp0,
            const Eigen::Vector3d& d2p0,
            const Eigen::Vector3d& p1,
            const Eigen::Vector3d& dp1,
            const Eigen::Vector3d& d2p1,
            double interval_w,
            double u,
            Eigen::Vector3d& p,
            Eigen::Vector3d& dp_dw,
            Eigen::Vector3d& d2p_dw2);

        static double closedGoalProgressiveMaxLookahead(double preferred,
                                                        double configured_max,
                                                        double extra,
                                                        bool recover_mode)
        {
            const double hi = std::max(0.0, configured_max);
            if (recover_mode) return hi;
            return std::min(hi, std::max(0.0, preferred) + std::max(0.0, extra));
        }

        static bool closedGoalAttemptComesBefore(double lhs_lookahead,
                                                 int lhs_idx,
                                                 double rhs_lookahead,
                                                 int rhs_idx,
                                                 double desired_lookahead)
        {
            const double safe_desired = std::isfinite(desired_lookahead)
                ? desired_lookahead
                : 0.0;
            const double safe_lhs = std::isfinite(lhs_lookahead)
                ? lhs_lookahead
                : std::numeric_limits<double>::infinity();
            const double safe_rhs = std::isfinite(rhs_lookahead)
                ? rhs_lookahead
                : std::numeric_limits<double>::infinity();
            const double lhs_error = std::abs(safe_lhs - safe_desired);
            const double rhs_error = std::abs(safe_rhs - safe_desired);
            if (lhs_error != rhs_error) return lhs_error < rhs_error;
            if (safe_lhs != safe_rhs) return safe_lhs < safe_rhs;
            return lhs_idx < rhs_idx;
        }

        static const char* closedGoalCandidateOrderReason(bool recover_mode)
        {
            return recover_mode ? "recover_full_window" : "track_progressive_window";
        }

        static bool preferClosedGoalProgressiveCandidate(
            const ClosedGoalProgressiveCandidate& lhs,
            const ClosedGoalProgressiveCandidate& rhs,
            double required_progress,
            double desired_lookahead)
        {
            const auto is_valid_candidate = [](const ClosedGoalProgressiveCandidate& value) {
                return value.valid && std::isfinite(value.lookahead) &&
                       std::isfinite(value.end_delta_w) &&
                       std::isfinite(value.kino_path_length) &&
                       std::isfinite(value.end_to_goal_dist);
            };
            const bool lhs_valid = is_valid_candidate(lhs);
            const bool rhs_valid = is_valid_candidate(rhs);
            if (lhs_valid != rhs_valid) return lhs_valid;
            if (!lhs_valid) return false;

            const double safe_desired_lookahead = std::isfinite(desired_lookahead)
                ? desired_lookahead
                : 0.0;
            const bool lhs_sufficient = closedGoalProgressSufficient(
                lhs.end_delta_w, required_progress);
            const bool rhs_sufficient = closedGoalProgressSufficient(
                rhs.end_delta_w, required_progress);
            if (lhs_sufficient != rhs_sufficient) return lhs_sufficient;

            if (lhs_sufficient) {
                const double lhs_error = std::abs(lhs.lookahead - safe_desired_lookahead);
                const double rhs_error = std::abs(rhs.lookahead - safe_desired_lookahead);
                if (lhs_error != rhs_error) return lhs_error < rhs_error;
                if (lhs.kino_path_length != rhs.kino_path_length)
                    return lhs.kino_path_length < rhs.kino_path_length;
                if (lhs.lookahead != rhs.lookahead)
                    return lhs.lookahead < rhs.lookahead;
            } else {
                if (lhs.end_delta_w != rhs.end_delta_w)
                    return lhs.end_delta_w > rhs.end_delta_w;
                if (lhs.kino_path_length != rhs.kino_path_length)
                    return lhs.kino_path_length < rhs.kino_path_length;
                if (lhs.lookahead != rhs.lookahead)
                    return lhs.lookahead < rhs.lookahead;
            }

            if (lhs.end_to_goal_dist != rhs.end_to_goal_dist)
                return lhs.end_to_goal_dist < rhs.end_to_goal_dist;
            return lhs.idx < rhs.idx;
        }

        struct ClosedGoalObstacleInterval
        {
            bool found_start = false;
            bool found_end = false;
            double start_delta_w = std::numeric_limits<double>::infinity();
            double end_delta_w = std::numeric_limits<double>::infinity();
        };

        static ClosedGoalObstacleInterval detectClosedGoalObstacleInterval(
            const std::vector<int>& occupancy,
            double step_w,
            int required_free_samples = 3)
        {
            ClosedGoalObstacleInterval result;
            if (!std::isfinite(step_w) || step_w <= 0.0 || required_free_samples <= 0) {
                return result;
            }

            int free_count = 0;
            double free_run_start = std::numeric_limits<double>::infinity();
            for (size_t i = 0; i < occupancy.size(); ++i) {
                const double delta_w = (static_cast<double>(i) + 1.0) * step_w;
                if (occupancy[i] < 0) {
                    continue;
                }
                if (!result.found_start) {
                    if (occupancy[i] != 0) {
                        result.found_start = true;
                        result.start_delta_w = delta_w;
                    }
                    continue;
                }
                if (occupancy[i] != 0) {
                    free_count = 0;
                    free_run_start = std::numeric_limits<double>::infinity();
                    continue;
                }
                if (free_count == 0) {
                    free_run_start = delta_w;
                }
                ++free_count;
                if (free_count >= required_free_samples) {
                    result.found_end = true;
                    result.end_delta_w = free_run_start;
                    break;
                }
            }
            return result;
        }

        static bool preferClosedGoalBypassCandidate(
            const ClosedGoalCandidateProgress& lhs,
            const ClosedGoalCandidateProgress& rhs,
            bool bypass_mode)
        {
            if (!bypass_mode) {
                return false;
            }
            if (lhs.valid != rhs.valid) {
                return lhs.valid;
            }
            if (!lhs.valid) {
                return false;
            }
            if (lhs.passed_obstacle != rhs.passed_obstacle) {
                return lhs.passed_obstacle;
            }

            constexpr double eps = 1e-6;
            if (std::abs(lhs.end_delta_w - rhs.end_delta_w) > eps) {
                return lhs.passed_obstacle ? lhs.end_delta_w < rhs.end_delta_w
                                           : lhs.end_delta_w > rhs.end_delta_w;
            }
            if (std::abs(lhs.end_to_goal_dist - rhs.end_to_goal_dist) > eps) {
                return lhs.end_to_goal_dist < rhs.end_to_goal_dist;
            }
            return lhs.lookahead < rhs.lookahead - eps;
        }
        static double closedGoalDesiredLookahead(double configured_lookahead,
                                                 bool has_accepted_goal,
                                                 double accepted_lookahead,
                                                 bool accepted_from_bypass)
        {
            return has_accepted_goal && !accepted_from_bypass
                ? accepted_lookahead
                : configured_lookahead;
        }
        static bool isFiniteClosedGoalCandidate(double end_x,
                                                double end_y,
                                                double end_z,
                                                double end_to_goal_dist,
                                                double end_delta_w)
        {
            return std::isfinite(end_x) && std::isfinite(end_y) &&
                   std::isfinite(end_z) && std::isfinite(end_to_goal_dist) &&
                   std::isfinite(end_delta_w);
        }
        static double closedGoalCandidateScore(double lookahead,
                                               double desired_lookahead,
                                               double end_to_goal_dist,
                                               double lookahead_weight,
                                               double end_dist_weight)
        {
            if (!std::isfinite(lookahead) ||
                !std::isfinite(desired_lookahead) ||
                !std::isfinite(end_to_goal_dist)) {
                return std::numeric_limits<double>::infinity();
            }
            return std::max(0.0, lookahead_weight) * std::abs(lookahead - desired_lookahead) +
                   std::max(0.0, end_dist_weight) * std::max(0.0, end_to_goal_dist);
        }
        static double closedGoalObstaclePushedLookahead(double desired_lookahead,
                                                        double obstacle_delta_w,
                                                        double min_lookahead,
                                                        double max_lookahead,
                                                        double pass_margin_w)
        {
            const double lo = std::max(0.0, std::min(min_lookahead, max_lookahead));
            const double hi = std::max(lo, std::max(min_lookahead, max_lookahead));
            double desired = std::isfinite(desired_lookahead) ? desired_lookahead : lo;
            desired = std::max(lo, std::min(desired, hi));

            if (!std::isfinite(obstacle_delta_w)) {
                return desired;
            }

            const double pushed = std::max(desired, obstacle_delta_w + std::max(0.0, pass_margin_w));
            return std::max(lo, std::min(pushed, hi));
        }
        void publishPathMsg(
            const Eigen::MatrixXd& traj,
            const Eigen::MatrixXd& vel,
            const std::vector<double>* authoritative_w_samples = nullptr);
        void publishReferencePathMsg(const Eigen::MatrixXd& traj, const Eigen::MatrixXd& vel, ros::Publisher& pub);
        bool closedPhaseV2Enabled() const;
        bool closedPhaseV2Active() const;
        bool pointPhaseV2Active() const;
        bool unifiedPhaseV2Active() const;
        double activeTrackingPhase() const;
        // User-goal boundary: unlike same-goal H2 retirement it also asks
        // the adapter to neutralize path-coordinate Runtime history.
        bool resetForNewNavigationTask();
        double findInitialClosedPhaseV2(const Eigen::Vector3d& curr_pos) const;
        bool buildNominalClosedFrontend(double start_w,
                                        Eigen::MatrixXd& traj,
                                        Eigen::MatrixXd& vel,
                                        Eigen::VectorXd& time,
                                        std::vector<double>& global_w) const;
        bool buildGlobalPhaseSamples(const Eigen::MatrixXd& traj,
                                     int anchor_idx,
                                     double anchor_w,
                                     double end_w,
                                     std::vector<double>& global_w) const;
        static bool buildArcLengthPhaseSamples(const Eigen::MatrixXd& traj,
                                               int anchor_idx,
                                               double anchor_w,
                                               std::vector<double>& global_w);
        bool installInitialClosedPhaseFrontend(gvfManager& pm,
                                               const Eigen::Vector3d& current_pos,
                                               const ros::Time& current_time);
        bool installInitialPointPhaseFrontend(gvfManager& pm,
                                              const Eigen::Vector3d& current_pos,
                                              const ros::Time& current_time,
                                              const Eigen::MatrixXd& candidate_traj,
                                              const Eigen::MatrixXd& candidate_vel,
                                              const Eigen::VectorXd& candidate_time,
                                              int candidate_anchor_idx,
                                              const UniformBspline& candidate_spline);
        void generateCircleReference(const Eigen::Vector3d& center);
        void generateFigureEightReference(const Eigen::Vector3d& center);
        std::pair<Eigen::Vector3d, Eigen::Vector3d> getCircleReferenceGoal(const Eigen::Vector3d& curr_pos);
        double wrapClosedW(double w) const;
        int indexFromClosedW(double w) const;
        Eigen::Vector3d pointFromClosedW(double w) const;
        Eigen::Vector3d tangentFromClosedW(double w) const;
        double findInitialClosedPhase(const Eigen::Vector3d& curr_pos) const;
        double projectClosedLocal(const Eigen::Vector3d& curr_pos, double w_prev,
                                  double back_window, double forward_window) const;
        double closedRefAlpha(double rho) const;
        double closedRefSigma(double e_parallel) const;
        double updateClosedRefPhaseByDynamics(const Eigen::Vector3d& curr_pos, double dt);
        std::vector<double> buildClosedLookaheadCandidates() const;
        static int selectDefaultClosedLookaheadIndex(
            const std::vector<double>& candidates,
            double desired_lookahead)
        {
            if (candidates.empty()) {
                return -1;
            }

            const double default_lookahead = std::min(
                std::max(desired_lookahead, candidates.front()), candidates.back());
            int best_idx = 0;
            double best_diff = std::abs(candidates[0] - default_lookahead);
            for (int i = 1; i < static_cast<int>(candidates.size()); ++i) {
                const double diff = std::abs(candidates[i] - default_lookahead);
                if (diff < best_diff) {
                    best_diff = diff;
                    best_idx = i;
                }
            }
            return best_idx;
        }
        void resetClosedGoalCandidateState();
        void ensureProgressInCurrentPathRange(double start_w, double end_w);
        void logReplanReason(const std::string& reason);

        //inline func 
        inline Eigen::Vector3d esdfGrad(const Eigen::Vector3d& p) const
        {
            Eigen::Vector3d grad;
            swarmParticlesManager[0]
                .sdf_map_->getDistWithGradTrilinear(p, grad);   // 距离返回值可忽略
            return grad;                                        // 单位: m
        }

    inline std::pair<double, double> cal_yaw( double current_yaw,double aim_yaw)
    {
    std::pair<double, double> yaw_yawdot(0, 0);
    if(current_yaw<0)                 current_yaw = current_yaw + 2*PI;
    else if(current_yaw>2*PI)  current_yaw = current_yaw - 2*PI;
        if(aim_yaw<0)                 aim_yaw = aim_yaw + 2*PI;
    else if(aim_yaw>2*PI)    aim_yaw = aim_yaw - 2*PI;
    double yaw_distance = aim_yaw - current_yaw;
    double sign_        = yaw_distance / fabs(yaw_distance);
    if(fabs(yaw_distance) < YAW_MAX )
    {cout<<"ca1"<<endl;
        output_yaw   = aim_yaw;
        output_d_yaw = yaw_distance / delta_T;
    }
    else
    {cout<<"ca2"<<endl;
        output_yaw = current_yaw + sign_ * YAW_MAX;
        output_d_yaw = sign_*D_YAW_MAX;
    }
    yaw_yawdot.first = output_yaw;
    yaw_yawdot.second = output_d_yaw;
    return yaw_yawdot;
    }
    
    inline std::pair<double, double> calculate_yaw( double current_yaw,double aim_yaw)
    {
    std::pair<double, double> yaw_yawdot(0, 0);
    double yaw_ = 0;
    double yawdot = 0;
    if (aim_yaw - current_yaw > PI)
    {
        
        if (aim_yaw - current_yaw - 2 * PI < -YAW_MAX)
        {
        yaw_ = current_yaw - YAW_MAX;
        if (yaw_ < -PI)
            yaw_ += 2 * PI;

        yawdot = -D_YAW_MAX;
        }
        else
        {
        yaw_ = aim_yaw;
        if (yaw_ - current_yaw > PI)
            yawdot = -D_YAW_MAX;
        else
            yawdot = (aim_yaw - current_yaw) /delta_T;
        }
    }
    else if (aim_yaw - current_yaw < -PI)
    {
        if (aim_yaw - current_yaw + 2 * PI > YAW_MAX)
        {
        yaw_ = current_yaw + YAW_MAX;
        if (yaw_ > PI)
            yaw_ -= 2 * PI;

        yawdot = D_YAW_MAX;
        }
        else
        {
        yaw_ = aim_yaw;
        if (yaw_ - current_yaw < -PI)
            yawdot = D_YAW_MAX;
        else
            yawdot = (aim_yaw - current_yaw) /delta_T;
        }
    }
    else
    {
        if (aim_yaw - current_yaw < -YAW_MAX)
        {
        yaw_ = current_yaw - YAW_MAX;
        if (yaw_ < -PI)
            yaw_ += 2 * PI;

        yawdot = -D_YAW_MAX;
        }
        else if (aim_yaw - current_yaw > YAW_MAX)
        {
        yaw_ = current_yaw + YAW_MAX;
        if (yaw_ > PI)
            yaw_ -= 2 * PI;

        yawdot = D_YAW_MAX;
        }
        else
        {
        yaw_ = aim_yaw;
        if (yaw_ - current_yaw > PI)
            yawdot = -D_YAW_MAX;
        else if (yaw_ - current_yaw < -PI)
            yawdot = D_YAW_MAX;
        else
            yawdot = (aim_yaw - current_yaw) /delta_T;
        }
    }
        if (fabs(yaw_ - last_yaw) <= YAW_MAX)
        yaw = 0.5 * last_yaw + 0.5 * yaw; // nieve LPF
    yawdot = 0.5 * last_yaw_dot + 0.5 * yawdot;
    last_yaw = yaw_;  
    last_yaw_dot = yawdot;
    yaw_yawdot.first = yaw_;
    yaw_yawdot.second = yawdot;

    return yaw_yawdot;
    }
};

}

#endif
