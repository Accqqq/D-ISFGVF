#ifndef  _GVF_MANAGER_H
#define  _GVF_MANAGER_H   

//standard
#include <fstream>
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
        double closed_ref_accepted_goal_w_ = 0.0;
        double closed_ref_accepted_lookahead_w_ = 0.0;
        bool closed_ref_has_accepted_goal_ = false;
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
        bool pathPointAtW(const std::shared_ptr<gvf>& g,
                          double query_w,
                          Eigen::Vector3d& point,
                          bool& clamped_to_end,
                          double& path_w_start,
                          double& path_w_end) const;
        bool pathTangentAtW(const std::shared_ptr<gvf>& g,
                            double query_w,
                            Eigen::Vector3d& tangent) const;
        void resetGovernorState();
        GovernorCommandResult makeGovernorInvalidHold(const Eigen::Vector3d& pos,
                                                      const std::string& reason,
                                                      GovernorCommandDebug& dbg);
        GovernorCommandResult runVelocityMatchingGovernor(gvfManager& pm,
                                                          const gvf::LiftedGuidanceResult& out,
                                                          const Eigen::Vector3d& pos,
                                                          double progress_w_after,
                                                          double dt,
                                                          double kp_equiv,
                                                          GovernorCommandDebug& dbg);
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
        void publishGovernorPositionCommand(const Eigen::Vector3d& cmd_pos,
                                            const Eigen::Vector3d& yaw_cmd_vec);

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
                      int& new_i0_out, Eigen::VectorXd& time);
                      
        bool checkCollision();
        void cmdCallback(const ros::TimerEvent& event);
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
        struct ClosedGoalCandidateProgress
        {
            bool valid = false;
            bool passed_obstacle = false;
            double end_delta_w = 0.0;
            double end_to_goal_dist = std::numeric_limits<double>::infinity();
            double lookahead = 0.0;
        };

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
        void publishPathMsg(const Eigen::MatrixXd& traj, const Eigen::MatrixXd& vel);
        void publishReferencePathMsg(const Eigen::MatrixXd& traj, const Eigen::MatrixXd& vel, ros::Publisher& pub);
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
        int selectDefaultClosedLookaheadIndex(const std::vector<double>& candidates) const;
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
