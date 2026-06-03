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

        double slow_radius = 1.0;//开始减速半径
        double stop_radius = 0.3;//判定到达目标点半径  
        double goal_reach_radius_ = 2.0;// m，判定到达目标点半径
        double start_pt_change_threshold_ = 1.0; // m，起点变化阈值

        double cmd_vel_max_ = 1.25;          // m/s，GVF 输出速度限幅
        double cmd_acc_max_ = 1.5;           // m/s^2，命令速度状态的加速度上限
        double cmd_jerk_max_ = 20.0;         // m/s^3，命令速度状态的 jerk 上限
        double cmd_offset_rate_max_ = 2.5;   // m/s，位置偏置变化率上限
        double cmd_vel_lpf_hz_ = 3.5;        // Hz，GVF 速度命令低通截止频率
        double cmd_pos_gain_equiv_ = 1.65;   // 位置环等效增益：v_actual ≈ K * position_error
        double cmd_lead_max_ = 1.5;          // m，速度转位置后的最大领先距离
        bool cmd_use_vel_slew_limit_ = false;
        bool cmd_use_vel_feedback_ = false;
        bool cmd_use_pos_ff_ = false;
        bool cmd_pos_ff_xy_only_ = false;
        double cmd_pos_ff_time_ = 0.08;      // s，基于 GVF 速度变化率的位置前馈时间
        double cmd_pos_ff_max_ = 0.20;       // m，位置前馈偏置限幅
        bool cmd_use_switch_motion_limits_ = false;
        bool cmd_skip_motion_limits_on_switch_ = false;
        double cmd_switch_motion_limit_time_ = 0.25; // s，换轨后短时间启用运动学限幅
        double cmd_switch_acc_max_ = 4.0;    // m/s^2，换轨窗口内 GVF 速度变化限幅
        double cmd_switch_jerk_max_ = 30.0;  // m/s^3，换轨窗口内 GVF 加速度变化限幅
        double cmd_switch_offset_rate_max_ = 3.0; // m/s，换轨窗口内位置偏置变化限幅

        double cmd_vel_fb_switch_time_ = 0.50;

        bool cmd_gain_test_enable_ = false;
        double cmd_gain_test_lead_ = 0.4;
        int cmd_gain_test_axis_ = 0;  // 0 表示 x 方向，1 表示 y 方向

        Eigen::Vector3d ref_pos;//上一条命令位置
        Eigen::Vector3d cmd_vel_state_ = Eigen::Vector3d::Zero();//命令速度状态
        bool ref_initialized = false;//是否已经初始化命令状态
        Eigen::Vector3d last_cmd_pos_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d last_curve_vel_ = Eigen::Vector3d::Zero();
        bool has_last_curve_vel_ = false;
        Eigen::Vector3d cmd_vel_lpf_state_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d last_v_cmd_ = Eigen::Vector3d::Zero();
        bool cmd_vel_state_initialized_ = false;
        Eigen::Vector3d last_v_gvf_for_pos_ff_ = Eigen::Vector3d::Zero();
        bool has_last_v_gvf_for_pos_ff_ = false;
        Eigen::Vector3d last_v_gvf_limited_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d last_a_gvf_limited_ = Eigen::Vector3d::Zero();
        bool has_last_v_gvf_limited_ = false;
        Eigen::Vector3d last_cmd_offset_ = Eigen::Vector3d::Zero();
        bool has_last_cmd_offset_ = false;
        ros::Time vel_fb_switch_until_;
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
                        const Eigen::Vector3d& goal_pt, std::string& reason_out);
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
