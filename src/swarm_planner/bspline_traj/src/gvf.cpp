#include "bspline_race/gvf.h"

namespace FLAG_Race
{
void gvf::init(ros::NodeHandle& nh, const std::string& particle, const std::string& odom, const std::string& cloud)
{
    // === 地图参数获取 ===
    double x_size, y_size, z_size;
    nh.param("sdf_map/resolution", gvf_.resolution_, -1.0);
    nh.param("sdf_map/map_size_x", x_size, -1.0);
    nh.param("sdf_map/map_size_y", y_size, -1.0);
    nh.param("sdf_map/map_size_z", z_size, -1.0);

    nh.param("sdf_map/esdf_slice_height", gvf_.esdf_slice_height_, -0.1);
    nh.param("sdf_map/visualization_truncate_height", gvf_.visualization_truncate_height_, -0.1);
    nh.param("sdf_map/virtual_ceil_height", gvf_.virtual_ceil_height_, -0.1);
    nh.param("sdf_map/ground_height", gvf_.ground_height_, 1.0);

    nh.param("sdf_map/show_occ_time", gvf_.show_occ_time_, false);
    nh.param("sdf_map/show_esdf_time", gvf_.show_esdf_time_, true);
    nh.param("sdf_map/frame_id", gvf_.frame_id_, std::string("world"));
    nh.param("sdf_map/local_bound_inflate", gvf_.local_bound_inflate_, 1.0);
    nh.param("sdf_map/local_map_margin", gvf_.local_map_margin_, 1);

    nh.param("gvf/gvf_gain1", gvf_.K1_, 0.0);
    nh.param("gvf/gvf_gain2", gvf_.K2_, 0.0);   
    nh.param("gvf/gvf_k3", gvf_.K3_, 0.01);
    nh.param("gvf/gvf_k4", gvf_.K4_, 0.0001);
    nh.param("gvf/gvf_use_kinopath", use_kinopath_, true);
    nh.param("gvf/gvf_use_quad_fit", use_quad_fit_, true);
    nh.param("gvf/gvf_inflation", gvf_.obstacles_inflation_, -1.0);
    nh.param("gvf/path_vis_timeout_sec", path_vis_timeout_sec_, 1.0);
    
    nh.param("gvf/local_update_range_x", gvf_.local_update_range_(0), -1.0);
    nh.param("gvf/local_update_range_y", gvf_.local_update_range_(1), -1.0);
    nh.param("gvf/local_update_range_z", gvf_.local_update_range_(2), -1.0);
    
    // 读取自适应GVF参数
    nh.param("gvf/adaptive_enabled", gvf_.adaptive_enabled_, true);
    nh.param("gvf/convergence_bandwidth", gvf_.convergence_bandwidth_, 0.3);
    nh.param("gvf/k1_min_scale", gvf_.k1_min_scale_, 0.3);
    nh.param("gvf/k1_max_scale", gvf_.k1_max_scale_, 1.2);
    nh.param("gvf/k2_min_scale", gvf_.k2_min_scale_, 0.8);
    nh.param("gvf/k2_max_scale", gvf_.k2_max_scale_, 1.8);
    nh.param("gvf/debug_output", gvf_.debug_output_, true);

    nh.param("gvf/progress_window", progress_window_, 1.0);
    nh.param("gvf/progress_rho0", progress_rho0_, 0.5);
    nh.param("gvf/progress_delta", progress_delta_, 0.3);
    nh.param("gvf/alpha_min", alpha_min_, 0.05);

    gvf_.local_bound_inflate_ = std::max(gvf_.resolution_, gvf_.local_bound_inflate_);
    gvf_.resolution_inv_ = 1.0 / gvf_.resolution_;
    gvf_.map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, gvf_.ground_height_);
    gvf_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);

    for (int i = 0; i < 3; ++i)
        gvf_.map_voxel_num_(i) = std::ceil(gvf_.map_size_(i) / gvf_.resolution_);

    gvf_.map_min_boundary_ = gvf_.map_origin_;
    gvf_.map_max_boundary_ = gvf_.map_origin_ + gvf_.map_size_;

    gvf_.map_min_idx_ = Eigen::Vector3i::Zero();
    gvf_.map_max_idx_ = gvf_.map_voxel_num_ - Eigen::Vector3i::Ones();

    // === 初始化数据缓存 ===
    int buffer_size = gvf_.map_voxel_num_(0) *
                      gvf_.map_voxel_num_(1) *
                      gvf_.map_voxel_num_(2);

    gvf_.occupancy_buffer_         = std::vector<double>(buffer_size, 0.0);
    gvf_.occupancy_buffer_neg      = std::vector<char>(buffer_size, 0);
    gvf_.occupancy_buffer_inflate_ = std::vector<char>(buffer_size, 0);

    gvf_.distance_buffer_      = std::vector<double>(buffer_size, 10000.0);
    gvf_.distance_buffer_neg_  = std::vector<double>(buffer_size, 10000.0);
    gvf_.distance_buffer_all_  = std::vector<double>(buffer_size, 10000.0);

    gvf_.tmp_buffer1_ = std::vector<double>(buffer_size, 0.0);
    gvf_.tmp_buffer2_ = std::vector<double>(buffer_size, 0.0);
    gvf_.velocity_buffer_ = std::vector<Eigen::Vector3d>(buffer_size, Eigen::Vector3d::Zero());
    velocity_buffer_ = std::vector<Eigen::Vector3d>(buffer_size, Eigen::Vector3d::Zero());

    // === 初始化局部更新区域和统计量 ===
    gvf_.local_bound_min_ = Eigen::Vector3i::Zero();
    gvf_.local_bound_max_ = Eigen::Vector3i::Zero();

    gvf_.esdf_time_ = 0.0;
    gvf_.max_fuse_time_ = gvf_.max_esdf_time_ = 0.0;
    gvf_.update_num_ = 0;

    gvf_.local_updated_ = false;
    gvf_.esdf_need_update_ = false;
    gvf_.has_odom_ = false;

    gvf_.esdf_time_ = 0.0;
    gvf_.update_num_ = 0;
    gvf_.max_esdf_time_ = 0.0;
    gvf_.max_fuse_time_ = 0.0;

    esdf_timer_ = nh.createTimer(ros::Duration(0.10), &gvf::updateESDFCallback, this);
    vis_timer_ = nh.createTimer(ros::Duration(0.10), &gvf::visCallback, this);

    indep_odom_sub_ = nh.subscribe<nav_msgs::Odometry>(odom, 10, &gvf::odomCallback, this);
    path_sub_ = nh.subscribe<nav_msgs::Path>(particle + "/path", 10, &gvf::pathCallback, this);
    kino_path_sub_ = nh.subscribe<nav_msgs::Path>(particle + "/kinopath", 10, &gvf::kinoPathCallback, this);
    goal_sub_ = nh.subscribe("/move_base_simple/goal", 1000, &gvf::goalCallback, this);
    map_pub_ = nh.advertise<sensor_msgs::PointCloud2>(particle +"/gvf/occupancy", 10);
    map_inf_pub_ = nh.advertise<sensor_msgs::PointCloud2>(particle +"/gvf/occupancy_inflate", 10);
    esdf_pub_ = nh.advertise<sensor_msgs::PointCloud2>(particle +"/gvf/esdf", 10);
    update_range_pub_ = nh.advertise<visualization_msgs::Marker>(particle +"/gvf/update_range", 10);
    gvf_vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>(particle +"/gvf/traj_vis", 10);
    vector_field_pub_ = nh.advertise<visualization_msgs::MarkerArray>(particle +"/gvf/vector_field", 10);

    last_path_recv_time_ = ros::Time(0);
}

void gvf::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // ROS_INFO("RECEIVE NEW GOAL, RESET ALL BUFFER!");
    // // resetBuffer();
}

template <typename F_get_val, typename F_set_val>
void gvf::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim) 
{
    int v[gvf_.map_voxel_num_(dim)];
    double z[gvf_.map_voxel_num_(dim) + 1];

    int k = start;
    v[start] = start;
    z[start] = -std::numeric_limits<double>::max();
    z[start + 1] = std::numeric_limits<double>::max();

    for (int q = start + 1; q <= end; q++) {
        k++;
        double s;

        do {
        k--;
        s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
        } while (s <= z[k]);

        k++;

        v[k] = q;
        z[k] = s;
        z[k + 1] = std::numeric_limits<double>::max();
    }

    k = start;

    for (int q = start; q <= end; q++) {
        while (z[k + 1] < q) k++;
        double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
        f_set_val(q, val);
        }
}

void gvf::updateESDF3d() 
{
    Eigen::Vector3i min_esdf = gvf_.local_bound_min_;
    Eigen::Vector3i max_esdf = gvf_.local_bound_max_;
    // ROS_INFO_STREAM("min_esdf: " << min_esdf.transpose() << ", max_esdf: " << max_esdf.transpose());
    // ========== compute positive DT ==========

    for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
        for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
            fillESDF(
                [&](int z) {
                    return gvf_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 1 ?
                        0 :
                        std::numeric_limits<double>::max();
                },
                [&](int z, double val) { gvf_.tmp_buffer1_[toAddress(x, y, z)] = val; }, 
                min_esdf[2], max_esdf[2], 2);
        }
    }

    for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
        for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
            fillESDF(
                [&](int y) { return gvf_.tmp_buffer1_[toAddress(x, y, z)]; },
                [&](int y, double val) { gvf_.tmp_buffer2_[toAddress(x, y, z)] = val; }, 
                min_esdf[1], max_esdf[1], 1);
        }
    }

    for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
        for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
            fillESDF(
                [&](int x) { return gvf_.tmp_buffer2_[toAddress(x, y, z)]; },
                [&](int x, double val) {
                    gvf_.distance_buffer_[toAddress(x, y, z)] = 
                        gvf_.resolution_ * std::sqrt(val);
                },
                min_esdf[0], max_esdf[0], 0);
        }
    }

    // ========== compute negative distance ==========

    for (int x = min_esdf(0); x <= max_esdf(0); ++x)
        for (int y = min_esdf(1); y <= max_esdf(1); ++y)
            for (int z = min_esdf(2); z <= max_esdf(2); ++z) {
                int idx = toAddress(x, y, z);
                if (gvf_.occupancy_buffer_inflate_[idx] == 0) {
                    gvf_.occupancy_buffer_neg[idx] = 1;
                } else if (gvf_.occupancy_buffer_inflate_[idx] == 1) {
                    gvf_.occupancy_buffer_neg[idx] = 0;
                } else {
                    ROS_ERROR("what?");
                }
            }

    for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
        for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
            fillESDF(
                [&](int z) {
                    return gvf_.occupancy_buffer_neg[
                        x * gvf_.map_voxel_num_[1] * gvf_.map_voxel_num_[2] +
                        y * gvf_.map_voxel_num_[2] + z] == 1 ?
                        0 : std::numeric_limits<double>::max();
                },
                [&](int z, double val) { gvf_.tmp_buffer1_[toAddress(x, y, z)] = val; },
                min_esdf[2], max_esdf[2], 2);
        }
    }

    for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
        for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
            fillESDF(
                [&](int y) { return gvf_.tmp_buffer1_[toAddress(x, y, z)]; },
                [&](int y, double val) { gvf_.tmp_buffer2_[toAddress(x, y, z)] = val; }, 
                min_esdf[1], max_esdf[1], 1);
        }
    }

    for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
        for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
            fillESDF(
                [&](int x) { return gvf_.tmp_buffer2_[toAddress(x, y, z)]; },
                [&](int x, double val) {
                    gvf_.distance_buffer_neg_[toAddress(x, y, z)] = 
                        gvf_.resolution_ * std::sqrt(val);
                },
                min_esdf[0], max_esdf[0], 0);
        }
    }

    // ========== combine pos and neg DT ==========

    for (int x = min_esdf(0); x <= max_esdf(0); ++x)
        for (int y = min_esdf(1); y <= max_esdf(1); ++y)
            for (int z = min_esdf(2); z <= max_esdf(2); ++z) {
                int idx = toAddress(x, y, z);
                gvf_.distance_buffer_all_[idx] = gvf_.distance_buffer_[idx];
                if (gvf_.distance_buffer_neg_[idx] > 0.0)
                    gvf_.distance_buffer_all_[idx] += 
                        (-gvf_.distance_buffer_neg_[idx] + gvf_.resolution_);
            }

}

void gvf::odomCallback(const nav_msgs::OdometryConstPtr& odom) 
{
  gvf_.camera_pos_(0) = odom->pose.pose.position.x;
  gvf_.camera_pos_(1) = odom->pose.pose.position.y;
  gvf_.camera_pos_(2) = odom->pose.pose.position.z;

  gvf_.has_odom_ = true;
}

void gvf::pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    if (use_kinopath_) return;  // 如果使用动力学路径，则不处理A*路径
    last_path_recv_time_ = ros::Time::now();
    if (msg->poses.empty()) {
        last_path_.poses.clear();
        reparam_ready_ = false;
        sample_w_.clear();
        sample_p_.clear();
        sample_dp_.clear();
        sample_tangent_.clear();
        total_w_ = 0.0;
        return;
    }
    if (!last_path_.poses.empty() &&
        last_path_.header.stamp == msg->header.stamp &&
        last_path_.poses.size() == msg->poses.size()) {
        return;
    }
    if (std::isnan(gvf_.camera_pos_(0)) || 
        std::isnan(gvf_.camera_pos_(1)) || 
        std::isnan(gvf_.camera_pos_(2))) return;

    last_path_ = *msg; 
    gvf_.last_camera_pos_ = gvf_.camera_pos_;
    buildReparamTableFromPathMsg(msg);
    // ROS_INFO("[GVF] Received path with %zu poses", msg->poses.size());

    this->resetBuffer(gvf_.camera_pos_ - gvf_.local_update_range_,
                      gvf_.camera_pos_ + gvf_.local_update_range_);

    Eigen::Vector3d p3d, p3d_inf;
    Eigen::Vector3i inf_pt;

    int inf_step = std::ceil(gvf_.obstacles_inflation_ / gvf_.resolution_);
    int inf_step_z = inf_step + 1;

    Eigen::Vector3d min_pos = gvf_.camera_pos_ - gvf_.local_update_range_;
    Eigen::Vector3d max_pos = gvf_.camera_pos_ + gvf_.local_update_range_;

    for (size_t i = 0; i < msg->poses.size(); ++i) {
        const auto& pose = msg->poses[i];

        p3d << pose.pose.position.x, pose.pose.position.y, pose.pose.position.z;
        Eigen::Vector3d v3d(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
        Eigen::Vector3d devi = p3d - gvf_.camera_pos_;
        if (fabs(devi(0)) < gvf_.local_update_range_(0) &&
            fabs(devi(1)) < gvf_.local_update_range_(1) &&
            fabs(devi(2)) < gvf_.local_update_range_(2)) {
            for (int x = -inf_step; x <= inf_step; ++x)
                for (int y = -inf_step; y <= inf_step; ++y)
                    for (int z = -inf_step_z; z <= inf_step_z; ++z) {
                        p3d_inf = p3d + gvf_.resolution_ * Eigen::Vector3d(x, y, z);
                        posToIndex(p3d_inf, inf_pt);
                        if (!isInMap(inf_pt)) continue;
                        int idx = toAddress(inf_pt);
                        gvf_.occupancy_buffer_inflate_[idx] = 1;
                        gvf_.velocity_buffer_[idx] = v3d;
                        velocity_buffer_[idx] = v3d;
                    }
        }
    }

    posToIndex(min_pos, gvf_.local_bound_min_);
    posToIndex(max_pos, gvf_.local_bound_max_);

    boundIndex(gvf_.local_bound_min_);
    boundIndex(gvf_.local_bound_max_);

    gvf_.esdf_need_update_ = true;
}

void gvf::kinoPathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    if (!use_kinopath_) return;  // 如果不使用动力学路径，则不处理
    last_path_recv_time_ = ros::Time::now();
    if (msg->poses.empty()) {
        last_path_.poses.clear();
        reparam_ready_ = false;
        sample_w_.clear();
        sample_p_.clear();
        sample_dp_.clear();
        sample_tangent_.clear();
        total_w_ = 0.0;
        return;
    }
    if (std::isnan(gvf_.camera_pos_(0)) || 
        std::isnan(gvf_.camera_pos_(1)) || 
        std::isnan(gvf_.camera_pos_(2))) return;

    last_path_ = *msg; 
    gvf_.last_camera_pos_ = gvf_.camera_pos_;
    buildReparamTableFromPathMsg(msg);

    this->resetBuffer(gvf_.camera_pos_ - gvf_.local_update_range_,
                      gvf_.camera_pos_ + gvf_.local_update_range_);

    Eigen::Vector3d p3d, p3d_inf;
    Eigen::Vector3i inf_pt;

    int inf_step = std::ceil(gvf_.obstacles_inflation_ / gvf_.resolution_);
    int inf_step_z = inf_step + 1;

    Eigen::Vector3d min_pos = gvf_.camera_pos_ - gvf_.local_update_range_;
    Eigen::Vector3d max_pos = gvf_.camera_pos_ + gvf_.local_update_range_;

    for (const auto& pose : msg->poses) {
        p3d << pose.pose.position.x, pose.pose.position.y, pose.pose.position.z;

        Eigen::Vector3d devi = p3d - gvf_.camera_pos_;

        if (fabs(devi(0)) < gvf_.local_update_range_(0) &&
            fabs(devi(1)) < gvf_.local_update_range_(1) &&
            fabs(devi(2)) < gvf_.local_update_range_(2)) {

            for (int x = -inf_step; x <= inf_step; ++x)
                for (int y = -inf_step; y <= inf_step; ++y)
                    for (int z = -inf_step_z; z <= inf_step_z; ++z) {

                        p3d_inf = p3d + gvf_.resolution_ * Eigen::Vector3d(x, y, z);
                        posToIndex(p3d_inf, inf_pt);
                        if (!isInMap(inf_pt)) continue;

                        int idx = toAddress(inf_pt);
                        gvf_.occupancy_buffer_inflate_[idx] = 1;
                    }
        }
    }

    posToIndex(min_pos, gvf_.local_bound_min_);
    posToIndex(max_pos, gvf_.local_bound_max_);

    boundIndex(gvf_.local_bound_min_);
    boundIndex(gvf_.local_bound_max_);

    gvf_.esdf_need_update_ = true;
}


Eigen::Vector3d gvf::estimateTangentViaQuadraticFit(const Eigen::Vector3d& pos) {
    Eigen::Vector3d tau = Eigen::Vector3d::Zero();

    if (last_path_.poses.size() < 3)  // 至少要有3个点
        return tau;

    size_t closest_idx = 0;
    double min_dist = std::numeric_limits<double>::max();

    // 1. 找到最近的路径点索引
    for (size_t i = 0; i < last_path_.poses.size(); ++i) {
        Eigen::Vector3d path_pt(
            last_path_.poses[i].pose.position.x,
            last_path_.poses[i].pose.position.y,
            last_path_.poses[i].pose.position.z
        );
        double d = (pos - path_pt).squaredNorm();
        if (d < min_dist) {
            min_dist = d;
            closest_idx = i;
        }
    }

    // 2. 提取路径点，处理边界情况
    std::vector<Eigen::Vector3d> path_points;
    if (closest_idx == 0) {
        // 如果是起点，使用前三个点
        for (size_t i = 0; i < 3 && i < last_path_.poses.size(); ++i) {
            path_points.push_back(Eigen::Vector3d(
                last_path_.poses[i].pose.position.x,
                last_path_.poses[i].pose.position.y,
                last_path_.poses[i].pose.position.z
            ));
        }
    } else if (closest_idx >= last_path_.poses.size() - 1) {
        // 如果是终点，使用最后三个点
        for (size_t i = last_path_.poses.size() - 3; i < last_path_.poses.size(); ++i) {
            path_points.push_back(Eigen::Vector3d(
                last_path_.poses[i].pose.position.x,
                last_path_.poses[i].pose.position.y,
                last_path_.poses[i].pose.position.z
            ));
        }
    } else {
        // 正常情况，使用前后各一个点
        for (size_t i = closest_idx - 1; i <= closest_idx + 1; ++i) {
            path_points.push_back(Eigen::Vector3d(
                last_path_.poses[i].pose.position.x,
                last_path_.poses[i].pose.position.y,
                last_path_.poses[i].pose.position.z
            ));
        }
    }

    // 3. 计算切向量
    if (path_points.size() >= 2) {
        // 使用相邻点的差分计算切向量
        tau = path_points.back() - path_points.front();
        
        // 如果路径点数量大于2，使用加权平均
        if (path_points.size() > 2) {
            Eigen::Vector3d tau2 = path_points[2] - path_points[0];
            tau = (tau + tau2) * 0.5;
        }
        
        // 归一化
        double tau_norm = tau.norm();
        if (tau_norm > 1e-6) {
            tau /= tau_norm;
        }
    }

    return tau;
}

Eigen::Vector3d gvf::getTangentVector(const Eigen::Vector3d& pos) {
    // 1. 先将pos转为栅格索引
    Eigen::Vector3i idx;
    posToIndex(pos, idx);
    boundIndex(idx);
    int min_idx = -1;
    double min_dist = std::numeric_limits<double>::max();
    
    // 2. 在一定范围内搜索最近的占据格子
    int search_range = 20; // 可根据需要调整
    int found_count = 0;
    
    for (int dx = -search_range; dx <= search_range; ++dx)
        for (int dy = -search_range; dy <= search_range; ++dy)
            for (int dz = -search_range; dz <= search_range; ++dz) {
                Eigen::Vector3i cur_idx = idx + Eigen::Vector3i(dx, dy, dz);
                if (!isInMap(cur_idx)) continue;
                int cur_addr = toAddress(cur_idx);
                if (gvf_.occupancy_buffer_inflate_[cur_addr] == 1) {
                    found_count++;
                    Eigen::Vector3d cur_pos;
                    indexToPos(cur_idx, cur_pos);
                    double dist = (cur_pos - pos).squaredNorm();
                    if (dist < min_dist) {
                        min_dist = dist;
                        min_idx = cur_addr;
                    }
                }
            }
    if (min_idx >= 0) {
        Eigen::Vector3d v = gvf_.velocity_buffer_[min_idx];
        double norm = v.norm();
        if (norm > 1e-6) return v / norm;
    }
    // 没找到则返回零向量
    return Eigen::Vector3d::Zero();
}

Eigen::Vector3d gvf::calcGuidingVectorField3D(const Eigen::Vector3d pos)
{
    /* ---------- 1. 距离 & 法向量 ---------- */
    Eigen::Vector3d grad_out_3d;
    
    double dist;
    if (use_quad_fit_) {
        dist = getDistWithGradQuadraticFit(pos, grad_out_3d);
    } else {
        dist = getDistWithGradTrilinear(pos, grad_out_3d);
    }
    Eigen::Vector3d n = (-grad_out_3d).normalized();        

    /* ---------- 2. 由vel_buffer获得切向量 ---------- */
    Eigen::Vector3d tau = getTangentVector(pos);
    if (tau.squaredNorm() < 1e-6) {
        tau = estimateTangentViaQuadraticFit(pos);
    }

    /* ---------- 3. 计算引导向量 ---------- */
    // Eigen::Vector3d guiding_vec = gvf_.K1_ * tau - gvf_.K2_ * dist * n;
    double r = gvf_.convergence_bandwidth_; // 使用可配置的收敛带宽
    double s = std::tanh(dist / r);
    
    Eigen::Vector3d guiding_vec;
    
    if (gvf_.adaptive_enabled_) {
        // 自适应参数调整：根据距离动态调整K1和K2的影响
        // 使用sigmoid函数实现更平滑的过渡
        double normalized_dist = dist / r;
        double sigmoid_factor = 1.0 / (1.0 + std::exp(normalized_dist - 1.0)); // 在r距离处为0.5
        
        // 使用可配置的参数调整范围
        double k1_min_scale = gvf_.k1_min_scale_;
        double k1_max_scale = gvf_.k1_max_scale_;
        double k2_min_scale = gvf_.k2_min_scale_;
        double k2_max_scale = gvf_.k2_max_scale_;
        
        // 计算有效的K1和K2参数
        double effective_K1 = gvf_.K1_ * (k1_min_scale + (k1_max_scale - k1_min_scale) * sigmoid_factor);
        double effective_K2 = gvf_.K2_ * (k2_max_scale - (k2_max_scale - k2_min_scale) * sigmoid_factor);
        
        // 当距离很远时，确保有足够的吸引力
        if (dist > 2.0 * r) {
            effective_K2 = gvf_.K2_ * k2_max_scale;
        }
        
        // 调试输出（可以注释掉以提高性能）
        if (gvf_.debug_output_) {
            static int debug_counter = 0;
            if (++debug_counter % 100 == 0) { // 每100次调用输出一次
                ROS_INFO_THROTTLE(1.0, "GVF Debug - Dist: %.3f, r: %.3f, sigmoid: %.3f, K1: %.3f->%.3f, K2: %.3f->%.3f", 
                                 dist, r, sigmoid_factor, gvf_.K1_, effective_K1, gvf_.K2_, effective_K2);
            }
        }
        
        guiding_vec = effective_K1 * tau - effective_K2 * s * n;
    } else {
        // 使用原始的非自适应方法
        guiding_vec = gvf_.K1_ * tau - gvf_.K2_ * s * n;
    }

    return guiding_vec;
}

void gvf::resetBuffer() {
  Eigen::Vector3d min_pos = gvf_.map_min_boundary_;
  Eigen::Vector3d max_pos = gvf_.map_max_boundary_;

  resetBuffer(min_pos, max_pos);

  gvf_.local_bound_min_ = Eigen::Vector3i::Zero();
  gvf_.local_bound_max_ = gvf_.map_voxel_num_ - Eigen::Vector3i::Ones();
}

void gvf::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos) {

  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);

  boundIndex(min_id);
  boundIndex(max_id);

  /* reset occ, dist, and velocity buffer */
  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z) {
        int idx = toAddress(x, y, z);
        gvf_.occupancy_buffer_inflate_[idx] = 0;
        gvf_.distance_buffer_[idx] = 10000;
        gvf_.velocity_buffer_[idx] = Eigen::Vector3d::Zero();
        velocity_buffer_[idx] = Eigen::Vector3d::Zero();
      }
}

void gvf::updateESDFCallback(const ros::TimerEvent& /*event*/) {
  if (!gvf_.esdf_need_update_) return;

  /* esdf */
  ros::Time t1, t2;
  t1 = ros::Time::now();

  updateESDF3d();

  t2 = ros::Time::now();

  gvf_.esdf_time_ += (t2 - t1).toSec();
  gvf_.max_esdf_time_ = max(gvf_.max_esdf_time_, (t2 - t1).toSec());

  if (gvf_.show_esdf_time_)
        ROS_WARN("GVF: cur t = %lf", (t2 - t1).toSec());

  gvf_.esdf_need_update_ = false;
}

void gvf::publishGVF()
{
    if (!gvf_.has_odom_) return;

    visualization_msgs::MarkerArray marker_array;
    int id = 0;

    // 首先添加一个删除所有标记的消息
    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = gvf_.frame_id_;
    delete_marker.header.stamp = ros::Time::now();
    delete_marker.ns = "gvf_vectors";
    delete_marker.id = 0;
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    // 计算采样点的范围
    Eigen::Vector3d min_pos = gvf_.camera_pos_ - gvf_.local_update_range_ * 0.8;
    Eigen::Vector3d max_pos = gvf_.camera_pos_ + gvf_.local_update_range_ * 0.8;

    // 设置采样间隔（可以根据需要调整）
    double sample_resolution = gvf_.resolution_ * 2.0;  // 使用2倍的分辨率作为采样间隔

    // 在x-y平面上采样点
    for (double x = min_pos(0); x <= max_pos(0); x += sample_resolution) {
        for (double y = min_pos(1); y <= max_pos(1); y += sample_resolution) {
            // 使用当前高度作为采样点的高度
            Eigen::Vector3d sample_pos(x, y, gvf_.camera_pos_(2));
            
            // 计算该点的引导向量
            Eigen::Vector3d guiding_vec = calcGuidingVectorField3D(sample_pos);
            
            // 检查是否在边界上且向量值过大
            bool is_boundary = (std::abs(x - min_pos(0)) < sample_resolution || 
                              std::abs(x - max_pos(0)) < sample_resolution ||
                              std::abs(y - min_pos(1)) < sample_resolution || 
                              std::abs(y - max_pos(1)) < sample_resolution);
            
            if ( guiding_vec.norm() > 50.0) {  
                guiding_vec = Eigen::Vector3d::Zero();
            }
            
            // 创建箭头标记
            visualization_msgs::Marker marker;
            marker.header.frame_id = gvf_.frame_id_;
            marker.header.stamp = ros::Time::now();
            marker.ns = "gvf_vectors";
            marker.id = id++;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;

            // 设置箭头的起点和终点
            geometry_msgs::Point start_point, end_point;
            start_point.x = sample_pos(0);
            start_point.y = sample_pos(1);
            start_point.z = sample_pos(2);

            // 计算箭头的终点（起点 + 向量）
            double arrow_scale = 0.08;  // 可以调整箭头的大小
            end_point.x = start_point.x + guiding_vec(0) * arrow_scale;
            end_point.y = start_point.y + guiding_vec(1) * arrow_scale;
            end_point.z = start_point.z + guiding_vec(2) * arrow_scale;

            marker.points.push_back(start_point);
            marker.points.push_back(end_point);

            // 设置箭头的属性
            marker.scale.x = 0.02;  // 箭头轴宽度
            marker.scale.y = 0.04;   // 箭头头部宽度
            marker.scale.z = 0.04;   // 箭头头部长度

            // 设置箭头的颜色（根据向量大小渐变）
            double vec_norm = guiding_vec.norm();
            marker.color.r = std::min(1.0, vec_norm);
            marker.color.g = 0.0;
            marker.color.b = std::max(0.0, 1.0 - vec_norm);
            marker.color.a = 0.8;  // 透明度

            // 设置箭头的方向四元数（使用默认方向）
            marker.pose.orientation.w = 1.0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;

            marker_array.markers.push_back(marker);
        }
    }

    // 发布标记数组
    vector_field_pub_.publish(marker_array);
}

void gvf::visCallback(const ros::TimerEvent& /*event*/) {
    publishMap();
    publishMapInflate(false);
    publishUpdateRange();
    publishESDF();
    publishGVF();  // 添加GVF可视化
    publishPathCylinderVisualization();  // 添加路径圆柱体可视化
}

void gvf::publishPathCylinderVisualization() {
    if (last_path_.poses.empty()) return;
    
    // 创建MarkerArray来存储多个圆柱体
    visualization_msgs::MarkerArray cylinder_array;
    
    // 首先添加一个删除所有标记的消息
    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = gvf_.frame_id_;
    delete_marker.header.stamp = ros::Time::now();
    delete_marker.ns = "path_cylinders";
    delete_marker.id = 0;
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    cylinder_array.markers.push_back(delete_marker);
    
    // 圆柱体参数设置
    double cylinder_radius = 0.05;  // 圆柱体半径，可以根据需要调整
    double cylinder_height = 0.1;   // 圆柱体高度，可以根据需要调整
    
    // 路径颜色设置（渐变效果）
    double color_r = 0.0;  // 红色分量
    double color_g = 1.0;  // 绿色分量
    double color_b = 0.0;  // 蓝色分量
    
    // 遍历路径点，为每两个相邻点创建一个圆柱体
    for (size_t i = 0; i < last_path_.poses.size() - 1; ++i) {
        const auto& pose1 = last_path_.poses[i];
        const auto& pose2 = last_path_.poses[i + 1];
        
        // 计算两个点之间的中点
        Eigen::Vector3d pos1(pose1.pose.position.x, pose1.pose.position.y, pose1.pose.position.z);
        Eigen::Vector3d pos2(pose2.pose.position.x, pose2.pose.position.y, pose2.pose.position.z);
        Eigen::Vector3d mid_point = 0.5 * (pos1 + pos2);
        
        // 计算两点之间的距离
        double segment_length = (pos2 - pos1).norm();
        
        // 计算圆柱体的方向（从点1指向点2）
        Eigen::Vector3d direction = (pos2 - pos1).normalized();
        
        // 计算圆柱体的旋转（将z轴旋转到direction方向）
        Eigen::Quaterniond rotation;
        if (std::abs(direction.z()) > 0.99) {
            // 如果方向接近垂直，使用特殊处理
            if (direction.z() > 0) {
                rotation = Eigen::Quaterniond::Identity();
            } else {
                rotation = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
            }
        } else {
            // 计算从z轴到direction的旋转
            Eigen::Vector3d z_axis(0, 0, 1);
            Eigen::Vector3d rotation_axis = z_axis.cross(direction);
            double rotation_angle = acos(z_axis.dot(direction));
            
            if (rotation_axis.norm() > 1e-6) {
                rotation_axis.normalize();
                rotation = Eigen::Quaterniond(Eigen::AngleAxisd(rotation_angle, rotation_axis));
            } else {
                rotation = Eigen::Quaterniond::Identity();
            }
        }
        
        // 创建圆柱体标记
        visualization_msgs::Marker cylinder;
        cylinder.header.frame_id = gvf_.frame_id_;
        cylinder.header.stamp = ros::Time::now();
        cylinder.ns = "path_cylinders";
        cylinder.id = i + 1;  // 从1开始，因为0是删除标记
        cylinder.type = visualization_msgs::Marker::CYLINDER;
        cylinder.action = visualization_msgs::Marker::ADD;
        
        // 设置圆柱体的位置和方向
        cylinder.pose.position.x = mid_point.x();
        cylinder.pose.position.y = mid_point.y();
        cylinder.pose.position.z = mid_point.z();
        cylinder.pose.orientation.w = rotation.w();
        cylinder.pose.orientation.x = rotation.x();
        cylinder.pose.orientation.y = rotation.y();
        cylinder.pose.orientation.z = rotation.z();
        
        // 设置圆柱体的尺寸
        cylinder.scale.x = 2.0 * cylinder_radius;  // 直径
        cylinder.scale.y = 2.0 * cylinder_radius;  // 直径
        cylinder.scale.z = segment_length;          // 高度为两点间距离
        
        // 设置圆柱体的颜色（渐变色效果）
        double progress = static_cast<double>(i) / static_cast<double>(last_path_.poses.size() - 2);

        if (progress < 0.25) {
            // 0.0-0.25: 蓝色到青色
            cylinder.color.r = 0.0;
            cylinder.color.g = progress * 4.0;  // 0 -> 1
            cylinder.color.b = 1.0;
        } else if (progress < 0.5) {
            // 0.25-0.5: 青色到绿色
            cylinder.color.r = 0.0;
            cylinder.color.g = 1.0;
            cylinder.color.b = 1.0 - (progress - 0.25) * 4.0;  // 1 -> 0
        } else if (progress < 0.75) {
            // 0.5-0.75: 绿色到黄色
            cylinder.color.r = (progress - 0.5) * 4.0;  // 0 -> 1
            cylinder.color.g = 1.0;
            cylinder.color.b = 0.0;
        } else {
            // 0.75-1.0: 黄色到红色
            cylinder.color.r = 1.0;
            cylinder.color.g = 1.0 - (progress - 0.75) * 4.0;  // 1 -> 0
            cylinder.color.b = 0.0;
        }
        
        cylinder.color.a = 0.9;  // 透明度
        
        cylinder_array.markers.push_back(cylinder);
    }
    
    // 发布圆柱体数组
    gvf_vis_pub_.publish(cylinder_array);
    
    // ROS_DEBUG_THROTTLE(2.0, "[GVF] Published %zu path cylinders", cylinder_array.markers.size() - 1);
}

void gvf::publishMap() 
{
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = gvf_.local_bound_min_;
  Eigen::Vector3i max_cut = gvf_.local_bound_max_;

  int lmm = gvf_.local_map_margin_ / 2;
  min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
  max_cut += Eigen::Vector3i(lmm, lmm, lmm);

  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z) {
        if (gvf_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 0) continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > gvf_.visualization_truncate_height_) continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = gvf_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_pub_.publish(cloud_msg);
}

void gvf::publishMapInflate(bool all_info) {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = gvf_.local_bound_min_;
  Eigen::Vector3i max_cut = gvf_.local_bound_max_;

  if (all_info) {
    int lmm = gvf_.local_map_margin_;
    min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
    max_cut += Eigen::Vector3i(lmm, lmm, lmm);
  }

  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z) {
        if (gvf_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 0) continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > gvf_.visualization_truncate_height_) continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = gvf_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_inf_pub_.publish(cloud_msg);

  // ROS_INFO("pub map");
}

void gvf::publishUpdateRange() {
  Eigen::Vector3d esdf_min_pos, esdf_max_pos, cube_pos, cube_scale;
  visualization_msgs::Marker mk;
  indexToPos(gvf_.local_bound_min_, esdf_min_pos);
  indexToPos(gvf_.local_bound_max_, esdf_max_pos);

  cube_pos = 0.5 * (esdf_min_pos + esdf_max_pos);
  cube_scale = esdf_max_pos - esdf_min_pos;
  mk.header.frame_id = gvf_.frame_id_;
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::CUBE;
  mk.action = visualization_msgs::Marker::ADD;
  mk.id = 0;

  mk.pose.position.x = cube_pos(0);
  mk.pose.position.y = cube_pos(1);
  mk.pose.position.z = cube_pos(2);

  mk.scale.x = cube_scale(0);
  mk.scale.y = cube_scale(1);
  mk.scale.z = cube_scale(2);

  mk.color.a = 0.3;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;

  mk.pose.orientation.w = 1.0;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;

  update_range_pub_.publish(mk);
}

void gvf::publishESDF() {
  double dist;
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI pt;

  const double min_dist = 0.0;
  const double max_dist = 3.0;

  Eigen::Vector3i min_cut = gvf_.local_bound_min_ -
      Eigen::Vector3i(gvf_.local_map_margin_, gvf_.local_map_margin_, gvf_.local_map_margin_);
  Eigen::Vector3i max_cut = gvf_.local_bound_max_ +
      Eigen::Vector3i(gvf_.local_map_margin_, gvf_.local_map_margin_, gvf_.local_map_margin_);
  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y) {

      Eigen::Vector3d pos;
      indexToPos(Eigen::Vector3i(x, y, 1), pos);
      pos(2) = gvf_.esdf_slice_height_;

      dist = getDistance(pos);
      dist = min(dist, max_dist);
      dist = max(dist, min_dist);

      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = -0.2;
      pt.intensity = (dist - min_dist) / (max_dist - min_dist);
      cloud.push_back(pt);
    }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = gvf_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);

  esdf_pub_.publish(cloud_msg);

  // ROS_INFO("pub esdf");
}

void gvf::setNextPathWAnchor(double w_anchor)
{
    next_path_w_anchor_ = w_anchor;
    has_next_path_w_anchor_ = true;
}

void gvf::clearPathReparamState()
{
    last_path_.poses.clear();
    sample_w_.clear();
    sample_p_.clear();
    sample_dp_.clear();
    sample_tangent_.clear();
    total_w_ = 0.0;
    reparam_ready_ = false;
    next_path_w_anchor_ = 0.0;
    has_next_path_w_anchor_ = false;
}

void gvf::buildReparamTableFromPathMsg(const nav_msgs::Path::ConstPtr& msg)
{
    sample_w_.clear();
    sample_p_.clear();
    sample_dp_.clear();
    sample_tangent_.clear();
    total_w_ = 0.0;
    reparam_ready_ = false;

    if (!msg || msg->poses.size() < 2) return;

    const size_t N = msg->poses.size();
    sample_w_.resize(N, 0.0);
    sample_p_.resize(N, Eigen::Vector3d::Zero());
    sample_dp_.resize(N, Eigen::Vector3d::Zero());
    sample_tangent_.resize(N, Eigen::Vector3d::Zero());

    // 1) 读取路径点
    for (size_t i = 0; i < N; ++i) {
        sample_p_[i] = Eigen::Vector3d(
            msg->poses[i].pose.position.x,
            msg->poses[i].pose.position.y,
            msg->poses[i].pose.position.z
        );
    }

    // 2) 近似弧长参数化：新轨迹起点先继承旧轨迹当前progress锚点，再按新轨迹自身弧长继续累加
    sample_w_[0] = has_next_path_w_anchor_ ? next_path_w_anchor_ : 0.0;
    has_next_path_w_anchor_ = false;
    for (size_t i = 1; i < N; ++i) {
        double ds = (sample_p_[i] - sample_p_[i - 1]).norm();
        sample_w_[i] = sample_w_[i - 1] + ds;
    }
    total_w_ = sample_w_.back();

    // 3) 用真实 w 间隔做标准差分，得到 dp/dw
    for (size_t i = 0; i < N; ++i) {
        Eigen::Vector3d dp = Eigen::Vector3d::Zero();

        if (i == 0) {
            double dw = std::max(1e-9, sample_w_[1] - sample_w_[0]);
            dp = (sample_p_[1] - sample_p_[0]) / dw;
        } else if (i == N - 1) {
            double dw = std::max(1e-9, sample_w_[N - 1] - sample_w_[N - 2]);
            dp = (sample_p_[N - 1] - sample_p_[N - 2]) / dw;
        } else {
            double dw = std::max(1e-9, sample_w_[i + 1] - sample_w_[i - 1]);
            dp = (sample_p_[i + 1] - sample_p_[i - 1]) / dw;
        }

        sample_dp_[i] = dp;
        if (dp.norm() > 1e-6) {
            sample_tangent_[i] = dp.normalized();
        } else {
            sample_tangent_[i] = Eigen::Vector3d::Zero();
        }
    }

    reparam_ready_ = true;

    ROS_WARN("[GVF][REPARAM] start_w=%.3f end_w=%.3f points=%zu",
             sample_w_.front(), sample_w_.back(), sample_w_.size());
}

Eigen::Vector3d gvf::evalPathByW(double w) const
{
    if (!reparam_ready_ || sample_w_.empty()) return Eigen::Vector3d::Zero();

    if (w <= sample_w_.front()) return sample_p_.front();
    if (w >= sample_w_.back())  return sample_p_.back();

    auto it = std::lower_bound(sample_w_.begin(), sample_w_.end(), w);
    size_t i1 = std::distance(sample_w_.begin(), it);
    size_t i0 = i1 - 1;

    double w0 = sample_w_[i0];
    double w1 = sample_w_[i1];
    double s = (w - w0) / std::max(1e-9, w1 - w0);

    return (1.0 - s) * sample_p_[i0] + s * sample_p_[i1];
}


Eigen::Vector3d gvf::evalDpDwByW(double w) const
{
    if (!reparam_ready_ || sample_w_.empty()) return Eigen::Vector3d::Zero();

    if (w <= sample_w_.front()) return sample_dp_.front();
    if (w >= sample_w_.back())  return sample_dp_.back();

    auto it = std::lower_bound(sample_w_.begin(), sample_w_.end(), w);
    size_t i1 = std::distance(sample_w_.begin(), it);
    size_t i0 = i1 - 1;

    double w0 = sample_w_[i0];
    double w1 = sample_w_[i1];
    double s = (w - w0) / std::max(1e-9, w1 - w0);

    return (1.0 - s) * sample_dp_[i0] + s * sample_dp_[i1];
}

Eigen::Vector3d gvf::evalTangentByW(double w) const
{
    Eigen::Vector3d dp = evalDpDwByW(w);
    if (dp.norm() > 1e-6) return dp.normalized();
    return Eigen::Vector3d::Zero();
}

double gvf::projectToPathLocal(const Eigen::Vector3d& x,
                               double w_prev,
                               double window) const
{
    if (!reparam_ready_ || sample_w_.empty()) return 0.0;

    double w_min = std::max(sample_w_.front(), w_prev - window);
    double w_max = std::min(sample_w_.back(),  w_prev + window);

    if (w_min > w_max) {
        ROS_INFO("[GVF][PROJ][EMPTY] w_prev=%.3f start_w=%.3f end_w=%.3f window=%.3f w_min=%.3f w_max=%.3f points=%zu",
                 w_prev, sample_w_.front(), sample_w_.back(), window, w_min, w_max, sample_w_.size());
    }

    double best_w = w_prev;
    double best_cost = std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < sample_w_.size(); ++i) {
        double wi = sample_w_[i];
        if (wi < w_min || wi > w_max) continue;

        double cost = (sample_p_[i] - x).squaredNorm()
                    + 2.0 * (wi - w_prev) * (wi - w_prev);  // continuity penalty
        if (cost < best_cost) {
            best_cost = cost;
            best_w = wi;
        }
    }
    return best_w;
}

gvf::LiftedGuidanceResult gvf::calcLiftedGuidance3D(const Eigen::Vector3d& pos,
                                              double w_prev) const
{
    LiftedGuidanceResult out;
    if (!reparam_ready_ || sample_w_.size() < 2) return out;

    // 1) 局部投影到路径，得到当前参考进度
    double w = projectToPathLocal(pos, w_prev, progress_window_);

    // 2) 查询参考点和切向
    Eigen::Vector3d p = evalPathByW(w);
    Eigen::Vector3d dpdw = evalDpDwByW(w);
    double dpdw_norm = dpdw.norm();
    if (dpdw_norm < 1e-6) return out;

    Eigen::Vector3d t = dpdw / dpdw_norm;

    // 3) 误差分解
    Eigen::Vector3d e = pos - p;
    double e_parallel = t.dot(e);
    Eigen::Vector3d e_perp = e - e_parallel * t;
    double rho = e_perp.norm();

    // 4) q_r(rho)
    double r = gvf_.convergence_bandwidth_;
    double q = (rho > 1e-6) ? std::tanh(rho / r) / rho : 1.0 / r;

    // 5) alpha(rho), 保证 > 0
    double alpha = alpha_min_ + (1.0 - alpha_min_) /
                   (1.0 + (rho / progress_rho0_) * (rho / progress_rho0_));

    // 6) sigma(e_parallel)
    double sigma = std::tanh(e_parallel / progress_delta_);

    // 7) lifted GVF 的前3维：实际给无人机用的速度
    Eigen::Vector3d v_cmd =
        gvf_.K1_ * alpha * t
        + gvf_.K2_ * q * e_perp;

    // 8) progress 更新律
    double w_dot =
        gvf_.K1_ * alpha
        + gvf_.K1_ * sigma;

    out.v_cmd = v_cmd;
    out.w_proj = w;
    out.w_dot = w_dot;
    out.e_parallel = e_parallel;
    out.e_perp = e_perp;
    out.ref_pt = p;
    out.tangent = t;
    out.valid = true;
    return out;
}

}
