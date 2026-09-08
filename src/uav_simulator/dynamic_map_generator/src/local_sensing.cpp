#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <algorithm>
#include <cmath>
#include <string>

pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
bool has_map = false;
bool has_odom = false;
bool static_map_frame_verified = false;

// 地图参数
double resolution, x_size, y_size, z_size;
Eigen::Vector3d local_range;

// 当前UAV位置
Eigen::Vector3d current_position;
// Identify the exact odometry sample used for the sensing box.  The cloud
// coordinates, filtering and legacy world-frame convention stay unchanged.
ros::Time sensing_odom_stamp;

// 话题和发布参数。默认值保持原仿真接口不变，同时允许 launch 显式接线。
std::string odom_topic = "/sim/odom";
std::string global_map_topic = "/mock_map";
std::string local_map_topic = "/sim/local_map";
std::string output_frame = "world";
double sensing_rate = 10.0;

// 发布器
ros::Publisher local_map_pub;

bool localSensingBoxComplete(const pcl::PointCloud<pcl::PointXYZ>& backing,
                            const Eigen::Vector3d& center,
                            const Eigen::Vector3d& range,
                            std::size_t returned_points) {
    if (!center.allFinite() || !range.allFinite() || (range.array() <= 0.0).any())
        return false;
    std::size_t expected_points = 0U;
    for (const auto& point : backing.points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
            return false;
        if (std::abs(point.x - center.x()) <= range.x() &&
            std::abs(point.y - center.y()) <= range.y() &&
            std::abs(point.z - center.z()) <= range.z()) ++expected_points;
    }
    return expected_points == returned_points;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_position = Eigen::Vector3d(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z
    );
    has_odom = true;
    sensing_odom_stamp = msg->header.stamp;
}

void mockMapCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    if (!has_map) {
        static_map_frame_verified = msg->header.frame_id == output_frame;
        // PCL 1.10's conversion/search must not dereference empty storage.
        if (msg->width != 0U && msg->height != 0U) pcl::fromROSMsg(*msg, *full_cloud);
        if (!full_cloud->empty()) kdtree.setInputCloud(full_cloud);
        has_map = true;
        ROS_INFO("[local_sensing] Mock map received with %lu points.", full_cloud->points.size());
    }
}

void pubLocalMap() {
    if (!has_map || !has_odom) return;

    pcl::PointCloud<pcl::PointXYZ> localMap;
    pcl::PointXYZ center(current_position.x(), current_position.y(), current_position.z());

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    // local_update_range_* 在 SDFMap 中表示各轴的半范围。先用包围球做
    // KD-tree 粗筛，再做轴对齐范围过滤，保证两端对参数的解释一致。
    const double sensing_radius = local_range.norm();

    if (!full_cloud->empty() &&
        kdtree.radiusSearch(center, sensing_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
        for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
            const pcl::PointXYZ& point = full_cloud->points[pointIdxRadiusSearch[i]];
            if (std::abs(point.x - current_position.x()) <= local_range.x() &&
                std::abs(point.y - current_position.y()) <= local_range.y() &&
                std::abs(point.z - current_position.z()) <= local_range.z()) {
                localMap.points.push_back(point);
            }
        }
    }

    // 空视野也发布空点云，便于 rosbag/话题检查明确区分“传感器正常但
    // 当前无点”和“感知节点没有启动”。
    localMap.width = localMap.points.size();
    localMap.height = 1;
    localMap.is_dense = true;

    sensor_msgs::PointCloud2 localMapMsg;
    pcl::toROSMsg(localMap, localMapMsg);
    localMapMsg.header.frame_id = output_frame;
    // The float KD-tree is only a broad phase.  Before attributing complete
    // support, verify it omitted no static-map point passing the SAME box
    // predicate.  This does not add/remove cloud points or alter sensing.
    // An unverified observation still reaches the legacy map, with no proof
    // identity (zero stamp).  In particular an empty box is checked too.
    localMapMsg.header.stamp = static_map_frame_verified &&
        localSensingBoxComplete(*full_cloud, current_position,
                                                     local_range, localMap.points.size())
        ? sensing_odom_stamp : ros::Time();

    local_map_pub.publish(localMapMsg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "local_sensing_node");
    ros::NodeHandle nh("~");

    // 获取地图参数
    nh.param("sdf_map/resolution", resolution, -1.0);
    nh.param("sdf_map/map_size_x", x_size, -1.0);
    nh.param("sdf_map/map_size_y", y_size, -1.0);
    nh.param("sdf_map/map_size_z", z_size, -1.0);
    nh.param("sdf_map/local_update_range_x", local_range(0), -1.0);
    nh.param("sdf_map/local_update_range_y", local_range(1), -1.0);
    nh.param("sdf_map/local_update_range_z", local_range(2), -1.0);

    nh.param<std::string>("odom_topic", odom_topic, odom_topic);
    nh.param<std::string>("global_map_topic", global_map_topic, global_map_topic);
    nh.param<std::string>("local_map_topic", local_map_topic, local_map_topic);
    nh.param<std::string>("output_frame", output_frame, output_frame);
    nh.param("sensing_rate", sensing_rate, sensing_rate);

    if (!local_range.allFinite() || (local_range.array() <= 0.0).any()) {
        ROS_FATAL_STREAM("[local_sensing] invalid local update range: "
                         << local_range.transpose());
        return 1;
    }
    sensing_rate = std::max(1.0, sensing_rate);

    ros::Subscriber odom_sub = nh.subscribe(odom_topic, 1, odomCallback);
    ros::Subscriber map_sub = nh.subscribe(global_map_topic, 1, mockMapCallback);

    local_map_pub = nh.advertise<sensor_msgs::PointCloud2>(local_map_topic, 1);

    ROS_INFO("[local_sensing] odom=%s global_map=%s local_map=%s frame=%s range=(%.2f, %.2f, %.2f) rate=%.1fHz",
             odom_topic.c_str(), global_map_topic.c_str(), local_map_topic.c_str(),
             output_frame.c_str(), local_range.x(), local_range.y(), local_range.z(),
             sensing_rate);

    ros::Rate rate(sensing_rate);

    while (ros::ok()) {
        ros::spinOnce();
        pubLocalMap();
        rate.sleep();
    }

    return 0;
}
