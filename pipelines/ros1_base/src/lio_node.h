#pragma once

// C++ standard library
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <tuple>

// PCL
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// ROS
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include "ros1_conversion.h"

// SE3-LIO
#include "common/utils.h"
#include "pipeline/SE3_LIO.h"
#include "synchronizer/measurement_synchronizer.h"

// Data recorder
#include "data_recorder.h"

struct EIGEN_ALIGN16 SavePoint {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
    SavePoint,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint32_t, time, time))

namespace se3_lio::ros1_base_node {

class LioNode {
public:
    LioNode(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
    ~LioNode();

    void run();

private:
    std::mutex mutex_;

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Publisher pub_odom_;
    ros::Publisher pub_path_;
    ros::Publisher pub_cloud_body_;

    std::string data_dir_;
    std::string lidar_name_;
    std::string imu_name_;

    int imu_idx_ = 0;
    int stop_from_ = 0;
    int stop_per_ = 1000;

    std::vector<std::tuple<std::string, std::string>> data_list_;
    std::vector<std::tuple<uint64_t, std::vector<double>>> imu_data_buffer_;

    se3_lio::synchronizer::MeasurementSynchronizer synchronizer_;

    se3_lio::pipeline::SE3_LIO se3_lio_pipeline_;
    se3_lio::pipeline::SE3_LIO_Config se3_lio_config_;

    Eigen::Matrix4d lidar_extrinsic_ = Eigen::Matrix4d::Identity();

    double lidar_min_range_ = 0.1;

    bool verbose_ = false;

    nav_msgs::Path path_;

    std::shared_ptr<DataRecorder<RecordPointType>> recorder_ptr_;
    int recorder_save_frame_mode_ = 0;
    bool recorder_verbose_ = true;

    void pubOdometry(ros::Publisher pub_odom,
                     const se3_lio::State &_state,
                     std::string frame_id,
                     std::string child_frame_id);
    void pubPath(ros::Publisher pub_path,
                 nav_msgs::Path &path,
                 const se3_lio::State &_state,
                 std::string frame_id);
    void pubCloud(ros::Publisher pub_cloud, const se3_lio::LiDAR &_lidar, std::string frame_id);

    void broadcastTF(const se3_lio::State &_state,
                     std::string frame_id,
                     std::string child_frame_id);

    void pushAllROSMessages();
    void loadDataList();
    void loadIMUData();

    IMU convertIMUData(uint64_t timestamp);
    LiDAR convertLIDARData(const std::string &filename, const std::string &stamp_str,
                           double _min_range);
};

}  // namespace se3_lio::ros1_base_node
