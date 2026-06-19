#pragma once

// C++ standard libraries
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// ROS
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

// SE3-LIO
#include "common/data_type.h"

namespace se3_lio::ros1_base_node {

inline void convertStateToROSOdomMsg(const se3_lio::State &_state, nav_msgs::Odometry &_odom_msg) {
    Eigen::Quaterniond q(_state.rot());
    _odom_msg.pose.pose.position.x = _state.pos()(0);
    _odom_msg.pose.pose.position.y = _state.pos()(1);
    _odom_msg.pose.pose.position.z = _state.pos()(2);
    _odom_msg.pose.pose.orientation.x = q.x();
    _odom_msg.pose.pose.orientation.y = q.y();
    _odom_msg.pose.pose.orientation.z = q.z();
    _odom_msg.pose.pose.orientation.w = q.w();
}

inline void convertStateToROSPoseStampedMsg(const se3_lio::State &_state,
                                            geometry_msgs::PoseStamped &_pose_msg) {
    Eigen::Quaterniond q(_state.rot());
    _pose_msg.pose.position.x = _state.pos()(0);
    _pose_msg.pose.position.y = _state.pos()(1);
    _pose_msg.pose.position.z = _state.pos()(2);
    _pose_msg.pose.orientation.x = q.x();
    _pose_msg.pose.orientation.y = q.y();
    _pose_msg.pose.orientation.z = q.z();
    _pose_msg.pose.orientation.w = q.w();
};

inline void convertStateToROSTFMsg(const se3_lio::State &_state,
                                   geometry_msgs::TransformStamped &_tf_msg) {
    Eigen::Quaterniond q(_state.rot());
    _tf_msg.transform.translation.x = _state.pos()(0);
    _tf_msg.transform.translation.y = _state.pos()(1);
    _tf_msg.transform.translation.z = _state.pos()(2);
    _tf_msg.transform.rotation.x = q.x();
    _tf_msg.transform.rotation.y = q.y();
    _tf_msg.transform.rotation.z = q.z();
    _tf_msg.transform.rotation.w = q.w();
};

}  // namespace se3_lio::ros1_base_node
