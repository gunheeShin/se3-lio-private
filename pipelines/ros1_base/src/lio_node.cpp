#include "lio_node.h"

namespace se3_lio::ros1_base_node {

LioNode::LioNode(ros::NodeHandle &nh, ros::NodeHandle &nh_private) : nh_(nh), pnh_(nh_private) {
    pnh_.param<double>("/sensors/imu/acc_cov", se3_lio_config_.acc_noise, 0.1);
    pnh_.param<double>("/sensors/imu/gyr_cov", se3_lio_config_.gyr_noise, 0.1);
    pnh_.param<double>("/sensors/imu/b_acc_cov", se3_lio_config_.bg_noise, 0.0001);
    pnh_.param<double>("/sensors/imu/b_gyr_cov", se3_lio_config_.ba_noise, 0.0001);

    pnh_.param<double>("/sensors/lidar/min_range", lidar_min_range_, 0.1);
    pnh_.param<double>("/sensors/lidar/range_cov", se3_lio_config_.lidar_range_noise,
                       0.001);  // 벡터로 수정해야함
    pnh_.param<double>("/sensors/lidar/angle_cov", se3_lio_config_.lidar_angle_noise, 0.01);

    std::vector<double> lidar_extrinsic_t, lidar_extrinsic_q;
    pnh_.param<std::vector<double>>("/sensors/t_exts", lidar_extrinsic_t, {0.0, 0.0, 0.0});
    pnh_.param<std::vector<double>>("/sensors/q_exts", lidar_extrinsic_q, {1.0, 0.0, 0.0, 0.0});

    pnh_.param<double>("/downsample/resolution", se3_lio_config_.downsample_resolution, 0.5);
    pnh_.param<int>("/max_iter", se3_lio_config_.max_iter, 4);

    pnh_.param<double>("/voxel_map/resolution", se3_lio_config_.voxel_map_resolution, 1.0);
    pnh_.param<int>("/voxel_map/max_layer", se3_lio_config_.voxel_map_max_layer, 2);
    pnh_.param<std::vector<int>>("/voxel_map/layer_size", se3_lio_config_.voxel_map_layer_size,
                                 {5, 5, 5, 5, 5});
    pnh_.param<int>("/voxel_map/max_point_size", se3_lio_config_.voxel_map_max_point_size, 1000);
    pnh_.param<float>("/voxel_map/plane_threshold", se3_lio_config_.voxel_map_plane_thres, 0.01f);
    pnh_.param<bool>("/voxel_map/map_sliding_en", se3_lio_config_.voxel_map_sliding_en, false);
    pnh_.param<double>("/voxel_map/sliding_thresh", se3_lio_config_.voxel_map_sliding_thresh, 8.0);
    pnh_.param<int>("/voxel_map/half_map_size", se3_lio_config_.voxel_map_half_size, 50);

    pnh_.param<bool>("/verbose", verbose_, true);
    pnh_.param<int>("/stop_from", stop_from_, 0);
    pnh_.param<int>("/stop_per", stop_per_, 1000);
    se3_lio_config_.verbose = verbose_;
    lidar_extrinsic_.block<3, 1>(0, 3) =
        Eigen::Vector3d(lidar_extrinsic_t[0], lidar_extrinsic_t[1], lidar_extrinsic_t[2]);
    Eigen::Quaterniond q_lidar_extrinsic(lidar_extrinsic_q[0], lidar_extrinsic_q[1],
                                         lidar_extrinsic_q[2], lidar_extrinsic_q[3]);
    lidar_extrinsic_.block<3, 3>(0, 0) = q_lidar_extrinsic.toRotationMatrix();
    std::cout << "LiDAR extrinsic: \n" << lidar_extrinsic_ << std::endl;

    se3_lio_pipeline_ = se3_lio::pipeline::SE3_LIO(se3_lio_config_);

    pub_odom_ = pnh_.advertise<nav_msgs::Odometry>("/local/odometry", 1000);
    pub_path_ = pnh_.advertise<nav_msgs::Path>("/local/path", 1000);
    pub_cloud_body_ =
        pnh_.advertise<sensor_msgs::PointCloud2>("/local/cloud_registered_body", 1000);

    pnh_.param<std::string>("/data/data_dir", data_dir_, "");
    pnh_.param<std::string>("/data/lidar_name", lidar_name_, "lidar");
    pnh_.param<std::string>("/data/imu_name", imu_name_, "imu");

    std::string result_dir, test_topic, param_set_name;
    bool recorder_verbose;
    pnh_.param<std::string>("/data_recorder/result_dir", result_dir, "/");
    pnh_.param<std::string>("/data_recorder/test_topic", test_topic, "test_topic");
    pnh_.param<std::string>("/data_recorder/param_set_name", param_set_name, "default");
    pnh_.param<int>("/data_recorder/save_frame_mode", recorder_save_frame_mode_, 0);
    pnh_.param<bool>("/data_recorder/verbose", recorder_verbose, true);

    std::string recorder_save_dir = result_dir + "/" + test_topic + "/" + param_set_name;

    recorder_ptr_.reset(new DataRecorder<RecordPointType>());
    recorder_ptr_->init(recorder_save_dir, recorder_save_frame_mode_, recorder_verbose);
}

LioNode::~LioNode() {}

void LioNode::loadDataList() {
    // Let's GO!
    std::string stamp_file = data_dir_ + "/stamps.txt";
    std::ifstream infile(stamp_file);
    if (!infile.is_open()) {
        std::cerr << "Failed to open stamp file: " << stamp_file << std::endl;
        return;
    }

    std::cout << "Loading data list from: " << stamp_file << std::endl;

    std::string line;
    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        std::string filename, stamp_str;
        if (!(iss >> filename >> stamp_str)) {
            break;
        }

        if ((filename == lidar_name_) || (filename == imu_name_)) {
            data_list_.emplace_back(filename, stamp_str);
        } else {
            continue;
        }
    }
    infile.close();

    if (verbose_) {
        std::cout << "Loaded " << data_list_.size() << " data" << std::endl;
    }
}

void LioNode::loadIMUData() {
    std::string imu_file = data_dir_ + "/sensor_data/" + imu_name_ + ".txt";
    std::ifstream infile(imu_file);
    if (!infile.is_open()) {
        std::cerr << "Failed to open IMU file: " << imu_file << std::endl;
        return;
    }

    std::string line;
    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        std::string stamp_str;
        std::vector<double> imu_values(6);  // Assuming 6 values: ax, ay, az, gx, gy, gz
        if (!(iss >> stamp_str >> imu_values[0] >> imu_values[1] >> imu_values[2] >>
              imu_values[3] >> imu_values[4] >> imu_values[5])) {
            break;
        }
        uint64_t timestamp = static_cast<uint64_t>(std::llround(std::stod(stamp_str) * 1e9));
        imu_data_buffer_.emplace_back(timestamp, imu_values);
    }
    infile.close();

    if (verbose_) {
        std::cout << "Loaded " << imu_data_buffer_.size() << " IMU data" << std::endl;
    }
}

IMU LioNode::convertIMUData(uint64_t tgt_time) {
    while (true) {
        uint64_t imu_time = std::get<0>(imu_data_buffer_[imu_idx_]);
        if (imu_time < tgt_time) {
            imu_idx_++;
            continue;
        } else if (imu_time == tgt_time) {
            const auto &imu_values = std::get<1>(imu_data_buffer_[imu_idx_]);
            IMU imu;
            imu.header.timestamp = imu_time / 1e9;
            imu.angular_velocity.x() = imu_values[0];
            imu.angular_velocity.y() = imu_values[1];
            imu.angular_velocity.z() = imu_values[2];
            imu.linear_acceleration.x() = imu_values[3];
            imu.linear_acceleration.y() = imu_values[4];
            imu.linear_acceleration.z() = imu_values[5];

            imu_idx_++;
            return imu;
        } else {
            std::cerr << "No matching IMU data for timestamp: " << tgt_time << std::endl;
        }
    }
}

LiDAR LioNode::convertLIDARData(const std::string &filename,
                                const std::string &stamp_str,
                                double _min_range) {
    std::string lidar_file =
        data_dir_ + "/sensor_data/" + filename + "/" + stamp_str + ".pcd";

    std::ifstream bin_file(lidar_file, std::ios::in | std::ios::binary);
    if (!bin_file.is_open()) {
        std::cout << "[Offline LIO Node] Couldn't read file " << lidar_file << std::endl;
        return LiDAR();
    }

    // Skip the ASCII PCD header; binary point records follow the "DATA" line.
    std::string header_line;
    while (std::getline(bin_file, header_line)) {
        if (header_line.rfind("DATA", 0) == 0) break;
    }

    LiDAR lidar;
    lidar.header.timestamp = std::stod(stamp_str);
    float data[4];
    uint32_t time_ns = 0;
    while (bin_file.read(reinterpret_cast<char *>(data), sizeof(data)) &&
           bin_file.read(reinterpret_cast<char *>(&time_ns), sizeof(time_ns))) {
        if (data[0] * data[0] + data[1] * data[1] + data[2] * data[2] <= (_min_range * _min_range))
            continue;
        CustomPointType pt;
        pt.x = data[0];
        pt.y = data[1];
        pt.z = data[2];
        pt.intensity = data[3];
        pt.timestamp = static_cast<double>(time_ns) * 1e-9;
        lidar.points.push_back(pt);
    }

    return lidar;
}

void LioNode::pubOdometry(ros::Publisher pub_odom,
                          const se3_lio::State &_state,
                          std::string frame_id,
                          std::string child_frame_id) {
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time(_state.stamp);
    odom_msg.header.frame_id = frame_id;
    odom_msg.child_frame_id = child_frame_id;

    convertStateToROSOdomMsg(_state, odom_msg);

    pub_odom.publish(odom_msg);
}

void LioNode::pubPath(ros::Publisher pub_path,
                      nav_msgs::Path &path,
                      const se3_lio::State &_state,
                      std::string frame_id) {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time(_state.stamp);
    pose_msg.header.frame_id = frame_id;

    convertStateToROSPoseStampedMsg(_state, pose_msg);

    path.header.stamp = ros::Time(_state.stamp);
    path.header.frame_id = frame_id;
    path.poses.push_back(pose_msg);
    pub_path.publish(path);
}

void LioNode::pubCloud(ros::Publisher pub_cloud,
                       const se3_lio::LiDAR &_lidar,
                       std::string frame_id) {
    sensor_msgs::PointCloud2 cloud_msg;

    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    pcl_cloud.reserve(_lidar.points.size());
    for (const auto &p : _lidar.points) {
        pcl::PointXYZI q;
        q.x = p.x;
        q.y = p.y;
        q.z = p.z;
        q.intensity = p.intensity;
        pcl_cloud.push_back(q);
    }
    pcl::toROSMsg(pcl_cloud, cloud_msg);

    cloud_msg.header.stamp = ros::Time(_lidar.header.timestamp);
    cloud_msg.header.frame_id = frame_id;

    pub_cloud.publish(cloud_msg);
}

void LioNode::broadcastTF(const se3_lio::State &_state,
                          std::string frame_id,
                          std::string child_frame_id) {
    static tf2_ros::TransformBroadcaster br;

    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time(_state.stamp);
    transformStamped.header.frame_id = frame_id;
    transformStamped.child_frame_id = child_frame_id;

    convertStateToROSTFMsg(_state, transformStamped);

    br.sendTransform(transformStamped);
}

void LioNode::run() {
    loadDataList();
    loadIMUData();

    MeasurementPtr synced_measurement;
    se3_lio::State state = se3_lio_pipeline_.getState();
    size_t data_idx = 0;

    for (size_t data_idx = 0; data_idx < data_list_.size(); ++data_idx) {
        std::cout << "\rProcessing data " << data_idx + 1 << " / " << data_list_.size() << " ("
                  << std::fixed << std::setprecision(1)
                  << 100.0 * (data_idx + 1) / data_list_.size() << "%)" << std::flush;
        const auto &data = data_list_[data_idx];
        const std::string &sensor_name = std::get<0>(data);
        const std::string &stamp_str = std::get<1>(data);

        if (sensor_name == imu_name_) {
            // Process IMU data
            uint64_t timestamp = static_cast<uint64_t>(std::llround(std::stod(stamp_str) * 1e9));
            IMU imu = convertIMUData(timestamp);
            synchronizer_.addIMU(imu);
        } else if (sensor_name == lidar_name_) {
            // Process LiDAR data
            LiDAR lidar = convertLIDARData(sensor_name, stamp_str, lidar_min_range_);
            synchronizer_.addLiDAR(lidar);
        }

        std::this_thread::sleep_for(std::chrono::microseconds(1));

        synced_measurement = synchronizer_.getSyncedMeasurement();
        if (!synced_measurement->is_synced) continue;

        if ((data_idx >= stop_from_) && (data_idx % stop_per_ == 0)) {
            std::cout << "[Processing index: " << data_idx << "], Press [Enter] to continue..."
                      << std::endl;
            std::cin.ignore();
        } else if (verbose_) {
            std::cout << "Processing data index: " << data_idx << "/" << data_list_.size()
                      << std::endl;
        }
        data_idx++;

        synced_measurement->lidar.points =
            transformPointCloud(synced_measurement->lidar.points, lidar_extrinsic_);

        std::sort(synced_measurement->lidar.points.begin(), synced_measurement->lidar.points.end(),
                  [](const CustomPointType &a, const CustomPointType &b) {
                      return a.timestamp < b.timestamp;
                  });

        std::streambuf *cout_buf = nullptr;
        if (!verbose_) {
            cout_buf = std::cout.rdbuf();
            std::cout.rdbuf(nullptr);
        }

        se3_lio_pipeline_.estimatePose(synced_measurement);

        if (!verbose_) {
            std::cout.rdbuf(cout_buf);
        }

        state = se3_lio_pipeline_.getState();

        if (recorder_ptr_ && recorder_ptr_->isInit()) {
            auto pose_cov = state.pose_cov();
            recorder_ptr_->recordPose(state.stamp, std::tie(state.pose, pose_cov));
            recorder_ptr_->savePose();

            if (recorder_ptr_->isCloudRecordEnabled() &&
                synced_measurement->raw_lidar.points.size() > 0) {
                pcl::PointCloud<RecordPointType>::Ptr record_cloud(
                    new pcl::PointCloud<RecordPointType>());
                record_cloud->reserve(synced_measurement->raw_lidar.points.size());
                for (const auto &point : synced_measurement->raw_lidar.points) {
                    RecordPointType out;
                    out.x = point.x;
                    out.y = point.y;
                    out.z = point.z;
                    out.intensity = point.intensity;
                    record_cloud->push_back(out);
                }

                recorder_ptr_->recordCloud(record_cloud, state.stamp);
                recorder_ptr_->saveCloud();
            }
        }

        pubOdometry(pub_odom_, state, "map", "base_link");
        pubCloud(pub_cloud_body_, synced_measurement->lidar, "base_link");
        pubPath(pub_path_, path_, state, "map");
        broadcastTF(state, "map", "base_link");
    }

    if (recorder_ptr_ && recorder_ptr_->isInit()) {
        recorder_ptr_->savePose();
    }
}
}  // namespace se3_lio::ros1_base_node

int main(int argc, char **argv) {
    ros::init(argc, argv, "lio_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    se3_lio::ros1_base_node::LioNode lio_node(nh, nh_private);
    lio_node.run();

    return 0;
}
