#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <optional>

#include <algorithm>
#include <memory>

#include "common/data_type.h"
#include "common/utils.h"
#include "pipeline/SE3_LIO.h"

// Colouring is optional and only compiled when the C++ core was built with
// OpenCV (se3_lio::visual). The released OpenCV-less wheel omits it entirely.
#ifdef SE3_LIO_WITH_VISUAL
#include <cstring>

#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

#include "visual/camera.h"
#include "visual/colorizer.h"
#endif

namespace py = pybind11;
using namespace pybind11::literals;

namespace {

// Mirrors the per-frame data path of the ROS2 node (lio_node.cpp::process):
// build a synced Measurement, apply the LiDAR extrinsic, sort points by
// relative timestamp, then run a single estimatePose step.
class SE3LIOWrapper {
public:
    SE3LIOWrapper(const se3_lio::pipeline::SE3_LIO_Config &config,
                  const Eigen::Matrix4d &lidar_extrinsic)
        : pipeline_(config), extrinsic_(lidar_extrinsic), config_(config) {}

    // Returns (state, cloud): cloud is the deskewed scan in the body frame (the
    // undistorted points estimatePose produced), matching the C++ node's
    // /local/cloud_registered_body.
    py::tuple RegisterFrame(
        const py::array_t<double, py::array::c_style | py::array::forcecast> &points,
        const py::array_t<double, py::array::c_style | py::array::forcecast> &point_times,
        const py::array_t<double, py::array::c_style | py::array::forcecast> &imu,
        double frame_stamp, const py::object &image) {
        if (points.ndim() != 2 || points.shape(1) != 3)
            throw std::invalid_argument("points must have shape (N, 3)");
        if (point_times.ndim() != 1 || point_times.shape(0) != points.shape(0))
            throw std::invalid_argument("point_times must have shape (N,) matching points");
        if (imu.ndim() != 2 || imu.shape(1) != 7)
            throw std::invalid_argument("imu must have shape (M, 7): [t, ax, ay, az, gx, gy, gz]");

        auto meas = std::make_shared<se3_lio::Measurement>();
        meas->is_synced = true;

        auto imu_u = imu.unchecked<2>();
        meas->imu.reserve(imu_u.shape(0));
        for (py::ssize_t i = 0; i < imu_u.shape(0); ++i) {
            se3_lio::IMU sample;
            sample.header.timestamp = imu_u(i, 0);
            sample.linear_acceleration =
                Eigen::Vector3d(imu_u(i, 1), imu_u(i, 2), imu_u(i, 3));
            sample.angular_velocity = Eigen::Vector3d(imu_u(i, 4), imu_u(i, 5), imu_u(i, 6));
            meas->imu.push_back(sample);
        }

        meas->lidar.header.timestamp = frame_stamp;
        auto pts = points.unchecked<2>();
        auto times = point_times.unchecked<1>();
        meas->lidar.points.reserve(pts.shape(0));
        for (py::ssize_t i = 0; i < pts.shape(0); ++i) {
            CustomPointType point;
            point.x = static_cast<float>(pts(i, 0));
            point.y = static_cast<float>(pts(i, 1));
            point.z = static_cast<float>(pts(i, 2));
            point.intensity = 0.0f;
            point.timestamp = times(i);
            meas->lidar.points.push_back(point);
        }

        meas->lidar.points = se3_lio::transformPointCloud(meas->lidar.points, extrinsic_);
        std::sort(meas->lidar.points.begin(), meas->lidar.points.end(),
                  [](const CustomPointType &a, const CustomPointType &b) {
                      return a.timestamp < b.timestamp;
                  });

        se3_lio::MeasurementPtr meas_ptr = meas;
        pipeline_.estimatePose(meas_ptr);

        // undistortCloud() overwrote meas->lidar.points in place with the
        // deskewed (body-frame) points; return them next to the state.
        const auto &out_pts = meas->lidar.points;
        py::ssize_t n = static_cast<py::ssize_t>(out_pts.size());
        py::array_t<double> cloud({n, static_cast<py::ssize_t>(3)});
        auto c = cloud.mutable_unchecked<2>();
        for (py::ssize_t i = 0; i < n; ++i) {
            c(i, 0) = out_pts[i].x;
            c(i, 1) = out_pts[i].y;
            c(i, 2) = out_pts[i].z;
        }

        se3_lio::State state = pipeline_.getState();

#ifdef SE3_LIO_WITH_VISUAL
        if (!image.is_none()) {
            if (!cam_)
                throw std::runtime_error(
                    "register_frame got an image but no camera is set; call set_camera() first");
            auto img =
                image.cast<py::array_t<uint8_t, py::array::c_style | py::array::forcecast>>();
            if (img.ndim() != 3 || img.shape(2) != 3)
                throw std::invalid_argument("image must have shape (H, W, 3) uint8");
            cv::Mat color(static_cast<int>(img.shape(0)), static_cast<int>(img.shape(1)), CV_8UC3,
                          const_cast<void *>(static_cast<const void *>(img.data())));
            const cv::Mat undist = cam_->undistort(color);

            const Eigen::Matrix3d R = state.rot();
            const Eigen::Vector3d t = state.pos();
            std::vector<Eigen::Vector3d> scan_world;
            scan_world.reserve(out_pts.size());
            for (const auto &p : out_pts)
                scan_world.push_back(R * Eigen::Vector3d(p.x, p.y, p.z) + t);

            const Sophus::SE3d T_w_i(Eigen::Quaterniond(R).normalized(), t);
            const Sophus::SE3d T_c_w = T_c_i_ * T_w_i.inverse();
            se3_lio::colorizePlanarLeaves(pipeline_.getMapManager()->voxel_map_, scan_world,
                                          config_.voxel_map_resolution, *cam_, T_c_w, undist);
        }
#else
        if (!image.is_none())
            throw std::runtime_error(
                "se3_lio was built without OpenCV; image colouring is unavailable in this build");
#endif

        return py::make_tuple(state, cloud);
    }

#ifdef SE3_LIO_WITH_VISUAL
    // Camera intrinsics + T_cam_imu (camera-from-IMU). Once set, register_frame
    // paints each scan point's colour onto the voxel-map plane it lands in.
    void SetCamera(int width, int height, double fx, double fy, double cx, double cy,
                   const std::vector<double> &dist_coeffs, const Eigen::Matrix4d &T_cam_imu) {
        cam_ = std::make_unique<se3_lio::PinholeCamera>(width, height, fx, fy, cx, cy, dist_coeffs);
        T_c_i_ = Sophus::SE3d(
            Eigen::Quaterniond(Eigen::Matrix3d(T_cam_imu.block<3, 3>(0, 0))).normalized(),
            T_cam_imu.block<3, 1>(0, 3));
    }

    // Orthographic top-down colour map, north-up, centred on center_xy (or the
    // current pose when absent — a past centre yields a fully observed patch).
    // Returns a (side, side, 3) uint8 image, side = round(2*half_extent_m/res).
    py::array_t<uint8_t> ExportBEV(double half_extent_m, double res,
                                   std::optional<std::pair<double, double>> center_xy) {
        const Eigen::Vector2d center =
            center_xy ? Eigen::Vector2d(center_xy->first, center_xy->second)
                      : Eigen::Vector2d(pipeline_.getState().pos().head<2>());
        cv::Mat bev = se3_lio::exportBEV(pipeline_.getMapManager()->voxel_map_, center,
                                         half_extent_m, res);
        py::array_t<uint8_t> out({static_cast<py::ssize_t>(bev.rows),
                                  static_cast<py::ssize_t>(bev.cols),
                                  static_cast<py::ssize_t>(3)});
        std::memcpy(out.mutable_data(), bev.data,
                    static_cast<size_t>(bev.rows) * bev.cols * 3);
        return out;
    }
#endif

private:
    se3_lio::pipeline::SE3_LIO pipeline_;
    Eigen::Matrix4d extrinsic_;
    se3_lio::pipeline::SE3_LIO_Config config_;
#ifdef SE3_LIO_WITH_VISUAL
    std::unique_ptr<se3_lio::PinholeCamera> cam_;
    Sophus::SE3d T_c_i_;
#endif
};

}  // namespace

PYBIND11_MODULE(se3_lio_pybind, m) {
    using se3_lio::State;
    using Config = se3_lio::pipeline::SE3_LIO_Config;

    py::class_<Config>(m, "_SE3LIOConfig")
        .def(py::init<>())
        .def_readwrite("acc_noise", &Config::acc_noise)
        .def_readwrite("gyr_noise", &Config::gyr_noise)
        .def_readwrite("bg_noise", &Config::bg_noise)
        .def_readwrite("ba_noise", &Config::ba_noise)
        .def_readwrite("lidar_range_noise", &Config::lidar_range_noise)
        .def_readwrite("lidar_angle_noise", &Config::lidar_angle_noise)
        .def_readwrite("downsample_resolution", &Config::downsample_resolution)
        .def_readwrite("max_iter", &Config::max_iter)
        .def_readwrite("voxel_map_resolution", &Config::voxel_map_resolution)
        .def_readwrite("voxel_map_max_layer", &Config::voxel_map_max_layer)
        .def_readwrite("voxel_map_layer_size", &Config::voxel_map_layer_size)
        .def_readwrite("voxel_map_max_point_size", &Config::voxel_map_max_point_size)
        .def_readwrite("voxel_map_plane_thres", &Config::voxel_map_plane_thres)
        .def_readwrite("voxel_map_sliding_en", &Config::voxel_map_sliding_en)
        .def_readwrite("voxel_map_sliding_thresh", &Config::voxel_map_sliding_thresh)
        .def_readwrite("voxel_map_half_size", &Config::voxel_map_half_size)
        .def_readwrite("verbose", &Config::verbose);

    py::class_<State>(m, "_State")
        .def(py::init<>())
        .def_readonly("stamp", &State::stamp)
        .def_readonly("num_inliers", &State::num_inliers)
        .def_readonly("residual", &State::residual)
        .def_readonly("pose", &State::pose)
        .def_readonly("vel", &State::vel)
        .def_readonly("bg", &State::bg)
        .def_readonly("ba", &State::ba)
        .def_readonly("grav", &State::grav)
        .def_readonly("covariance", &State::covariance);

    py::class_<SE3LIOWrapper>(m, "_SE3LIO")
        .def(py::init<const Config &, const Eigen::Matrix4d &>(), "config"_a,
             "lidar_extrinsic"_a)
        .def("_register_frame", &SE3LIOWrapper::RegisterFrame, "points"_a, "point_times"_a,
             "imu"_a, "frame_stamp"_a, "image"_a = py::none())
#ifdef SE3_LIO_WITH_VISUAL
        .def("_set_camera", &SE3LIOWrapper::SetCamera, "width"_a, "height"_a, "fx"_a, "fy"_a,
             "cx"_a, "cy"_a, "dist_coeffs"_a, "T_cam_imu"_a)
        .def("_export_bev", &SE3LIOWrapper::ExportBEV, "half_extent_m"_a, "res"_a,
             "center_xy"_a = py::none())
#endif
        ;

#ifdef SE3_LIO_WITH_VISUAL
    m.attr("_HAS_VISUAL") = true;
#else
    m.attr("_HAS_VISUAL") = false;
#endif
}
