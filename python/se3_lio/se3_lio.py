"""High-level Python API over the raw pybind module."""

import numpy as np

from se3_lio.pybind import se3_lio_pybind as _pb
from se3_lio.config import SE3LIOConfig


class SE3LIO:
    """SE(3) LiDAR-Inertial Odometry.

    Wraps the C++ core. `register_frame` reproduces the ROS2 node's per-frame
    data path (build measurement -> apply LiDAR extrinsic -> sort points by
    relative timestamp -> estimatePose) and returns the updated state.
    """

    def __init__(self, config: SE3LIOConfig = None, lidar_extrinsic=None):
        self.config = config if config is not None else SE3LIOConfig()
        self.extrinsic = np.eye(4) if lidar_extrinsic is None else np.asarray(
            lidar_extrinsic, dtype=float
        )
        self._odom = _pb._SE3LIO(self.config.to_pybind(), self.extrinsic)

    #: True if the C++ core was built with OpenCV (set_camera/export_bev usable).
    has_visual = bool(getattr(_pb, "_HAS_VISUAL", False))

    def register_frame(self, points, point_times, imu, frame_stamp, image=None):
        """Run one odometry step.

        points:      (N, 3) float xyz in the LiDAR frame
        point_times: (N,)   float per-point time offset from frame start [s]
        imu:         (M, 7) float rows of [t, ax, ay, az, gx, gy, gz]
        frame_stamp: float, absolute start time of the scan [s]
        image:       optional (H, W, 3) uint8. When given (and a camera was set
                     via set_camera), each scan point's colour is painted onto
                     the voxel-map plane it lands in for a later export_bev().
                     Odometry ignores it. Requires an OpenCV-enabled build.
        Returns ``(state, cloud)``:
          state: pybind _State (pose, vel, bg, ba, grav, covariance, ...)
          cloud: (N, 3) deskewed scan points in the body frame (post-extrinsic).
        """
        return self._odom._register_frame(
            np.ascontiguousarray(points, dtype=float),
            np.ascontiguousarray(point_times, dtype=float).ravel(),
            np.ascontiguousarray(imu, dtype=float),
            float(frame_stamp),
            None if image is None else np.ascontiguousarray(image, dtype=np.uint8),
        )

    def set_camera(self, width, height, fx, fy, cx, cy, dist_coeffs, T_cam_imu):
        """Register the colour camera used by ``register_frame(image=...)``.

        Intrinsics are pinhole; ``dist_coeffs`` are the radtan coefficients used
        once per frame to undistort. ``T_cam_imu`` is the 4x4 camera-from-IMU
        extrinsic. Only available in an OpenCV-enabled build.
        """
        self._odom._set_camera(
            int(width), int(height), float(fx), float(fy), float(cx), float(cy),
            [float(d) for d in dist_coeffs],
            np.ascontiguousarray(T_cam_imu, dtype=float),
        )

    def export_bev(self, half_extent_m, res, center_xy=None):
        """Render the coloured map as a top-down BEV centred on ``center_xy``
        (world (x, y); defaults to the current pose — a past centre yields a
        fully observed patch).

        Returns an (S, S, 3) uint8 image, S = round(2*half_extent_m/res),
        north-up, same channel order as the images passed to register_frame.
        Only available in an OpenCV-enabled build.
        """
        center = None if center_xy is None else (float(center_xy[0]), float(center_xy[1]))
        return self._odom._export_bev(float(half_extent_m), float(res), center)
