#ifndef VISUAL_COLORIZER_H
#define VISUAL_COLORIZER_H

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>
#include <unordered_map>
#include <vector>

#include "core/temporary/voxel_map_util.h"
#include "visual/camera.h"

namespace se3_lio {

// Colour each planar leaf voxel once (frozen thereafter) with the camera colour
// at its plane centre, restricted to planar leaves in voxels the current scan
// hit (so only surfaces actually observed this frame get painted). Odom never
// reads Plane::color, so this has no effect on state estimation.
void colorizePlanarLeaves(const std::unordered_map<VOXEL_LOC, OctoTree *> &voxel_map,
                          const std::vector<Eigen::Vector3d> &scan_world, double voxel_size,
                          const PinholeCamera &cam, const Sophus::SE3d &T_c_w,
                          const cv::Mat &color);

// Rasterise coloured planar leaves into a north-up BEV: a (2*half_extent_m)-metre
// square centred on center_xy at res metres/pixel, CV_8UC3 BGR, black where empty.
cv::Mat exportBEV(const std::unordered_map<VOXEL_LOC, OctoTree *> &voxel_map,
                  const Eigen::Vector2d &center_xy, double half_extent_m, double res);

}  // namespace se3_lio

#endif  // VISUAL_COLORIZER_H
