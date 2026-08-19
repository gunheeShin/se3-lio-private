#include "visual/colorizer.h"

#include <cmath>

namespace se3_lio {

namespace {

// A plane counts as up-facing (ground/roof, what a satellite sees) when its
// normal is within ~37 deg of vertical. Walls/sign faces (|nz| ~ 0) are dropped.
constexpr double kUpFacingMinNz = 0.8;

// Same voxel-index convention as the core map builder (voxel_map_util.cpp).
VOXEL_LOC toVoxelLoc(const Eigen::Vector3d &p, double voxel_size) {
    double loc[3];
    for (int j = 0; j < 3; j++) {
        loc[j] = p[j] / voxel_size;
        if (loc[j] < 0) loc[j] -= 1.0;
    }
    return VOXEL_LOC((int64_t)loc[0], (int64_t)loc[1], (int64_t)loc[2]);
}

// Descend to the leaf node whose octant contains p (root already matched by hash).
OctoTree *leafFor(OctoTree *node, const Eigen::Vector3d &p) {
    while (node != nullptr && node->octo_state_ == 1) {  // internal node
        const int leafnum = 4 * (p[0] > node->voxel_center_[0]) +
                            2 * (p[1] > node->voxel_center_[1]) + (p[2] > node->voxel_center_[2]);
        if (node->leaves_[leafnum] == nullptr) break;
        node = node->leaves_[leafnum];
    }
    return node;
}

// Visit every planar leaf (leaf node whose plane converged) under node.
template <typename F>
void forEachPlanarLeaf(OctoTree *node, const F &f) {
    if (node == nullptr) return;
    if (node->octo_state_ == 0) {
        if (node->plane_ptr_ != nullptr && node->plane_ptr_->is_plane) f(node);
        return;
    }
    for (int i = 0; i < 8; i++) forEachPlanarLeaf(node->leaves_[i], f);
}

}  // namespace

void colorizePlanarLeaves(const std::unordered_map<VOXEL_LOC, OctoTree *> &voxel_map,
                          const std::vector<Eigen::Vector3d> &scan_world, double voxel_size,
                          const PinholeCamera &cam, const Sophus::SE3d &T_c_w,
                          const cv::Mat &color) {
    if (color.empty() || color.type() != CV_8UC3) return;

    // Colour each scan point from the image and accumulate it into the plane of
    // the leaf it lands in. Sampling real surface hits (vs the plane centre) is
    // more robust, and each frame contributes many samples to the running mean.
    for (const auto &p_w : scan_world) {
        auto it = voxel_map.find(toVoxelLoc(p_w, voxel_size));
        if (it == voxel_map.end()) continue;
        OctoTree *leaf = leafFor(it->second, p_w);
        if (leaf == nullptr || leaf->plane_ptr_ == nullptr) continue;
        Plane *plane = leaf->plane_ptr_;
        if (!plane->update_enable) continue;  // plane finalised — colour frozen

        const Eigen::Vector3d p_c = T_c_w * p_w;
        if (p_c.z() <= 0.1) continue;
        const Eigen::Vector2d px = cam.project(p_c);
        const int u = (int)std::lround(px.x());
        const int v = (int)std::lround(px.y());
        if (u < 0 || v < 0 || u >= color.cols || v >= color.rows) continue;

        const cv::Vec3b &bgr = color.at<cv::Vec3b>(v, u);
        plane->color_sum += Eigen::Vector3f(bgr[0], bgr[1], bgr[2]);
        plane->color_n += 1;
    }
}

cv::Mat exportBEV(const std::unordered_map<VOXEL_LOC, OctoTree *> &voxel_map,
                  const Eigen::Vector2d &center_xy, double half_extent_m, double res) {
    const int side = std::max(1, (int)std::lround(2.0 * half_extent_m / res));
    cv::Mat bev(side, side, CV_8UC3, cv::Scalar(0, 0, 0));
    const double x0 = center_xy.x() - half_extent_m;
    const double y0 = center_xy.y() - half_extent_m;

    for (const auto &kv : voxel_map) {
        forEachPlanarLeaf(kv.second, [&](OctoTree *node) {
            Plane *plane = node->plane_ptr_;
            if (plane->color_n == 0) return;
            if (std::abs(plane->normal.z()) < kUpFacingMinNz) return;  // ground/roof only
            const Eigen::Vector3f c = plane->color_sum / (float)plane->color_n;
            const cv::Vec3b bgr((uchar)c[0], (uchar)c[1], (uchar)c[2]);

            // Fill the leaf's xy footprint (half-edge = 2 * quater_length_), so a
            // voxel wider than one pixel paints a solid patch instead of a dot.
            const double h = 2.0 * node->quater_length_;
            const int gx_lo = (int)std::floor((node->voxel_center_[0] - h - x0) / res);
            const int gx_hi = (int)std::floor((node->voxel_center_[0] + h - x0) / res);
            const int gy_lo = (int)std::floor((node->voxel_center_[1] - h - y0) / res);
            const int gy_hi = (int)std::floor((node->voxel_center_[1] + h - y0) / res);
            for (int gy = gy_lo; gy <= gy_hi; gy++) {
                if (gy < 0 || gy >= side) continue;
                const int row = side - 1 - gy;  // north-up
                for (int gx = gx_lo; gx <= gx_hi; gx++) {
                    if (gx < 0 || gx >= side) continue;
                    bev.at<cv::Vec3b>(row, gx) = bgr;
                }
            }
        });
    }
    return bev;
}

}  // namespace se3_lio
