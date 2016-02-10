#include "pinhole_camera.h"
#include "utilities.h"
#include <cassert>

using namespace Eigen;

PinholeCamera::PinholeCamera()
{
}

PinholeCamera::PinholeCamera(Matrix3d K, int width, int height) : K(K), width(width), height(height)
{
}

void PinholeCamera::setProjection(Pose P)
{
    this->P = P;
    pose = P.inverse();
}

void PinholeCamera::setPose(Pose pose)
{
    this->pose = pose;
    P = pose.inverse();
}

bool PinholeCamera::projectPoint(const Vector3d& world_pt, Vector3d& cam_pt, Vector2d& img_pt)
{
    cam_pt = P * world_pt;
    img_pt = (K * cam_pt).hnormalized();
    return (img_pt(0) >= 0 && img_pt(0) < width
        && img_pt(1) >= 0 && img_pt(1) < height
        && cam_pt(2) > 1.0);
}

void PinholeCamera::projectPoints(const std::vector<Vector3d> world_pts,
    std::vector<Vector3d>& cam_pts, std::vector<Vector2d>& img_pts, std::vector<bool>& visible)
{
    size_t n = world_pts.size();

    cam_pts.resize(n);
    img_pts.resize(n);
    visible.resize(n);

    for (size_t i = 0; i < n; i++) {
        visible[i] = projectPoint(world_pts[i], cam_pts[i], img_pts[i]);
    }
}

void PinholeCamera::getVisiblePoints(const PinholeCamera& cam1, const PinholeCamera& cam2,
    std::vector<Vector2d>& y1n_pts, std::vector<Vector2d>& y2n_pts)
{
    assert(cam1.cam_pts.size() == cam2.cam_pts.size());

    size_t n = cam1.cam_pts.size();

    y1n_pts.clear();
    y1n_pts.reserve(n);

    y2n_pts.clear();
    y2n_pts.reserve(n);

    for (size_t i = 0; i < n; i++) {
        if (!cam1.visible[i] || !cam2.visible[i]) continue;

        y1n_pts.push_back(cam1.cam_pts[i].hnormalized());
        y2n_pts.push_back(cam2.cam_pts[i].hnormalized());
    }
}
