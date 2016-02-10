#pragma once

#include <vector>
#include "pose.h"
#include "common.h"

/**
 * A basic pinhole camera class.
 * Points in 3D space can be projected onto its image plane. Points behind the camera or outside the field-of-view
 * are not projected.
 */
struct PinholeCamera
{
    Eigen::Matrix3d K;	//!< Intrinsic parameters
    int width;			//!< image width [pixels]
    int height;			//!< image height [pixels]
    Pose pose;		//!< world pose
    Pose P;		//!< projection transform

    std::vector<Eigen::Vector3d> cam_pts;	//!< The latest set of projected points, in the camera coordinate frame
    std::vector<Eigen::Vector2d> img_pts;	//!< The latest set of projected points, in the image coordinate frame
    std::vector<bool> visible;		//!< Visibility flags for the latest set of projected points

    PinholeCamera();
    PinholeCamera(Eigen::Matrix3d K, int width, int height);

    /** Set the camera's projection transform, aka "camera matrix",
     * i.e the transform from the world frame to the camera's coordinate frame.
     */
    void setProjection(Pose P);

    /// Set the camera's pose, i.e orientation and translation in the world frame.
    void setPose(Pose pose);

    /** Project a point in the world frame into the camera
     *
     * @param world_pt  The point
     * @param cam_pt    [Output] The point in camera coordinates
     * @param img_pt    [Output] The point in image plane coordinates
     *
     * @returns true if the projected point is visible in the image plane
     */
    bool projectPoint(const Eigen::Vector3d& world_pt, Eigen::Vector3d& cam_pt, Eigen::Vector2d& img_pt);

    /** Project a set of points in the world frame into the camera.
     *
     * @param world_pts  Points to project
     * @param cam_pts    The points in camera coordinates
     * @param img_pts    The points in image plane coordinates
     * @param visible    Visibility flags: true for points projected inside the camera frustum.
     */
    void projectPoints(const std::vector<Eigen::Vector3d> world_pts,
        std::vector<Eigen::Vector3d>& cam_pts, std::vector<Eigen::Vector2d>& img_pts, std::vector<bool>& visible);

    /** Project a set of points in the world frame into the camera.
    * This method stores the result inside the camera object for convenience, removing previous projections. */
    void projectPoints(const std::vector<Eigen::Vector3d> world_pts)
    {
        projectPoints(world_pts, cam_pts, img_pts, visible);
    }

    /** Find the subset of points projected into two cameras that are visible in both.
     *
     * @param cam1  The first camera
     * @param cam2  The second camera
     *
     * @param y1n_pts [Output] K-normalized 2D points visible in camera 1
     * @param y2n_pts [Output] K-normalized 2D points visible in camera 2
     *
     * @note Both cameras must have had the same set of world points projected into them first.
     */
    static void getVisiblePoints(const PinholeCamera& cam1, const PinholeCamera& cam2,
        std::vector<Eigen::Vector2d>& y1n_pts, std::vector<Eigen::Vector2d>& y2n_pts);
};