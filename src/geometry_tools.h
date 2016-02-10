#pragma once

#include <vector>
#include <eigen3/Eigen/Eigen>
#include "pose.h"

Eigen::Matrix3d crossMatrix(const Eigen::Vector3d& v);

//// Essential matrix tools ////////////////////////////////////////////////

/**
* Get the signed distance of a point to the epipolar line.
*
* @param E    Essential matrix
* @param x1n  K-normalized 2D point used to form the epipolar line L = E*x1 / Z, with Z = sqrt(L(0)^2 + L(1)^2)
* @param x2n  K-normalized 2D point to evaluate
*
* @returns  Signed distance to the line.
*
* @note x2*E*x1 = 0,  E = t[x] * R given by cameras P1 = [I|0] and P2 = [R|t].
*       x1 is observed in P1 and x2 is observed in P2.
*/
double getEpipolarLineDistance(const Eigen::Matrix3d& E, const Eigen::Vector2d& x1, const Eigen::Vector2d& x2);

/** Retrieve all possible camera transforms from an essential matrix and a pair of observations
*
* @param E  The essential matrix
* @param y1n K-normalized 2D observation in the first camera
* @param y2n K-normalized 2D observation in the second camera
*/
std::vector<Pose> extractCamerasFromE(const Eigen::Matrix3d& E);

/** Select one of the four camera transforms extracted from E, by triangulating, reprojecting, and selecting
 * the camera which places the triangulated point in front of both the selected camera and camera [I|0].
 *
 * @param cameras  Vector of possible camera projection transforms (camera "matrices")
 * @param y1n      K-normalized 2D observation in camera [I|0]
 * @param y2n      K-normalized 2D observation in cameras under evaluation
 * @returns index in the input vector of the chosen transform.
 */
int selectCameraFromE(const std::vector<Pose>& cameras, const Eigen::Vector2d& y1n, const Eigen::Vector2d& y2n);

/** Select one of the four camera transforms extracted from E, by triangulating, reprojecting, and selecting
* the camera with the most inlier triangulated points. A point is an inlier if it has a small reprojection error
* and is located in front of the selected camera as well as camera [I|0].
*
* @param cameras       Vector of possible camera projection transforms (camera "matrices")
* @param y1s           K-normalized 2D observation in camera [I|0]
* @param y2s           K-normalized 2D observation in cameras under evaluation.
* @param max_e         Inlier maximum reprojection error (K-normalized)
* @param inliers       [Input/Output] Vector of known-good points in y1s and y2s. Points flagged as outliers in the input are ignored.
* @param inlier_count  [Output] Number of triangulation inliers in the selected camera
* @returns index in the input vector of the chosen transform.
*/
uint selectCameraFromE(double max_e, const std::vector<Pose>& cameras,
	const std::vector<Eigen::Vector2d>& y1s, const std::vector<Eigen::Vector2d>& y2s,
	std::vector<bool>& inlier_flags, uint& inlier_count);

/** Retrieve the camera transform from an essential matrix and a pair of observations
*
* @param E   The essential matrix
* @param y1  K-normalized 2D observation in the first camera
* @param y2  K-normalized 2D observation in the second camera
*/
Pose extractCameraFromE(const Eigen::Matrix3d& E, const Eigen::Vector2d& y1, const Eigen::Vector2d& y2);

/** Evaluate and count the point-correspondences (y1,y2) that are inliers based on the the epipolar line distance.
*
* @param max_d    Inlier maximum epipolar-line distance (K-normalized)
* @param y1s      K-normalized 2D points from camera 1
* @param y2s      K-normalized 2D points from camera 2
* @param E        Essential matrix
*
* @param inliers  [Output] Vector of same the length as the input points, with inliers marked true, outliers false.
*
* @returns Number of inliers found
*/
uint getEpipolarLineInliers(double max_d, const std::vector<Eigen::Vector2d>& y1s, const std::vector<Eigen::Vector2d>& y2s,
	const Eigen::Matrix3d& E, std::vector<bool>& inliers);

/** Evaluate and count the point-correspondences (y1,y2) that are inliers.
* 
* A point-pair is an inlier if it
* is markeedhas a small
* reprojection error
* and is located in front of the selected camera as well as camera [I|0]. based on the the epipolar line distance.
*
* @param max_e    Inlier maximum reprojection error (K-normalized)
* @param y1s      K-normalized 2D points from camera 1
* @param y2s      K-normalized 2D points from camera 2
* @param P2       Camera projection transform of the second camera. P1 is assumed to be [I|0].
*
* @param inliers  [Input/Output] Vector of same the length as the input points, with inliers marked true, outliers false.
*
* @returns Number of inliers found.
*
* @note Only the point-correspondences marked as inliers will be evaluated. 
*       Either call getEpipolarLineInliers() first or pass a all-true inlier vector.
*/
uint getReprojectionInliers(double max_e, const std::vector<Eigen::Vector2d>& y1s, const std::vector<Eigen::Vector2d>& y2s,
	const Pose& P2, std::vector<bool>& inliers);

//// 2-view midpoint triangulation /////////////////////////////////////////

class MidpointTriangulator
{
public:
	const Pose& P1;	//!< camera 1 projection transform
	const Pose pose1;	//!< camera 1 pose in world frame = P1.inverse()
	const Eigen::Vector3d c1;	//!< center 1 in world frame
	const Pose& P2;	//!< camera 2 projection transform
	const Pose pose2;	//!< camera 2 pose in world frame = P2.inverse()
	const Eigen::Vector3d c2;	//!< center 2 in world frame

	const double far_distance; //!< The world distance from the cameras to any "far away" 3D point.

	/**
	* @param P0  First camera projection transform (aka camera matrix)
	* @param P1  Second camera projection transform
	* @param d   A world distance such that any 3D point at least
	*            this far away from the camera is projected on the
	*            center camera pixel. Default: 1E7
	*/
	MidpointTriangulator(const Pose& P1, const Pose& P2, double d = 1E7);

	/**
	* Triangulate a 3D point from two camera observations
	* @param y1  K-normalized observation in the first camera
	* @param y2  K-normalized observation in the second camera
	*/
    Eigen::Vector3d triangulate(const Eigen::Vector2d& y1, const Eigen::Vector2d& y2) const;

    static Eigen::Vector3d triangulate(const Pose& P1, const Pose& P2,
        const Eigen::Vector2d& y1, const Eigen::Vector2d& y2, double d = 1E7);

protected:

    void refine(Eigen::Vector2d y1, Eigen::Vector2d y2, Eigen::Vector3d& X) const;

	/** Calculate the line segment PaPb that is the shortest route between two lines P1P2 and P3P4.
    *
	* @param p1    First point on the first line
	* @param p2    Second point on the first line
	* @param p1    First point on the second line
	* @param p2    Second point on the second line
	* @param pa    [Output] Pa
	* @param pb    [Output] Pb
	* @param mua   [Output] Pa = P1 + mua (P2 - P1)
	* @param mub   [Output] Pb = P3 + mub (P4 - P3)
    *
	* @returns true on success, false if no solution exists.
	*/
	bool lineLineIntersect(
		const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
		const Eigen::Vector3d& p3, const Eigen::Vector3d& p4,
        Eigen::Vector3d& pa, Eigen::Vector3d& pb, double& mua, double& mub) const;
};
