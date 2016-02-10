#pragma once

#include <set>
#include <vector>
#include <eigen3/Eigen/Eigen>
#include "pose.h"

/**
 * A basic essential-matrix estimator. The E matrix is estimated with a 5-point solver [1] in
 * a standard RANSAC loop followed by nonlinear optimization of the inlier set.
 *
 * [1] Li, Hongdong, and Richard Hartley. "Five-point motion estimation made easy."
 * Pattern Recognition, 2006. ICPR 2006. 18th International Conference on.
 * Vol. 1. IEEE, 2006.
 *
 * Some definitions to help remembering what is what:
 *
 * X = 3D space vector
 * P1 = [I | 0] = normalized camera projection matrix, first camera
 * P2 = [R | t] = normalized camera projection matrix, second camera
 *
 * With x1 = P1*X, x2 = P2*X and x2^T E x1 = 0 then E = Cross{t} * R
 * where Cross{t} = cross-product matrix of relative camera translation
 * and          R = relative camera rotation
 */
class EssentialMatrixSolver
{
public:

    typedef std::vector<Eigen::Matrix3d>(*MinimalSolverFn)(const std::vector<Eigen::Vector2d>&, const std::vector<Eigen::Vector2d>&);

    struct Parameters
    {
        uint minimal_test_sz;			//!< Size of the test set used to select the minimal E solution

        float ransac_max_d;				//!< RANSAC inlier maximum epipolar-line distance (K-normalized)
        float max_reproj_error;			//!< Inlier reprojection error threshold (K-normalized pixels)
        uint ransac_max_iters;			//!< RANSAC maximum number of iterations allowed
        float ransac_answer_confidence;	//!< RANSAC confidence in answer (0.0 - 1.0), try 0.995
        float ransac_failure_iratio;	//!< RANSAC signals complete failure if the inlier ratio is lower
        float early_success_iratio;		//!< RANSAC early-exit inlier ratio: stop if the current inlier ratio is higher.
    };

    Parameters parameters;
    bool is_configured;

    EssentialMatrixSolver();

    //!< Selected minimal solver function
    MinimalSolverFn ematrixMinSolve;

    /** Configure the solver */
    void setParameters(const Parameters& params);

    /** Update the maximum number of iterations a RANSAC solver may perform, based on current results.
    *
    * @param p              RANSAC success probability, e.g 0.99
    * @param outlier_ratio  Percentage of outliers in the current RANSAC iteration.
    * @param model_points   Number of data points needed by the minimal solver inside the RANSAC loop
    * @param max_iters      Current maximum number of iterations the RANSAC solver is allowed to do.
    *
    * @returns New maximum number of iterations.
    *
    * @note  This function is ported from the C-API of OpenCV, where it is named cvRANSACUpdateNumIters().
    */
    static uint RANSACUpdateNumIters(double p, double outlier_ratio, uint model_points, uint max_iters);

    /** Estimate E in the minimal (5-point) case and test it against some correspondences.
    *
    * @param est_y1s          K-normalized 2D points from camera 1, used for estimating E. Must be of size 5.
    * @param est_y2s          K-normalized 2D points from camera 2 of the same size as est_y1s
    *
    * @param test_y1s         K-normalized 2D points from camera 1, used for testing E. Must not share any points with est_y1s.
    * @param test_y2s         K-normalized 2D points from camera 2 of the same size as test_y1s
    *
    * @param E                [Output] The essential matrix
    * @param test_inliers     [Output] Vector of the same size as test_y1s, marking test inliers true, outliers false.
    * @param num_test_inliers [Output] Number of inliers in the test set
    *
    * @returns  Zero on failure, or the number of good candidates that were evaluated.
    */
    uint estimateMinimalAndTest(
        const std::vector<Eigen::Vector2d>& est_y1s, const std::vector<Eigen::Vector2d>& est_y2s,
        const std::vector<Eigen::Vector2d>& test_y1s, const std::vector<Eigen::Vector2d>& test_y2s,
        Eigen::Matrix3d& E, std::vector<bool>& test_inliers, uint& num_test_inliers) const;

    /**
     * Extract the relative camera transform from an essential matrix.
     * The method selects the transform has the most inlier support
     *
     * @param y1s      K-normalized 2D points from camera 1
     * @param y2s      K-normalized 2D points from camera 2
     * @param E        Essential matrix
     *
     * @param inliers  [Input/output] y1s/y2s inlier flags.
     *                 Expects inliers to be used for camera extraction while ignoring outliers.
     *                 Returns the reprojection inliers of the extracted camera.
     *
     * @param P        [Output] Extracted camera transform
     *
     * @returns inlier count
     */
    uint getCamera(const std::vector<Eigen::Vector2d>& y1s, const std::vector<Eigen::Vector2d>& y2s,
        const Eigen::Matrix3d& E, std::vector<bool>& inliers, Pose& P) const;

    /** Estimate E from N points
    *
    * @param y1s          K-normalized 2D points from camera 1
    * @param y2s          K-normalized 2D points from camera 2
    *
    * @param P            [Output] Camera projection [R|t]. E is computed as E = crossMatrix(t) * R.
    * @param inliers      [Output] Vector of same length as input points, marking inliers 1, outliers 0.
    * @param num_inliers  [Output] Number inliers with the returned E.
    * @param num_iters    [Output] Number of iterations in the RANSAC loop
    *
    * @returns  True on success
    */
    bool estimate(const std::vector<Eigen::Vector2d>& y1s, const std::vector<Eigen::Vector2d>& y2s,
        Pose& P, std::vector<bool>& inliers, uint& num_inliers, uint& num_iters) const;

    /** Refine the estimated E with all inlier point pairs.
    *
    * @param y1s      K-normalized 2D points from camera 1
    * @param y2s      K-normalized 2D points from camera 2
    * @param inliers  [Input/Output] Vector of same length as the input points, flagging inliers true, outliers false.
    * @param P2       [Input/Output] The camera projection [R|t] to work on. This is related to E = Cross{t} * R.
    */
    void refine(const std::vector<Eigen::Vector2d>& y1s, const std::vector<Eigen::Vector2d>& y2s,
        std::vector<bool> *inliers, Pose *P) const;
};

/**
* Compute candidate E matrices using Hartley's 5 point solver implementation.
*
* @param pt1 5 normalized image 2D points from the first camera.
* @param pt2 5 normalized image 2D points from the second camera.
*
* @return A vector of up to 10 3x3 E-matrix candidates satisfying the condition x2^T*E*x1 = 0
*         for all points x1 in pt1 and x2 in pt2.
*/
std::vector<Eigen::Matrix3d> computeEssentialMatrices_Hartley(const std::vector<Eigen::Vector2d>& pts1, const std::vector<Eigen::Vector2d>& pts2);
