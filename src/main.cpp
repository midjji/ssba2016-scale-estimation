#include <cmath>
#include <cassert>
#include <numeric>
#include <algorithm>
#include <limits>
#include <thread>

#include "utilities.h"
#include "geometry_tools.h"
#include "pinhole_camera.h"
#include "essential_matrix_solver.h"

using namespace std;
using namespace Eigen;

std::vector<Vector3d> gaussianPointCloud(Vector3d mu, Vector3d sigma, uint n)
{
    std::vector<Vector3d> pcl;
    pcl.reserve(n);

    auto rng = [](){ return randn(0, 1); };

    for (uint i = 0; i < n; i++) {
        pcl.push_back(Vector3d(rng() * sigma(0) + mu(0), rng() * sigma(1) + mu(1), rng() * sigma(2) + mu(2)));
    }

    return pcl;
}

/// Generate a random camera projection transform located on a sphere's surface, looking at the center.
Pose randomCameraOnSphere(float radius, Vector3d sphere_center = Vector3d(0, 0, 0))
{
    auto rng = [](){ return randn(0, 1); };

    Vector4d q(rng(), rng(), rng(), rng());
    q.normalize();
    Pose pose(q, Vector3d(0, 0, 0));
    pose.normalize(); // normalize rotation quaternion
    Vector3d lookAt = pose.rotation().col(2);
    pose.translation() = -lookAt * radius + sphere_center;

    return pose.inverse();
}

// Add uniform noise [-max_d, max_d] pixels a set of image points.
void addImageNoise(float max_d, vector<Vector2d>& img_pts)
{
    auto rng = [&](){ return randu(-max_d, max_d); };

    for (auto& y : img_pts) {
        y += Vector2d(rng(), rng());
    }
}

// Generate outliers by reordering a fraction rho of the image points
void createOutliers(float rho, std::vector<Vector2d>& img_pts)
{
    uint n = (uint)img_pts.size();
    uint K = (uint)(rho * n) / 2;

    vector<uint> first = getUniqueRandomInts(0, n, K);
    vector<uint> second = getUniqueRandomInts(0, n, K);

    for (uint i = 0; i < K; i++) {
        std::swap(img_pts[first[i]], img_pts[second[i]]);
    }
}

void estimateError(const Pose& P_ref, const Pose& P_test, double *dRa, double *dta)
{
    Vector3d t_test = P_test.translation().normalized();
    Vector3d t_ref = P_ref.translation().normalized();

    *dRa = (P_test * P_ref.inverse()).getAngle(); // delta rotation angle
    *dta = fabs(t_test.dot(t_ref)); // cosine of the angle between translation directions
}

Matrix3d angleAxisToRotation(double angle, Vector3d axis)
{
    Eigen::AngleAxisd aa(angle, axis.normalized());
    return aa.toRotationMatrix();
}

void EstimateScales(double max_reproj_error, const Pose& Pa1,
    const Pose& Pa2,
    const Pose& Pb1,
    const std::vector<Vector2d>& yb1s,
    const std::vector<Vector2d>& yb2s,
    std::vector<double>& scale_nums,
    std::vector<double>& scale_dens,
    std::vector<double>& iratios)
{
    Pose Pba = Pb1 * Pa1.inverse(); // Known relative transform from camera a to b
    Pose Pa21 = Pa2 * Pa1.inverse(); // Relative transform from Pa1 to Pa2

    auto Rba = Pba.rotation(); // Must not be identity
    auto tba = Pba.translation();

    auto Pab = Pba.inverse();
    auto Rab = Pab.rotation();
    auto tab = Pab.translation();

    auto Ra21 = Pa21.rotation();
    auto ta21 = Pa21.translation();

    auto Rb21 = Rba * Ra21 * Rab; // Relative rotation from Pb1 to Pb2

    auto A = crossMatrix(tba + Rba * Ra21 * tab) * Rb21;
    auto B = crossMatrix(Rba * ta21) * Rb21;

#if 0
    int id = 2;
    printf("[%d] Tba = %s\n", id, toString(Tba).c_str());
    printf("[%d] Ta21 = %s\n", id, toString(Ta21).c_str());
    printf("[%d] Tab = %s\n", id, toString(Tab).c_str());
    printf("[%d] Rab = %s\n", id, toString(Ra21).c_str());
    printf("[%d] Ra21 = %s\n", id, toString(Ra21).c_str());
    printf("[%d] Rb21 = %s\n", id, toString(Rb21).c_str());
    printf("[%d] A = %s\n", id, toString(A).c_str());
    printf("[%d] B = %s\n", id, toString(B).c_str());
#endif
    uint n = (uint)yb1s.size();

    scale_nums.clear();
    scale_dens.clear();
    iratios.clear();

    for (uint i = 0; i < n; i++) {

        // Get a scale estimate from one correspondence pair in camera b

        auto y1 = yb1s[i].homogeneous();
        auto y2 = yb2s[i].homogeneous();

        double s_num = y2.dot(A * y1);
        double s_den = (-y2).dot(B * y1);
        double s = s_num / s_den;

        // Count the inliers 

        Pose Pb2 = Pba * Pose(Pa2.rotation(), s * Pa2.translation());
        Pose Pb21 = Pb2 * Pb1.inverse(); // Relative pose

        vector<bool> inliers(n, true);
        uint icnt = getReprojectionInliers(max_reproj_error, yb1s, yb2s, Pb21, inliers);

        iratios.push_back(icnt / (float)n);
        scale_nums.push_back(s_num);
        scale_dens.push_back(s_den);
    }
}

struct TestCase
{
    Matrix3d K;             // Camera intrinsics
    Pose Pba;               // Rigid relative pose s.t Pb = Pba * Pa;

    float sigma_n;          // 2D pixel observation noise standard deviation [pixels]
    float rho;              // Fraction of observations that are outliers

    int id;                 // Test number
    int pose_id;            // Pose number
    Pose gt_Pa1_w, gt_Pa2_w;    // Camera A ground truth, K-normalized projection
    Pose gt_Pb1_w, gt_Pb2_w;    // Camera B, ground truth, K-normalized projection
    Pose gt_Pa21;
    float gt_scale;         // Ground truth scale
};

/// Test the essential matrix estimator
bool test_ematrix_estimator(int test_no, EssentialMatrixSolver& ems, Pose gt_P2, const vector<Vector2d>& y1, const vector<Vector2d>& y2)
{
    bool is_success = true;

    // Estimate E (returned as camera projection P2) from camera motion

    uint num_points = (uint)y1.size();
    vector<bool> inliers(num_points, true);
    uint ninliers = num_points;
    uint niters;

    double dRa, dta;
    vector<bool> tmp;
    double max_d = ems.parameters.ransac_max_d;
    uint ninliers1;
    float iratio1;

    Pose P2;
    ems.estimate(y1, y2, P2, inliers, ninliers, niters);

    if (ninliers < num_points * 0.1) {
        eprintf("Not enough inliers found. Skipping test.\n");
        is_success = false;
        goto done;
    }

    // Compare to ground truth

    ninliers1 = getEpipolarLineInliers(max_d, y1, y2, P2.getEssentialMatrix(), tmp);
    iratio1 = ninliers1 / (float)num_points;

    estimateError(gt_P2, P2, &dRa, &dta);
    eprintf("[%3d]: nit: %d, nin: %d / %d (%3.2f%%)\tdRa: %.2f deg, cos(dta): %.2f\n",
        test_no,
        niters,
        ninliers1,
        (int)num_points,
        iratio1 * 100,
        dRa * (180 / M_PI),
        dta);

    if (iratio1 < 0.20) {
        is_success = false;
        goto done;
    }

    // TODO: Save result

done:
    return is_success;
}

bool test_scale_estimator(FILE *f, const TestCase& test, double max_reproj_error, Pose& Pa1, Pose& Pa2, Pose& Pb1,
    const vector<Vector2d>& yb1, const vector<Vector2d>& yb2)
{
    vector<double> scale_nums, scale_dens, iratios;
    EstimateScales(max_reproj_error, Pa1, Pa2, Pb1, yb1, yb2, scale_nums, scale_dens, iratios);
#if 01
    fprintf(f, "test{%d}.pose_id = %d;\n", test.id, test.pose_id);
    fprintf(f, "test{%d}.sigma_n = %.2f;\n", test.id, test.sigma_n);
    fprintf(f, "test{%d}.rho = %.2f;\n", test.id, test.rho);
    fprintf(f, "test{%d}.gt_scale = %.2f;\n", test.id, test.gt_scale);
    fprintf(f, "test{%d}.scale_nums = %s;\n", test.id, toString(scale_nums).c_str());
    fprintf(f, "test{%d}.scale_dens = %s;\n", test.id, toString(scale_dens).c_str());
    fprintf(f, "test{%d}.iratios = %s;\n", test.id, toString(iratios).c_str());
#endif
    return true;
}

void test_runner()
{
    //// Test setup ////

    // Point cloud

    Vector3d pcl_center = Vector3d(0, 0, 100);
    Vector3d pcl_stddev = Vector3d(10, 10, 10);
    uint pcl_npoints = 500;

    vector<Vector3d> point_cloud = gaussianPointCloud(pcl_center, pcl_stddev, pcl_npoints);

    // Cameras (The cameras are on a sphere centered on the point cloud, and are looking at its center.)

    float cam_sphere_radius = 100;
    unsigned int ntest_poses = 2;
    vector<Pose> cam_poses; // Camera a poses at t=2

    Matrix3d K;
    K << 800, 0, 320, 0, 800, 240, 0, 0, 1;
    Pose Pba = Pose(angleAxisToRotation(20.0f / 180 * M_PI, Vector3d(0, 1, 0)), Vector3d(2, 0, 0)).inverse(); // Rigid relative pose from Pa to Pb

    while (cam_poses.size() < ntest_poses) {
        Pose p = randomCameraOnSphere(cam_sphere_radius, pcl_center).inverse();

        if (p.translation().norm() < 10 || p.translation().norm() > cam_sphere_radius) {
            continue;
        }
        cam_poses.push_back(p);
    }

    // Test cases

    vector<TestCase> tests;

    int i = 1;

    for (float sigma_n : { 0.0f, 0.5f, 1.0f, 1.5f, 2.0f }) { // Observation noise (added to observed image coordinates)
        for (float rho : { 0.0f, 0.1f, 0.2f, /*0.3f, 0.4f*/ }) {
            int pose_id = 0;
            for (auto p : cam_poses) {

                TestCase test;
                test.K = K;
                test.Pba = Pba;
                test.id = i++;
                test.pose_id = pose_id++;

                test.sigma_n = sigma_n;
                test.rho = rho;
                test.gt_Pa1_w = Pose(); // identity
                test.gt_Pa21 = p.inverse();
                test.gt_Pa2_w = test.gt_Pa21 * test.gt_Pa1_w;
                test.gt_Pb1_w = Pba * test.gt_Pa1_w;
                test.gt_Pb2_w = Pba * test.gt_Pa2_w;
                test.gt_scale = test.gt_Pa21.translation().norm();

                tests.push_back(test);
            }
        }
    }

    // Set up pinhole cameras

    PinholeCamera cam1 = PinholeCamera(K, 640, 480);
    PinholeCamera cam2 = PinholeCamera(K, 640, 480);

    // Set up E-matrix solver

    EssentialMatrixSolver ems;
    EssentialMatrixSolver::Parameters ems_params;

    ems_params.ransac_answer_confidence = 0.9995;
    ems_params.ransac_max_d = 2 / K(0, 0);
    ems_params.max_reproj_error = 2 / K(0, 0);
    ems_params.ransac_max_iters = 1000;
    ems_params.early_success_iratio = 0.85;
    ems_params.minimal_test_sz = 50;
    ems.setParameters(ems_params);

    FILE *f = fopen("scaleest_results2.m", "w");

    i = 0;
    for (auto& test : tests) {

        i++;

        eprintf("Test %d of %d: sigma_n = %.2f, rho = %.2f\n", i, (int)tests.size(), test.sigma_n, test.rho);

        vector<Vector2d> ya1, ya2; // Observations in Pa at t = {1,2}
        vector<Vector2d> yb1, yb2; // Observations in Pb at t = {1,2}

        // Project into camera A at both time instances

        cam1.setProjection(test.gt_Pa1_w);
        cam2.setProjection(test.gt_Pa2_w);
        cam1.projectPoints(point_cloud);
        cam2.projectPoints(point_cloud);

        PinholeCamera::getVisiblePoints(cam1, cam2, ya1, ya2);  // Get points visible in both cameras
        //eprintf("camera 0: %ld matches\n", y_0a.size());

        // Project into camera B at both time instances

        cam1.setProjection(test.gt_Pb1_w);
        cam2.setProjection(test.gt_Pb2_w);
        cam1.projectPoints(point_cloud);
        cam2.projectPoints(point_cloud);

        PinholeCamera::getVisiblePoints(cam1, cam2, yb1, yb2);
        //eprintf("camera 1: %ld matches\n", y_1a.size());

        // Generate outliers and add noise to the observations

        Matrix3d& K = test.K;
        float y_noise = test.sigma_n / K(0, 0);
        float rho = test.rho;

        addImageNoise(y_noise, ya1);
        addImageNoise(y_noise, ya2);

        addImageNoise(y_noise, yb1);
        addImageNoise(y_noise, yb2);

        createOutliers(rho, ya2);
        createOutliers(rho, yb2);

        //// Test the scale estimator ////

        // Test
        //test_ematrix_estimator(i, ems, test.gt_Pa2_w, ya1, ya2);

        Pose Pa1_w = test.gt_Pa1_w; // [I | 0]
        Pose Pa2_w;
#if 1
        // Estimate E-matrix
        uint ninliers = (uint)ya1.size();
        vector<bool> inliers(ninliers, true);
        uint niters;
        ems.estimate(ya1, ya2, Pa2_w, inliers, ninliers, niters);
#else
        // Use ground truth
        Pa2_w = test.gt_Pa2_w;
#endif
        Pa2_w.translation() = Pa2_w.translation().normalized(); // Remove the scale
        Pose Pb1_w = test.gt_Pb1_w;

        double max_reproj_error = ems_params.max_reproj_error;
        test_scale_estimator(f, test, max_reproj_error, Pa1_w, Pa2_w, Pb1_w, yb1, yb2);
    }

    fclose(f);
}

int main()
{
    initRandomNumberGenerators();
    test_runner();
    return 0;
}
