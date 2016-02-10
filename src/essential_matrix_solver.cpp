#include <cmath>
#include <numeric>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/local_parameterization.h>

#include "utilities.h"
#include "geometry_tools.h"
#include "ematrix_hartley_gpl.h"
#include "essential_matrix_solver.h"
#include "common.h"

using namespace std;
using namespace Eigen;

//// 5-point solver wrapper //////////////////////////////////////

std::vector<Matrix3d> computeEssentialMatrices_Hartley(const std::vector<Vector2d>& pts1, const std::vector<Vector2d>& pts2)
{
	using namespace hartley;

	vector<Matrix3d> candidates;

	int nroots;
	Ematrix Ematrices[10];

	Matches_5 q1, q2;

	for (int i = 0; i < 5; i++) {
		q1[i][0] = pts1[i](0);
		q1[i][1] = pts1[i](1);
		q1[i][2] = 1;
		q2[i][0] = pts2[i](0);
		q2[i][1] = pts2[i](1);
		q2[i][2] = 1;
	}

	compute_E_matrices(q1, q2, Ematrices, nroots, true);
	candidates.resize(nroots);

	for (int k = 0; k < nroots; k++) {
		for (int j = 0; j < 3; j++) {
			for (int i = 0; i < 3; i++) {
				candidates[k](i, j) = Ematrices[k][i][j];
			}
		}
	}

	return candidates;
}

//// 2-view essential matrix solver //////////////////////////

EssentialMatrixSolver::EssentialMatrixSolver()
{
	ematrixMinSolve = computeEssentialMatrices_Hartley; // Hartley's own implementation
}

void EssentialMatrixSolver::setParameters(const Parameters& params)
{
	this->parameters = params;
	is_configured = true;
}

static bool isNumericallySound(const Matrix3d& E)
{
    double Emax = E.cwiseAbs().sum();
    double Esum = E.cwiseAbs().maxCoeff();

	if (Esum < 1E-10) return false;
	Esum = Esum / Emax; // Eigen: Ecand.cwiseAbs().sum() / Ecand.lpNorm<Infinity>()
	return !std::isnan(Esum) && !std::isinf(Esum);
}

uint EssentialMatrixSolver::RANSACUpdateNumIters(double p, double outlier_ratio, uint model_points, uint max_iters)
{
    p = std::max(std::min(p, 1.0), 0.0);
    outlier_ratio = std::max(std::min(outlier_ratio, 1.0), 0.0);
    double dbl_min = std::numeric_limits<double>::min();

    // avoid inf's & nan's
    double num = std::max(1.0 - p, dbl_min);
    double denom = 1.0 - pow(1.0 - outlier_ratio, model_points);
    if (denom < dbl_min)
        return 0;

    num = log(num);
    denom = log(denom);

    return (denom >= 0 || -num >= max_iters*(-denom)) ? max_iters : (uint)floor(num / denom + 0.5);
}

uint EssentialMatrixSolver::estimateMinimalAndTest(
	const std::vector<Vector2d>& est_y1s, const std::vector<Vector2d>& est_y2s,
	const std::vector<Vector2d>& test_y1s, const std::vector<Vector2d>& test_y2s,
	Matrix3d& E, vector<bool>& test_inliers, uint& num_test_inliers) const
{
	assert(is_configured);
	const Parameters& p = parameters;

	uint good_candidates = 0;
	test_inliers = vector<bool>(test_y1s.size(), false);
	num_test_inliers = 0;

	// Estimate and select the best E candidate
	// The test is used to find the E candidate with the most inliers

	vector<Matrix3d> candidates = ematrixMinSolve(est_y1s, est_y2s);

	if (candidates.size() == 0) {
		goto done;
	}

	for (auto& E_candidate : candidates) {

		// Ensure the minimal solver actually returned a good candidate:
		// the matrix should be nonzero, and have no infs or nans
		// all points in the estimation set should be on the epipolar lines

		if (!isNumericallySound(E_candidate)) continue;

		bool bad_solution = false;
		uint n = (uint)est_y1s.size();

		for (uint i = 0; i < n; i++) {
			double e = fabs(getEpipolarLineDistance(E_candidate, est_y1s[i], est_y2s[i]));
			if (e > 1E-14) {
				bad_solution = true;
				break;
			}
		}
		if (bad_solution) continue;

		// Find the test-set inliers, using epipolar-line distance error
		// store the E matrix if the result is better than for previous candidates

		n = (uint)test_y1s.size();
		vector<bool> iflags(n);
		uint icnt = 0;

		for (uint i = 0; i < n; i++) {
			double e = fabs(getEpipolarLineDistance(E_candidate, test_y1s[i], test_y2s[i]));
			iflags[i] = (e < p.ransac_max_d);
			if (iflags[i]) {
				icnt++;
			}
		}

		if (icnt > num_test_inliers) {
			test_inliers = iflags;
			num_test_inliers = icnt;
			E = E_candidate;
			good_candidates++;
		}
	}

	if (num_test_inliers < p.minimal_test_sz * 0.5) { // Require at least 50% inliers
		good_candidates = 0;
		goto done;
	}

done:
	return good_candidates;
}

uint EssentialMatrixSolver::getCamera(const std::vector<Vector2d>& y1s, const std::vector<Vector2d>& y2s,
	const Matrix3d& E, std::vector<bool>& inliers, Pose& P) const
{
	assert(is_configured);
	const Parameters& p = parameters;

	// Get four camera candidates and pick the one with most reprojection inliers

	vector<Pose> cameras = extractCamerasFromE(E);

	vector<bool> best_iflags;
	uint best_cam_ix = 0;
	uint best_icnt = 0;
	for (uint j = 0; j < 4; j++) {
		auto& cam = cameras[j];
		vector<bool> iflags = inliers;
		uint icnt = getReprojectionInliers(p.max_reproj_error, y1s, y2s, cam, iflags);
		if (icnt > best_icnt) {
			best_icnt = icnt;
			best_iflags = iflags;
			best_cam_ix = j;
		}
	}
	P = cameras[best_cam_ix];
	inliers = best_iflags;

	if (best_icnt == 0) {
		return 0;
	}

	// Optimize the result 

	refine(y1s, y2s, &inliers, &P);

	// Count inliers after refinement
	uint icnt = 0;
	for (auto i : inliers) {
		icnt += i ? 1 : 0;
	}

	return icnt;
}

bool EssentialMatrixSolver::estimate(const std::vector<Vector2d>& y1s,
	const std::vector<Vector2d>& y2s,
	Pose& P,
	std::vector<bool>& inliers,
	uint& num_inliers,
	uint& num_iters) const
{
	assert(is_configured);
	assert(y1s.size() == y2s.size());
	uint num_points = (uint)y1s.size();

	const Parameters& p = parameters;
	const uint model_points = 5; // Number points needed for the minimal case
	const uint max_failures = 100*p.ransac_max_iters; // Number of retries before giving up.


	if (num_points < model_points + p.minimal_test_sz) {
		return false; // No E matrix can be estimated
	}

	inliers = vector<bool>(num_points, false);
	num_inliers = 0;

	// Estimate P (RANSAC loop + optimization)

	uint max_iters = p.ransac_max_iters; // Current maximum number of iterations the RANSAC loop will perform
	uint num_failures = 0; // Number of times minimal solver returned without any hypothesis
	num_iters = 0; // Number of iterations made

	for (uint i = 0; i < max_iters + num_failures && num_failures < max_failures; i++)
	{
		Matrix3d E_candidate;
		Pose P_candidate;

		// Estimate an E and find its epipolar-line inliers

		std::set<uint> estimation_set, test_set;
		vector<Vector2d> est_y1s, est_y2s, test_y1s, test_y2s;
		vector<bool> test_inliers;
		uint num_test_inliers;

		getDisjointRandomIndexSets(5, p.minimal_test_sz, num_points, estimation_set, test_set);

		est_y1s = selectFromIndexSet(y1s, estimation_set);
		est_y2s = selectFromIndexSet(y2s, estimation_set);

		test_y1s = selectFromIndexSet(y1s, test_set);
		test_y2s = selectFromIndexSet(y2s, test_set);

		uint num_hypotheses = estimateMinimalAndTest(est_y1s, est_y2s, test_y1s, test_y2s, E_candidate, test_inliers, num_test_inliers);
		if (num_hypotheses == 0) {
			num_failures++;
			continue;
		}

		// Get a camera from the E, with the test set and
		// test the reprojection error against all point correspondences

		num_test_inliers = getCamera(test_y1s, test_y2s, E_candidate, test_inliers, P_candidate);
		vector<bool> iflags(num_points, true);
		uint icnt = getReprojectionInliers(p.max_reproj_error, y1s, y2s, P_candidate, iflags);

		// Store the result if it is the best so far

		if (icnt > num_inliers) {
			P = P_candidate;
			num_inliers = icnt;
			inliers = iflags;

			// Update the estimate of the maximum number of iterations
			double error_p = (double)(num_points - num_inliers) / num_points; // error probability
			max_iters = RANSACUpdateNumIters(p.ransac_answer_confidence, error_p, model_points, max_iters);
		}
		num_iters++;

		if (num_inliers > num_points * p.early_success_iratio) {
#if 0
			fprintf(stderr, "Have good inlier ratio - stopping early, %d inliers of %d correspondences (%.2f%%)\n", num_inliers, num_points,
				num_inliers * 100.0f / num_points);
#endif
			break; // Quit early if the inlier ratio is good enough
		}
	}

	fprintf(stderr, "E-matrix: %d inliers, %d failures, %d iters\n", num_inliers, num_failures, num_iters);

	if (num_inliers == 0) {
		return false;
	}

	refine(y1s, y2s, &inliers, &P);
	refine(y1s, y2s, &inliers, &P);
	return true;
}

struct PointReprojectionError
{
	Vector2d y1, y2;

	PointReprojectionError(const Vector2d& y1, const Vector2d& y2)
		: y1(y1), y2(y2)
	{
	}

	template <typename T>
	bool operator()(const T* const quaternion, const T* const translation, const T* const X, T* residuals) const
	{

		// Reproject X into the identity camera

		T yr1[3];

		yr1[0] = X[0] / X[2];
		yr1[1] = X[1] / X[2];

		// Reproject X into the camera to optimize over

		T yr2[3];
		ceres::UnitQuaternionRotatePoint(quaternion, X, yr2);
		yr2[0] += translation[0];
		yr2[1] += translation[1];
		yr2[2] += translation[2];

		yr2[0] = yr2[0] / yr2[2];
		yr2[1] = yr2[1] / yr2[2];

		// Get reprojection errors

		residuals[0] = yr1[0] - y1(0);
		residuals[1] = yr1[1] - y1(1);
		residuals[2] = yr2[0] - y2(0);
		residuals[3] = yr2[1] - y2(1);

		return true;
	}

	static ceres::CostFunction* Create(const Vector2d& y1, const Vector2d& y2)
	{
		return (new ceres::AutoDiffCostFunction<PointReprojectionError, 4, 4, 3, 3>(
			new PointReprojectionError(y1, y2)));
	}
};

template<int Size>
class UnitLengthError
{
public:
    UnitLengthError(){}

    template <typename T>
    bool operator()(const T* const rq, // vector
        T* residuals) const
    {
        residuals[0] = T(1);
        for (int i = 0; i < Size; ++i) {
            residuals[0] -= rq[i] * rq[i];
        }
        return true;
    }

    static ceres::CostFunction* Create()
    {
        // resid,first param,second param, third param
        return (new ceres::AutoDiffCostFunction<UnitLengthError, 1, Size>(new UnitLengthError<Size>()));
    }
};

void EssentialMatrixSolver::refine(const std::vector<Vector2d>& y1s, const std::vector<Vector2d>& y2s,
	std::vector<bool> *inliers, Pose *P) const
{
	assert(is_configured);
	assert(y1s.size() == y2s.size());
	const Parameters& p = parameters;

	// Assume P1 = [I|0]
	Pose& P2 = *P;

	size_t n = y1s.size();
	assert(y2s.size() == n);

	// Triangulate inliers 

	MidpointTriangulator mpt(Pose(), P2);
	vector<Vector3d> triangulations;
	vector<Vector2d> observations1, observations2;

	triangulations.reserve(n);
	observations1.reserve(n);
	observations2.reserve(n);

	for (size_t i = 0; i < n; i++) {
        //if (!(*inliers)[i]) continue;

		// Triangulate

		const Vector2d& y1 = y1s[i];
		const Vector2d& y2 = y2s[i];

		Vector3d X = mpt.triangulate(y1, y2);


		// Validate

		if (std::isnan(X(0)) || std::isnan(X(1)) || std::isnan(X(2)) ||
            std::isinf(X(0)) || std::isinf(X(1)) || std::isinf(X(2))||(X(2)<0))
		{
			(*inliers)[i] = false;
			continue;
		}

		double e1 = (y1 - X.hnormalized()).norm();
		double e2 = (y2 - (P2 * X).hnormalized()).norm();

		if (e1 + e2 > 2 * p.max_reproj_error) {
			(*inliers)[i] = false;
			continue;
		}

		// Store

		observations1.push_back(y1s[i]);
		observations2.push_back(y2s[i]);
		triangulations.push_back(X);
		(*inliers)[i] = true;
	}
	n = triangulations.size();

	// Prepare solver

	ceres::Problem problem;
	ceres::Solver::Options options;

	options.linear_solver_type = ceres::SPARSE_SCHUR;
	options.minimizer_progress_to_stdout = false;

    options.max_num_iterations = 15;
	options.function_tolerance = 1e-8;     // default 1e-6
	options.gradient_tolerance = 1e-10;     //default 1e-4*function_tolerance

    //ceres::LossFunction* loss = nullptr;
    ceres::LossFunction* loss = new ceres::HuberLoss(p.max_reproj_error);

	// P1 is [I|0]
	double *P2_q = P2.getRRef();
	double *P2_t = P2.getTRef();

	for (size_t i = 0; i < n; i++) {

		Vector2d& y1 = observations1[i];
		Vector2d& y2 = observations2[i];
		Vector3d& X = triangulations[i];

		ceres::CostFunction* cost_fn = PointReprojectionError::Create(y1, y2);
		problem.AddResidualBlock(cost_fn, loss, P2_q, P2_t, &X(0));
	}
    problem.AddResidualBlock(UnitLengthError<3>::Create(),nullptr,P2_t);

	ceres::ParameterBlockOrdering *ordering = new ceres::ParameterBlockOrdering();
	ordering->AddElementToGroup(P2_q, 1);
	ordering->AddElementToGroup(P2_t, 2);
	for (size_t i = 0; i < n; i++) {
		Vector3d& X = triangulations[i];
		ordering->AddElementToGroup(&X(0), 0);
    }

    options.linear_solver_ordering.reset(ordering);

	problem.SetParameterization(P2_q, new ceres::QuaternionParameterization());

	// Optimize

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
    //cout << summary.FullReport() << endl;

	// P2 already has the output since Ceres modifies it directly.
}
