#include <eigen3/Eigen/SVD>
#include <ceres/ceres.h>
#include "geometry_tools.h"

using namespace std;
using namespace Eigen;

Eigen::Matrix3d crossMatrix(const Eigen::Vector3d& v)
{
    Eigen::Matrix3d M;

    M <<
        0, -v(2), v(1),
        v(2), 0, -v(0),
        -v(1), v(0), 0;

    return M;
}

//// Essential matrix tools ////////////////////////////////////////////////

double getEpipolarLineDistance(const Matrix3d& E,
	const Vector2d& x1, const Vector2d& x2)
{
	Vector3d L = E * x1.homogeneous();
	double Z = sqrt(L(0)*L(0) + L(1)*L(1));
	return x2.homogeneous().dot(L / Z);
}

vector<Pose> extractCamerasFromE(const Matrix3d& E)
{
	vector<Pose> cameras;

	// Reference: H&Z pp. 258-260

	Pose P1 = Pose(); // First camera: [I|0] transformation
	Pose P2; // Second camera: [R|t] - sought

	// Decompose E

	Matrix3d U, V;
	Eigen::JacobiSVD<Matrix3d > svd(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
	//Vec3 s = svd.singularValues(); // TODO: Verify that E is a matrix, s.t s(0) == s(1), s(2) = 0 (?)
	U = svd.matrixU();
	V = svd.matrixV();

	// Generate possible solutions

    Matrix3d W;
    W <<
		0, -1, 0,
		1, 0, 0,
		0, 0, 1;

	Matrix3d R1 = U * W * V.transpose();
	R1 = R1 * R1.determinant(); // Ensure det(R1) = 1
	Matrix3d R2 = U * W.transpose() * V.transpose();
	R2 = R2 * R2.determinant();
	Vector3d u3 = U.col(2);

	cameras.push_back(Pose(R1, u3));
	cameras.push_back(Pose(R1, -u3));
	cameras.push_back(Pose(R2, u3));
	cameras.push_back(Pose(R2, -u3));

	return cameras;
}

uint getEpipolarLineInliers(double max_d, const std::vector<Vector2d>& y1s, const std::vector<Vector2d>& y2s,
	const Matrix3d& E, std::vector<bool>& inliers)
{
	const uint num_points = (uint)y1s.size();
	assert(y2s.size() == num_points);
	inliers.resize(num_points);

	uint inlier_cnt = 0;
	for (uint j = 0; j < num_points; j++) {
		double d = fabs(getEpipolarLineDistance(E, y1s[j], y2s[j]));
		inliers[j] = (d < max_d);
		if (inliers[j]) {
			inlier_cnt++;
		}
	}

	return inlier_cnt;
}

uint getReprojectionInliers(double max_e, const std::vector<Vector2d>& y1s, const std::vector<Vector2d>& y2s,
	const Pose& P2, std::vector<bool>& inliers)
{
	const uint num_points = (uint)y1s.size();
	assert(y2s.size() == num_points);
	inliers.resize(num_points);

	MidpointTriangulator mpt(Pose(), P2);

	uint inlier_cnt = 0;
	uint i1 = 0, i2 = 0, i3 = 0;
	for (uint i = 0; i < num_points; i++) {

		if (inliers[i] == false) continue;

		inliers[i] = false;

		Vector2d y1 = y1s[i];
		Vector2d y2 = y2s[i];

		Vector3d X = mpt.triangulate(y1, y2);

		if (std::isnan(X(0)) || std::isnan(X(1)) || std::isnan(X(2)) ||
			std::isinf(X(0)) || std::isinf(X(1)) || std::isinf(X(2)))
		{
			inliers[i] = false;
			continue;
		}

		i1++;

		// Reproject and check check camera P1 = [I|0]

		Vector3d x1 = X; // x1 = [I|0] * X

		if (x1(2) < 0) {
			continue; // Behind camera
		}

		i2++;

		if ((y1 - x1.hnormalized()).norm() > max_e) {
			continue;
		}

		// Reproject and check camera P2

		Vector3d x2 = P2 * X;

		if (x2(2) < 0) {
			continue; // Behind camera
		}

		if ((y2 - x2.hnormalized()).norm() > max_e) {
			continue;
		}

		inliers[i] = true;
		inlier_cnt++;
		i3++;
	}

	//fprintf(stderr, "i1=%d i2=%d i3=%d --\n", i1, i2, i3);

	return inlier_cnt;
}

int selectCameraFromE(const std::vector<Pose>& cameras, const Vector2d& y1, const Vector2d& y2)
{
	Pose P1;

	for (int i = 0; i < 4; i++) {
		const Pose& P2 = cameras[i];

		Vector3d X = MidpointTriangulator::triangulate(P1, P2, y1, y2);
		Vector3d x1 = P1 * X;
		if (x1(2) > 0) {
			Vector3d x2 = P2 * X;
			if (x2(2) > 0) {
				return i;
			}
		}
	}
	return -1;
}

uint selectCameraFromE(double max_e, const std::vector<Pose>& cameras,
	const std::vector<Vector2d>& y1s, const std::vector<Vector2d>& y2s,
	std::vector<bool>& inliers, uint& inlier_count)
{
	vector<bool> best_iflags(y1s.size(), false);
	uint best_cam_ix = 0;
	uint best_icnt = 0;
	for (uint j = 0; j < 4; j++) {
		vector<bool> iflags = inliers; // Copy
		uint icnt = getReprojectionInliers(max_e, y1s, y2s, cameras[j], iflags);
		if (icnt > best_icnt) {
			best_icnt = icnt;
			best_iflags = iflags;
			best_cam_ix = j;
		}
	}

	if (best_iflags.empty()) {
		// Found no inliers whatsoever.
		inliers = vector<bool>(y1s.size(), false);
	}
	else {
		inliers = best_iflags;
	}

	inlier_count = best_icnt;
	return best_cam_ix;
}

Pose extractCameraFromE(const Matrix3d& E, const Vector2d& y1n, const Vector2d& y2n)
{
	// Reference: H&Z pp. 258-260

	Pose P1 = Pose(); // First camera: [I|0] transformation
	Pose P2; // Second camera: [R|t] - sought

	// Decompose E

	Matrix3d U, V;
	Eigen::JacobiSVD<Matrix3d> svd(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
	//Vec3 s = svd.singularValues(); // TODO: Verify that E is a matrix, s.t s(0) == s(1), s(2) = 0 (?)
	U = svd.matrixU();
	V = svd.matrixV();

	// Generate possible solutions

    Matrix3d W;
    W <<
		0, -1, 0,
		1, 0, 0,
		0, 0, 1;

	Matrix3d R1 = U * W * V.transpose();
	R1 = R1 * R1.determinant(); // Ensure det(R1) = 1
	Matrix3d R2 = U * W.transpose() * V.transpose();
	R2 = R2 * R2.determinant();
	Vector3d u3 = U.col(2);

	// Test and select one of the solutions

	for (int i = 0; i < 4; i++) {
		switch (i) {
		case 0: P2 = Pose(R1, u3); break;
		case 1: P2 = Pose(R1, -u3); break;
		case 2: P2 = Pose(R2, u3); break;
		case 3: P2 = Pose(R2, -u3); break;
		}

        Vector3d X = MidpointTriangulator::triangulate(P1, P2, y1n, y2n);
		Vector3d x1 = P1 * X;
		if (x1(2) > 0) {
			Vector3d x2 = P2 * X;
			if (x2(2) > 0) {
				return P2;
			}
		}
	}

	return P2; // Never reached (?)
}

//// 2-view midpoint triangulation /////////////////////////////////////////

class TriangulationError
{
public:
    TriangulationError(Matrix4d P, Vector2d yn)
    {
        this->P = P;
        this->yn = yn;
    }

    template <typename T>
    bool operator()(const T* const x, T* residuals) const {

        T xr[3];
        for (uint i = 0; i<3; ++i)
            xr[i] = T(0);
        for (uint i = 0; i<3; ++i){
            for (uint j = 0; j<3; ++j)
                xr[i] += T(P(i,j))*x[j];
            xr[i] += T(P(i,3));
        }
        //xr[3] = ceres::abs(xr[3]);

        // The error is the difference between the predicted && observed position.
        T xp = (xr[0] / xr[2]);
        T yp = (xr[1] / xr[2]);

        residuals[0] = xp - T(yn[0]);
        residuals[1] = yp - T(yn[1]);
        return true;
    }

    static ceres::CostFunction* Create(Matrix4d P, Vector2d yn)
    {
        // resid,first param,second param, third param
        return (new ceres::AutoDiffCostFunction<TriangulationError, 2, 3>(
            new TriangulationError(P, yn)));
    }

    Matrix4d P;
    Vector2d yn;
};

void MidpointTriangulator::refine(Vector2d y1, Vector2d y2, Vector3d& X) const
{
    ceres::Problem problem;
    problem.AddResidualBlock(TriangulationError::Create(P1.get4x4(), y1), nullptr, X.data());
    problem.AddResidualBlock(TriangulationError::Create(P2.get4x4(), y2), nullptr, X.data());

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 3;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    //cout<<summary.FullReport();
}

MidpointTriangulator::MidpointTriangulator(const Pose& P1, const Pose& P2, double d)
	: P1(P1),
	pose1(P1.inverse()),
	c1(pose1.translation()),
	P2(P2),
	pose2(P2.inverse()),
	c2(pose2.translation()),
	far_distance(d)
{
}

Vector3d MidpointTriangulator::triangulate(const Pose& P1, const Pose& P2, const Vector2d& y1, const Vector2d& y2, double d)
{
    return MidpointTriangulator(P1, P2, d).triangulate(y1, y2);
}

Vector3d MidpointTriangulator::triangulate(const Vector2d& y1, const Vector2d& y2) const
{
	Vector3d X;
	Vector3d pa, pb;
	double mua, mub;

	// Create lines in the world frame, from the camera centers to the observed 2D points y1,y2

	Vector3d y1c1 = pose1 * y1.homogeneous(); // point from camera 1, moved into the world frame
	Vector3d y2c2 = pose2 * y2.homogeneous(); // point from camera 2, moved into the world frame

	if (!lineLineIntersect(c1, y1c1, c2, y2c2, pa, pb, mua, mub)) {
		// No midpoint found, place the 3D point "far away", in front of a camera.
		X = pose2 * Vector3d(0, 0, far_distance);
        //refine(y1, y2, X);
	}
	else {
		X = 0.5 * (pa + pb);
	}

	Vector3d x1 = P1 * X;
	Vector3d x2 = P2 * X;

	if (x1(2) < 0 && x2(2) < 0) {
		// The 3D point is behind both cameras, probably due to
		// uncertainty in the observations of a far-away image feature.
		// Fix this by placing the point in front of one camera.
		x1(2) = -x1(2);
		X = pose1 * x1;
	}

    //refine(y1, y2, X);
	return X;
}

bool MidpointTriangulator::lineLineIntersect(const Vector3d& p1, const Vector3d& p2,
	const Vector3d& p3, const Vector3d& p4, Vector3d& pa, Vector3d& pb, double& mua, double& mub) const
{
	Vector3d p13, p43, p21;
	double d1343, d4321, d1321, d4343, d2121;
	double numer, denom;

	const double eps = std::numeric_limits<double>::epsilon();

	p13 = p1 - p3;

	p43 = p4 - p3;
	if (fabs(p43(0)) + fabs(p43(1)) + fabs(p43(2)) < 3 * eps) {
		return false;
	}

	p21 = p2 - p1;
	if (fabs(p21(0)) + fabs(p21(1)) + fabs(p21(2)) < 3 * eps) {
		return false;
	}

	d1343 = p13.dot(p43);
	d4321 = p43.dot(p21);
	d1321 = p13.dot(p21);
	d4343 = p43.dot(p43);
	d2121 = p21.dot(p21);

	denom = d2121 * d4343 - d4321 * d4321;
	if (fabs(denom) < eps) {
		return false;
	}

	numer = d1343 * d4321 - d1321 * d4343;

	mua = numer / denom;
	mub = (d1343 + d4321 * mua) / d4343;

	pa = p1 + mua * p21;
	pb = p3 + mub * p43;

	return true;
}
