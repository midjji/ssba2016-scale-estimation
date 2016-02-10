#include <cassert>
#include <iostream>
#include "rotation_helpers.h"

using std::cout;
using std::endl;

using namespace Eigen;

bool isRotationMatrix(const Matrix3d& R)
{
    return (std::abs((R.determinant() - 1.0)) < 1e-7);
}

bool isAlmostRotationMatrix(const Matrix3d& R)
{
    return (std::abs((R.determinant() - 1.0)) < 1e-5);
}

bool isRotationQuaternion(const Vector4d& q)
{
    return (std::abs((q.norm() - 1.0)) < 1e-14);
}

bool isAlmostRotationQuaternion(const Vector4d& q)
{
    return (std::abs((q.norm() - 1.0)) < 1e-5);
}

Matrix3d getRotationMatrix(const Vector4d& q)
{
    assert(isAlmostRotationQuaternion(q));
    //q.normalize();
    double aa = q[0] * q[0];
    double ab = q[0] * q[1];
    double ac = q[0] * q[2];
    double ad = q[0] * q[3];
    double bb = q[1] * q[1];
    double bc = q[1] * q[2];
    double bd = q[1] * q[3];
    double cc = q[2] * q[2];
    double cd = q[2] * q[3];
    double dd = q[3] * q[3];

    double a = aa + bb - cc - dd;
    double b = 2 * (ad + bc);
    double c = 2 * (bd - ac);
    double d = 2 * (bc - ad);
    double e = aa - bb + cc - dd;
    double f = 2 * (ab + cd);
    double g = 2 * (ac + bd);
    double h = 2 * (cd - ab);
    double i = aa - bb - cc + dd;

    Matrix3d R;
    R <<
        a, d, g,
        b, e, h,
        c, f, i;
    return R;
}

// Convert a rotation matrix to a quaternion. The quaternion is on a form that matches Ceres-solver
Vector4d getRotationQuaternion(const Matrix3d& R)
{
    Vector4d q;
    assert(isAlmostRotationMatrix(R));
    double S;

    double tr = R.trace() + 1.0;
    if (tr > 1e-12) {
        S = 0.5 / std::sqrt(tr);
        q[0] = 0.25 / S;
        q[1] = (R(2,1) - R(1,2)) * S;
        q[2] = (R(0,2) - R(2,0)) * S;
        q[3] = (R(1,0) - R(0,1)) * S;
        // does not work for a 45 degree rot around y...
    }
    else {
        // If the trace of the matrix is equal to or close to zero,
        // then identify which major diagonal element has the greatest value.

        // Depending on this, calculate the following:

        if ((R(0,0) > R(1,1)) && (R(0,0) > R(2,2)))  {  // Column 0:
            S = std::sqrt(1.0 + R(0,0) - R(1,1) - R(2,2)) * 2;
            q[0] = (R(2,1) - R(1,2)) / S;
            q[1] = 0.25 * S;
            q[2] = (R(1,0) + R(0,1)) / S;
            q[3] = (R(0,2) + R(2,0)) / S;
        }
        else if (R(1,1) > R(2,2)) {   // Column 1:
            S = std::sqrt(1.0 + R(1,1) - R(0,0) - R(2,2)) * 2;
            q[0] = (R(0,2) - R(2,0)) / S;
            q[1] = (R(1,0) + R(0,1)) / S;
            q[2] = 0.25 * S;
            q[3] = (R(2,1) + R(1,2)) / S;
        }
        else {  // Column 2:
            S = std::sqrt(1.0 + R(2,2) - R(0,0) - R(1,1)) * 2;
            q[0] = (R(1,0) - R(0,1)) / S;
            q[1] = (R(0,2) + R(2,0)) / S;
            q[2] = (R(2,1) + R(1,2)) / S;
            q[3] = 0.25 * S;
        }
    }

    if (std::abs(q.norm() - 1) >= 1e-6) {
        cout << "q.len-1: " << q.norm() - 1 << endl;
        assert(std::abs(q.norm() - 1) < 1e-6);
    }

    q.normalize();
    return q;
}

Matrix3d normalizeRotationMatrix(const Matrix3d& R)
{
    assert(isAlmostRotationMatrix(R));

    Vector3d a, b, c;
    a(0) = R(0,0);  b(0) = R(0,1);  c(0) = R(0,2);
    a(1) = R(1,0);  b(1) = R(1,1);  c(1) = R(1,2);
    a(2) = R(2,0);  b(2) = R(2,1);  c(2) = R(2,2);
    a.normalize();
    b = b - b.dot(a) * a;
    b.normalize();
    c = a.cross(b);

    Matrix3d Rn;
    Rn <<
        a(0), b(0), c(0),
        a(1), b(1), c(1),
        a(2), b(2), c(2);
    return Rn;
}

Vector3d quaternionRotate(const double* q, const Vector3d& x)
{
    const double t2 =  q[0] * q[1];
    const double t3 =  q[0] * q[2];
    const double t4 =  q[0] * q[3];
    const double t5 = -q[1] * q[1];
    const double t6 =  q[1] * q[2];
    const double t7 =  q[1] * q[3];
    const double t8 = -q[2] * q[2];
    const double t9 =  q[2] * q[3];
    const double t1 = -q[3] * q[3];

    Vector3d out(
        (2 * ((t8 + t1) * x(0) + (t6 - t4) * x(1) + (t3 + t7) * x(2)) + x(0)),
        (2 * ((t4 + t6) * x(0) + (t5 + t1) * x(1) + (t9 - t2) * x(2)) + x(1)),
        (2 * ((t7 - t3) * x(0) + (t2 + t9) * x(1) + (t5 + t8) * x(2)) + x(2)));
    return out;
}
