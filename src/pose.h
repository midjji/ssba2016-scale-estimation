#pragma once
#include <eigen3/Eigen/Eigen>
#include "common.h"

class Pose
{
public:    
    Pose();

    Pose(const Eigen::Matrix3d& R);
    Pose(const Eigen::Vector3d& t);
    Pose(const Eigen::Matrix3d& R, const Eigen::Vector3d& t);
    Pose(const Eigen::Vector4d& q, const Eigen::Vector3d& t);
    Pose(const Matrix34d& P);
    Pose(const Eigen::Matrix4d& P);

    ~Pose();

    /**
     * @brief getRRef
     * @return a pointer to the first element of the quaternion in the pose
     */
    double* getRRef();
    /**
     * @brief getTRef
     * @return a pointer to the first element of the translation in the pose
     */
    double* getTRef();
    void setT(const Eigen::Vector3d& tt);
    void setQuaternion(const Eigen::Vector4d& qq);


    Eigen::Vector3d operator*(const Eigen::Vector3d& ins) const;

    Pose operator*(const Pose& rhs) const;
    Pose inverse() const;
    double angleDistance(const Pose& p) const;

    Eigen::Vector4d getQuaternion() const;
    Eigen::Matrix3d getR() const;
    Eigen::Matrix3d rotation() const;

    Eigen::Vector3d getT() const;
    Eigen::Vector3d translation() const;
    Eigen::Vector3d& translation();

    Matrix34d get3x4() const;
    Eigen::Matrix4d get4x4() const;
    double getAngle() const;

    Eigen::Matrix3d getEssentialMatrix() const;

    bool noRotation() const;
    bool isIdentity() const;
    /// returns true if no value is strange
    bool isnormal() const;

    void scaleT(double scale);

    void normalize();
    Eigen::Vector3d rotate(const Eigen::Vector3d& x) const;
    void rotateInPlace(Eigen::Vector3d& x) const;

private:
    Eigen::Vector4d q; // Rotation quaternion. Coefficients are ordered as (scalar, i, j, k)
    Eigen::Vector3d t;
};


