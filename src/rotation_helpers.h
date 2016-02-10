#pragma once
#include <eigen3/Eigen/Eigen>

Eigen::Matrix3d getRotationMatrix(const Eigen::Vector4d& q);
Eigen::Vector4d getRotationQuaternion(const Eigen::Matrix3d& R);
Eigen::Matrix3d normalizeRotationMatrix(const Eigen::Matrix3d& R);
Eigen::Vector3d quaternionRotate(const double* q, const Eigen::Vector3d& x);

bool isRotationMatrix(const Eigen::Matrix3d& R);
bool isAlmostRotationMatrix(const Eigen::Matrix3d& R);
bool isRotationQuaternion(const Eigen::Vector4d& q);
bool isAlmostRotationQuaternion(const Eigen::Vector4d& q);
