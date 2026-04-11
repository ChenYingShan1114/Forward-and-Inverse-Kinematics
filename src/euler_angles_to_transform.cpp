#include "euler_angles_to_transform.h"

Eigen::Affine3d euler_angles_to_transform(
  const Eigen::Vector3d & xzx)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code

  // 0. twist
  double theta1 = M_PI / 180 * xzx[0];
  Eigen::Matrix3d R1;
  R1 << 1,                0,                 0,
        0, std::cos(theta1), -std::sin(theta1),
        0, std::sin(theta1),  std::cos(theta1);

  // 0. bend
  double theta2 = M_PI / 180 * xzx[1];
  Eigen::Matrix3d R2;
  R2 << std::cos(theta2), -std::sin(theta2), 0,
        std::sin(theta2),  std::cos(theta2), 0,
                       0,                 0, 1;

  // 0. twist
  double theta3 = M_PI / 180 * xzx[2];
  Eigen::Matrix3d R3;
  R3 << 1,                0,                 0,
        0, std::cos(theta3), -std::sin(theta3),
        0, std::sin(theta3),  std::cos(theta3);

  // 1. Define rotation (as a Matrix3d) and translation (as a Vector3d)
  Eigen::Matrix3d rotation = R3 * R2 * R1;
  // Eigen::Vector3d translation(0.0, 0.0, 0.0);

  // 2. Define the Affine3d
  Eigen::Affine3d aff = Eigen::Affine3d::Identity();
  aff.linear() = rotation;
  // aff.translation() = translation;
  return aff;
  /////////////////////////////////////////////////////////////////////////////
}
