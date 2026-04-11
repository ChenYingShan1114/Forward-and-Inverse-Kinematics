#include "transformed_tips.h"
#include "forward_kinematics.h"

Eigen::VectorXd transformed_tips(
  const Skeleton & skeleton, 
  const Eigen::VectorXi & b)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
  std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d>> T;
  forward_kinematics(skeleton, T);

  Eigen::VectorXd xb0;
  xb0.resize(3 * b.size());

  for (int i = 0; i < b.size(); i++) {
    int skeleton_index = b(i);

    // end bone's tall at rest position
    Eigen::Vector3d rest_bone_tall = skeleton[skeleton_index].rest_T.translation();

    // end bone's tip at canonical position
    Eigen::Vector3d bone_tip;
    bone_tip << skeleton[skeleton_index].length, 0, 0;

    // end bone's tip at rest position
    Eigen::Vector3d rest_bone_tip = skeleton[skeleton_index].rest_T * bone_tip;

    // end bone's tip at pose position
    Eigen::Vector3d pose_bone_tip = T[skeleton_index] * rest_bone_tip;
    
    xb0[3 * i] = pose_bone_tip[0];
    xb0[3 * i + 1] = pose_bone_tip[1];
    xb0[3 * i + 2] = pose_bone_tip[2];

  }

  return xb0;
  /////////////////////////////////////////////////////////////////////////////
}
