#include "forward_kinematics.h"
#include "euler_angles_to_transform.h"
#include <functional> // std::function

// T of each bone represent the affine transformation matrix related to rest bone not canonical bone
// Initial all T are identity matrix

void forward_kinematics(
  const Skeleton & skeleton,
  std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > & T)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
  T.resize(skeleton.size(),Eigen::Affine3d::Identity());

  for (int i = 0; i < skeleton.size(); i++) {
    int p = skeleton[i].parent_index;
    Eigen::Affine3d R_relative = euler_angles_to_transform(skeleton[i].xzx);

    if (p == -1) {
      T[i] = skeleton[i].rest_T.matrix() * R_relative.matrix() * skeleton[i].rest_T.matrix().inverse();
    } else {
      T[i] = T[p].matrix() * skeleton[i].rest_T.matrix() * R_relative.matrix() * skeleton[i].rest_T.matrix().inverse();
    }
  }
  /////////////////////////////////////////////////////////////////////////////
}
