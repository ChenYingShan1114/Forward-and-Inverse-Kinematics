#include "linear_blend_skinning.h"

void linear_blend_skinning(
  const Eigen::MatrixXd & V,
  const Skeleton & skeleton,
  const std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > & T,
  const Eigen::MatrixXd & W,
  Eigen::MatrixXd & U)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
  
  for (int i = 0; i < V.rows(); i++) { // which vertex

    Eigen::Vector3d V_i = V.row(i);

    // initial V_new
    Eigen::Vector3d V_new = Eigen::Vector3d::Zero();
    for (int j = 0; j < skeleton.size(); j++) { // which bone
      if (skeleton[j].weight_index != -1) {
        double weight = W(i, skeleton[j].weight_index);
        Eigen::Vector3d V_tmp = T[j] * V_i;
        V_new += weight * V_tmp;
      }
    }
    U.row(i) = V_new;
  }  
  /////////////////////////////////////////////////////////////////////////////
}
