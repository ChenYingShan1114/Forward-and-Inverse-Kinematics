#include "end_effectors_objective_and_gradient.h"
#include "transformed_tips.h"
#include "kinematics_jacobian.h"
#include "copy_skeleton_at.h"
#include <iostream>

void end_effectors_objective_and_gradient(
  const Skeleton & skeleton,
  const Eigen::VectorXi & b,
  const Eigen::VectorXd & xb0,
  std::function<double(const Eigen::VectorXd &)> & f,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> & grad_f,
  std::function<void(Eigen::VectorXd &)> & proj_z)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code

  f = [&](const Eigen::VectorXd & A)->double  //   A is xzx angle. z  #z vector of optimized z values
  {
    
    // put the initial guess angle A into skeleton_tmp with the angles constrain
    Skeleton skeleton_guess = copy_skeleton_at(skeleton, A);

    // use new angles to calculate tips position
    Eigen::VectorXd xb_guess = transformed_tips(skeleton_guess, b);
    
    // calculate squareNorm between guess position and goal position 
    return (xb_guess - xb0).squaredNorm();
  };
  
  grad_f = [&](const Eigen::VectorXd & A)->Eigen::VectorXd
  {
    // put the initial guess angle A into skeleton_tmp with the angles constrain
    Skeleton skeleton_guess = copy_skeleton_at(skeleton, A);

    // use new angles to calculate tips position
    Eigen::VectorXd xb_guess = transformed_tips(skeleton_guess, b);
    
    // calculate dE/dx
    Eigen::VectorXd dE = 2 * (xb_guess - xb0);

    // calculate Jacobian dx/da
    Eigen::MatrixXd J;
    kinematics_jacobian(skeleton_guess, b, J);

    // Newton method
    // return J.transpose() * dE;

    // Gauss-Newton method: inverse diverse quickly
    // return (J.transpose() * J).inverse() * J.transpose() * dE;

    // Leverberg-Marquadt algorithm: https://en.wikipedia.org/wiki/Levenberg-Marquardt_algorithm
    double lambda = 0.0001;
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(J.cols(), J.cols());
    return (J.transpose() * J + lambda * I).inverse() * J.transpose() * dE;
  };

  proj_z = [&](Eigen::VectorXd & A)
  {

    for (int i = 0; i < skeleton.size(); i++) {

      // slamp the angle in the joint limits angle
      A[3 * i    ] = std::min(std::max(A[3 * i    ], skeleton[i].xzx_min[0]), skeleton[i].xzx_max[0]);
      A[3 * i + 1] = std::min(std::max(A[3 * i + 1], skeleton[i].xzx_min[1]), skeleton[i].xzx_max[1]);
      A[3 * i + 2] = std::min(std::max(A[3 * i + 2], skeleton[i].xzx_min[2]), skeleton[i].xzx_max[2]);

    }
  };
  /////////////////////////////////////////////////////////////////////////////
}
