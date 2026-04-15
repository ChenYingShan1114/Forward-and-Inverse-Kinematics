#include "line_search.h"
#include <iostream>

double line_search(
  const std::function<double(const Eigen::VectorXd &)> & f,
  const std::function<void(Eigen::VectorXd &)> & proj_z,
  const Eigen::VectorXd & z,
  const Eigen::VectorXd & dz,
  const double max_step)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
  double alpha = 10000;
  for (int i = 0; i < max_step; i++) {

    Eigen::VectorXd z0 = z;
    Eigen::VectorXd z_test = z + alpha * dz;
    proj_z(z_test);

    if (f(z_test) > f(z)) {
      alpha = alpha / 2;
    }

  }
  return alpha;
  /////////////////////////////////////////////////////////////////////////////
}
