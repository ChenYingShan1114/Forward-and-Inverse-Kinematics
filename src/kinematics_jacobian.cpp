#include "kinematics_jacobian.h"
#include "transformed_tips.h"
#include <iostream>

void kinematics_jacobian(
  const Skeleton & skeleton,
  const Eigen::VectorXi & b,
  Eigen::MatrixXd & J)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code

  J.resize(3 * b.size(), 3 * skeleton.size());
  J.setZero();
  double h = 1.0e-7;

  // x(a)
  Eigen::VectorXd xb = transformed_tips(skeleton, b);

  for (int j = 0; j < skeleton.size(); j++) {

    // x(a + h * dx)
    Skeleton skeleton_dx = skeleton;
    skeleton_dx[j].xzx[0] += h;
    Eigen::VectorXd xb_x = transformed_tips(skeleton_dx, b);
    Eigen::VectorXd J_dx = (xb_x - xb) / h;

    // x(a + h * dy)
    Skeleton skeleton_dy = skeleton;
    skeleton_dy[j].xzx[1] += h;
    Eigen::VectorXd xb_y = transformed_tips(skeleton_dy, b);
    Eigen::VectorXd J_dy = (xb_y - xb) / h;

    // x(a + h * dz)
    Skeleton skeleton_dz = skeleton;
    skeleton_dz[j].xzx[2] += h;
    Eigen::VectorXd xb_z = transformed_tips(skeleton_dz, b);
    Eigen::VectorXd J_dz = (xb_z - xb) / h;

    for (int i = 0; i < b.size(); i++) {

      // angle x
      J(3 * i    , 3 * j    ) = J_dx(3 * i    );
      J(3 * i + 1, 3 * j    ) = J_dx(3 * i + 1);
      J(3 * i + 2, 3 * j    ) = J_dx(3 * i + 2);

      // angle y
      J(3 * i    , 3 * j + 1) = J_dy(3 * i    );
      J(3 * i + 1, 3 * j + 1) = J_dy(3 * i + 1);
      J(3 * i + 2, 3 * j + 1) = J_dy(3 * i + 2);

      // angle z
      J(3 * i    , 3 * j + 2) = J_dz(3 * i    );
      J(3 * i + 1, 3 * j + 2) = J_dz(3 * i + 1);
      J(3 * i + 2, 3 * j + 2) = J_dz(3 * i + 2);

    }
  }
  /////////////////////////////////////////////////////////////////////////////
}
