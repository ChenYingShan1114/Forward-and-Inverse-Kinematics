#include "catmull_rom_interpolation.h"
#include <Eigen/Dense>
#include <iostream>

// reference https://en.wikipedia.org/wiki/Catmull–Rom_spline
Eigen::Vector3d catmull_rom_interpolation(
  const std::vector<std::pair<double, Eigen::Vector3d> > & keyframes,
  double t)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
  
  
  // if (keyframes.size() > 1) {
  if (keyframes.size() > 3) {
    
    // find t's index
    int index = -1;
    for (int i = 0; i < keyframes.size() - 1; i++) {
      if (t > keyframes[i].first && t < keyframes[i+1].first) {
        index = i;
        break;
      }
    }

    if (index != -1) {
      // Catmull-Rom interpolation
      double t_nor = (t - keyframes[index].first) / (keyframes[index + 1].first - keyframes[index].first);
      Eigen::VectorXd t_vec(4);
      t_vec << pow(t_nor, 3), pow(t_nor, 2), t_nor, 1;

      Eigen::MatrixXd coeff(4, 4);
      coeff << -1,  3, -3,  1,
                2, -5,  4, -1,
               -1,  0,  1,  0,
                0,  2,  0,  0;  

      Eigen::Vector3d p0, p1, p2, p3;
      // Method 1 to deal with BC
      // if (index == 0) {
      //   p0 = keyframes[keyframes.size() - 1].second; // periodic BC
      //   p1 = keyframes[index].second;
      //   p2 = keyframes[index + 1].second;
      //   p3 = keyframes[index + 2].second;
      // } else if (index == keyframes.size() - 1) {
      //   p0 = keyframes[index - 1].second;
      //   p1 = keyframes[index].second;
      //   p2 = keyframes[0].second; // periodic BC
      //   p3 = keyframes[1].second; // periodic BC
      // } else if (index == keyframes.size() - 2) {
      //   p0 = keyframes[index - 1].second;
      //   p1 = keyframes[index].second;
      //   p2 = keyframes[index + 1].second;
      //   p3 = keyframes[0].second; // periodic BC
      // } else {
      //   p0 = keyframes[index - 1].second;
      //   p1 = keyframes[index].second;
      //   p2 = keyframes[index + 1].second;
      //   p3 = keyframes[index + 2].second;
      // }
      
      // Method 2 to deal with BC
      if (index == 0) {
        p0 = keyframes[keyframes.size() - 1].second; // periodic BC
      } else {
        p0 = keyframes[index - 1].second;
      }

      p1 = keyframes[index].second;

      if (index == keyframes.size() - 1) {
        p2 = keyframes[0].second; // periodic BC
      } else {
        p2 = keyframes[index + 1].second;
      }

      if (index == keyframes.size() - 1) {
        p3 = keyframes[1].second; // periodic BC
      } else if (index == keyframes.size() - 2) {
        p3 = keyframes[0].second; // periodic BC
      } else {
        p3 = keyframes[index + 2].second;
      }

      Eigen::MatrixXd p(4, 3);
      p << p0.transpose(), p1.transpose(), p2.transpose(), p3.transpose();
      return 0.5 * t_vec.transpose() * coeff * p;

    } else {
      return Eigen::Vector3d(0,0,0);
    }

  } 
  else if ( keyframes.size() == 2 || keyframes.size() == 3) { // test case no this condition
    std::cout << "Catmull-Rom interpolation method needs more than 3 keyframes. Now keyframes: " << keyframes.size() << std::endl;
    std::cout << "Change into linear interpolation" << std::endl;
    // find t's index
    int index = -1;
    for (int i = 0; i < keyframes.size() - 1; i++) {
      if (t > keyframes[i].first && t < keyframes[i+1].first) {
        index = i;
        break;
      }
    }
    if (index != -1) {
      // linear interpolation
      return keyframes[index].second + (t - keyframes[index].first) * (keyframes[index + 1].second - keyframes[index].second) / (keyframes[index + 1].first - keyframes[index].first);
    } else {
      return Eigen::Vector3d(0,0,0);
    }

  } else if (keyframes.size() == 1) { // test case no this condition
    return keyframes[0].second;
  } else {
    return Eigen::Vector3d(0,0,0);
  }
  /////////////////////////////////////////////////////////////////////////////
}
