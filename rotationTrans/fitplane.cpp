#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>

void fitPlane(std::vector<Eigen::Vector3d> points)
{
    Eigen::MatrixXd mat_A(points.size(), 3);
    Eigen::VectorXd mat_B(points.size());
    for (unsigned int i = 0; i < points.size(); i++)
    {
        Eigen::Vector3d point =points[i];
        mat_A(i, 0) = point[0];
        mat_A(i, 1) = point[1];
        mat_A(i, 2) = point[2];
        mat_B(i) = -1.0;
    }
    Eigen::Vector3d plane_ceoff = mat_A.colPivHouseholderQr().solve(mat_B);
    std::cout<<plane_ceoff.transpose()/plane_ceoff.norm()<<std::endl;
}


int main()
{
  std::vector<Eigen::Vector3d> points;
  points.push_back(Eigen::Vector3d(5.1481, 0.46761, -0.81268));
  points.push_back(Eigen::Vector3d(3.6717, -1.1803, -0.74225));
  points.push_back(Eigen::Vector3d(1.5946, 4.8757, -0.80642));
  points.push_back(Eigen::Vector3d(0.94173, -3.8254, -0.61792));
  points.push_back(Eigen::Vector3d(1.6228, 2.4005, -0.7656));
  points.push_back(Eigen::Vector3d(3.5338, 4.0523, -0.84553));

  fitPlane(points);

  return 0;
}
