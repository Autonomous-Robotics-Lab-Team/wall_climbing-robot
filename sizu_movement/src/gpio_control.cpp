#include <ros/ros.h>
#include <string>
#include <math.h>
#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>


using Eigen::MatrixXd;
using namespace std;

int main() {

Eigen::Vector3d eulerAngle(M_PI/4,0,0);

Eigen::Matrix<double,3,1> P;
P<<1,
   1,
   0;

Eigen::AngleAxisd rollAngle(eulerAngle(2), Eigen::Vector3d::UnitX());
Eigen::AngleAxisd pitchAngle(eulerAngle(1), Eigen::Vector3d::UnitY());
Eigen::AngleAxisd yawAngle(eulerAngle(0), Eigen::Vector3d::UnitZ());
 
Eigen::Matrix3d rotation_matrix;
rotation_matrix=yawAngle*pitchAngle*rollAngle;

// rotation_matrix * P produces a 3x1 vector, so store result in a 3x1 type
Eigen::Vector3d P_rotated;
P_rotated = rotation_matrix * P;

std::cout<<"rotation_matrix is :\n"<<rotation_matrix<<std::endl;

std::cout<<"rotation_matrix is :\n"<<P_rotated<<std::endl;
return 0;
}