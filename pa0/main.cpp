#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>

int main(){

    Eigen::Vector3f point(2.0, 1.0, 1.0);
    Eigen::Matrix3f transformation;
    transformation << cos(M_PI/4), -sin(M_PI/4), 1.0, sin(M_PI/4), cos(M_PI/4), 2.0, 0.0, 0.0, 1.0;
    point = transformation * point;
    std::cout << point << std::endl;
    return 0;
}