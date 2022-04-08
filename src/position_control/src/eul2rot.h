#ifndef __EUL2ROT_H__
#define __EUL2ROT_H__

#define _USE_MATH_DEFINES
#include <iostream>
#include <Eigen/Dense>
#include <cmath>

using namespace Eigen;

MatrixXd xrot(double psi){
    MatrixXd mat(3,3); 
    mat<<1, 0, 0,
        0, cos(psi), -sin(psi),
        0, sin(psi), cos(psi);
    return mat;
}
MatrixXd yrot(double theta){
    MatrixXd mat(3,3);
    mat<<cos(theta), 0 , sin(theta), 
        0, 1, 0,
        -sin(theta), 0, cos(theta);
    return mat;
}
MatrixXd zrot(double phi){
    MatrixXd mat(3,3);
    mat<<cos(phi), - sin(phi), 0, 
        sin(phi), cos(phi), 0, 
        0, 0, 1;
    return mat;
}
MatrixXd eul2rot(Vector3d angle){
    return zrot(angle(0))*yrot(angle(1))*xrot(angle(2));
}
#endif
