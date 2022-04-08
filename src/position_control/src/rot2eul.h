#ifndef __ROT2EUL_H__
#define __ROT2EUL_H__

#define _USE_MATH_DEFINES
#include <iostream>
#include <Eigen/Dense>
#include <cmath>

using namespace Eigen;

VectorXd rot2eul(MatrixXd R){
    double sy = sqrt(R(0,0) * R(0,0) +  R(1,0) * R(1,0) );
    bool singular = sy < 1e-6; // If
    double x, y, z;
    if (!singular){ 
        x = atan2(R(2,1) , R(2,2));
        y = atan2(-R(2,0), sy);
        z = atan2(R(1,0), R(0,0));
    }
    else{
        x = atan2(-R(1,2), R(1,1));
        y = atan2(-R(2,0), sy);
        z = 0;
    }
    VectorXd v(3);
    v<<z,y,x;
    return v;
}

#endif
