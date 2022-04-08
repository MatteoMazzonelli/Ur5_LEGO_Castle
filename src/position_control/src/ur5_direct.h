#ifndef __DIRECT_H__
#define __DIRECT_H__

#define _USE_MATH_DEFINES
#include <iostream>
#include <Eigen/Dense>
#include <cmath>


//FUNCTIONS FOR DIRECT KINEMATICS


using namespace Eigen;
class Position{ 
public:
    MatrixXd pe; // x,y,z end effector
    MatrixXd Re; //rotation matrix of end effector 
    Position(){
        pe.resize(3,1);
        Re.resize(3,3);
    }
    friend std::ostream& operator <<(std::ostream& os,const Position& p);
};
std::ostream& operator <<(std::ostream& os,const Position& p){
    return os<<p.pe<<std::endl<<std::endl<<p.Re;
}
namespace robot{
    // double alpha[6] = {0,M_PI_2,0,0,M_PI_2,-M_PI_2};
    // double D[6] = {0.1625, 0, 0, 0.1333, 0.0997, 0.0996};
    // double A[6] = {0, -0.425, -0.3922, 0, 0, 0};
     double alpha[6] = {1.570796327, 0, 0, 1.570796327, -1.570796327, 0};
     double D[6] = {0.089159,  0.00000,  0.00000,  0.10915,  0.09465,  0.0823};
     double A[6] = {0.00000, -0.42500, -0.39225,  0.00000,  0.00000,  0.0000}; 
    // a = [0.00000, -0.42500, -0.39225,  0.00000,  0.00000,  0.0000]
    // d = [0.089159,  0.00000,  0.00000,  0.10915,  0.09465,  0.0823]
    // alpha = [ 1.570796327, 0, 0, 1.570796327, -1.570796327, 0 ]
    Vector3d xe0(0.3, 0.3, 0.1);
    Vector3d phie0(0,0,0);
    Vector3d xef(0.5, 0.5, 0.5);
    Vector3d phief(M_PI_4,M_PI_4,M_PI_4);
    double default_pos_high[6] = {M_PI_2,-M_PI_2,M_PI_2,-M_PI_2,-M_PI_2,M_PI_2};
    double default_pos_ground[6] = {M_PI_2,-M_PI_2,M_PI_2,0,M_PI_2,0};

}

class Ur5{
public:
    Position p; //position of end effector
    Ur5():p(){} //call position constructor
    void set_pos(MatrixXd _pe, MatrixXd _re){
        p.pe=_pe;
        p.Re=_re;
    }
    friend std::ostream& operator <<(std::ostream& os,const Ur5& a);
};
std::ostream& operator <<(std::ostream& os,const Ur5& u){
    return os<<u.p;
}
void ur5Direct(Ur5 &u, VectorXd th){ //reference to Ur5 robot, array of angles
    MatrixXd T10(4,4);
    MatrixXd T21(4,4);
    MatrixXd T32(4,4);
    MatrixXd T43(4,4);
    MatrixXd T54(4,4);
    MatrixXd T65(4,4);
    MatrixXd T06(4,4);
    T10<<cos(th[0]), -sin(th[0]), 0, 0,
        sin(th[0]), cos(th[0]), 0, 0,
        0, 0, 1, robot::D[0],
        0, 0, 0, 1;
    T21<<cos(th[1]), -sin(th[1]), 0, 0,
        0, 0, -1, 0,
        sin(th[1]), cos(th[1]), 0, 0,
        0, 0, 0, 1;
    T32<<cos(th[2]), -sin(th[2]), 0, robot::A[1],
        sin(th[2]), cos(th[2]), 0, 0,
        0, 0, 1, robot::D[2],
        0, 0, 0, 1;
    T43<<cos(th[3]), -sin(th[3]), 0, robot::A[2],
        sin(th[3]), cos(th[3]), 0, 0,
        0, 0, 1, robot::D[3],
        0, 0, 0, 1;
    T54<<cos(th[4]), -sin(th[4]), 0, 0,
        0, 0, -1, -robot::D[4],
        sin(th[4]), cos(th[4]), 0, 0,
        0, 0, 0, 1;
    T65<<cos(th[5]), -sin(th[5]), 0, 0,
        0, 0, 1, robot::D[5],
        -sin(th[5]), -cos(th[5]), 0, 0,
        0, 0, 0, 1;
    T06=T10*T21*T32*T43*T54*T65;
    u.set_pos(T06(seq(0,2),3),T06(seq(0,2),seq(0,2)));
} 

#endif
