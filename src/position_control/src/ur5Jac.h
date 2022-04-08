#ifndef __JACOBIAN_H__
#define __JACOBIAN_H__

#define _USE_MATH_DEFINES
#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include "ur5_direct.h"
#include "rot2eul.h"

using namespace Eigen;

MatrixXd Jac(VectorXd th){
    double A1,A2,A3,A4,A5,A6;
    double D1,D2,D3,D4,D5,D6;
    double th1,th2,th3,th4,th5,th6;
    A1=robot::A[0];
    A2=robot::A[1];
    A3=robot::A[2];
    A4=robot::A[3];
    A5=robot::A[4];
    A6=robot::A[5];
    D1=robot::D[0];
    D2=robot::D[1];
    D3=robot::D[2];
    D4=robot::D[3];
    D5=robot::D[4];
    D6=robot::D[5];
    th1=th[0];
    th2=th[1];
    th3=th[2];
    th4=th[3];
    th5=th[4];
    th6=th[5];
    VectorXd J1(6);
    VectorXd J2(6);
    VectorXd J3(6);
    VectorXd J4(6);
    VectorXd J5(6);
    VectorXd J6(6);
    J1<<D5*(cos(th1)*cos(th5) + cos(th2 + th3 + th4)*sin(th1)*sin(th5)) + D3*cos(th1) + D4*cos(th1) - A3*cos(th2 + th3)*sin(th1) - A2*cos(th2)*sin(th1) - D5*sin(th2 + th3 + th4)*sin(th1),
        D5*(cos(th5)*sin(th1) - cos(th2 + th3 + th4)*cos(th1)*sin(th5)) + D3*sin(th1) + D4*sin(th1) + A3*cos(th2 + th3)*cos(th1) + A2*cos(th1)*cos(th2) + D5*sin(th2 + th3 + th4)*cos(th1),
        0,
        0,
        0,
        1;
    J2<<-cos(th1)*(A3*sin(th2 + th3) + A2*sin(th2) + D5*(sin(th2 + th3)*sin(th4) - cos(th2 + th3)*cos(th4)) - D5*sin(th5)*(cos(th2 + th3)*sin(th4) + sin(th2 + th3)*cos(th4))),
        -sin(th1)*(A3*sin(th2 + th3) + A2*sin(th2) + D5*(sin(th2 + th3)*sin(th4) - cos(th2 + th3)*cos(th4)) - D5*sin(th5)*(cos(th2 + th3)*sin(th4) + sin(th2 + th3)*cos(th4))),
        A3*cos(th2 + th3) - (D5*sin(th2 + th3 + th4 + th5))/2 + A2*cos(th2) + (D5*sin(th2 + th3 + th4 - th5))/2 + D5*sin(th2 + th3 + th4),
        sin(th1),
        -cos(th1),
        0;
    J3<<cos(th1)*(D5*cos(th2 + th3 + th4) - A3*sin(th2 + th3) + D5*sin(th2 + th3 + th4)*sin(th5)),
        sin(th1)*(D5*cos(th2 + th3 + th4) - A3*sin(th2 + th3) + D5*sin(th2 + th3 + th4)*sin(th5)),
        A3*cos(th2 + th3) - (D5*sin(th2 + th3 + th4 + th5))/2 + (D5*sin(th2 + th3 + th4 - th5))/2 + D5*sin(th2 + th3 + th4),
        sin(th1),
        -cos(th1),
        0;
    J4<<D5*cos(th1)*(cos(th2 + th3 + th4) + sin(th2 + th3 + th4)*sin(th5)),
        D5*sin(th1)*(cos(th2 + th3 + th4) + sin(th2 + th3 + th4)*sin(th5)),
        D5*(sin(th2 + th3 + th4 - th5)/2 + sin(th2 + th3 + th4) - sin(th2 + th3 + th4 + th5)/2),
        sin(th1),
        -cos(th1),
        0;
    J5<<-D5*sin(th1)*sin(th5) - D5*cos(th2 + th3 + th4)*cos(th1)*cos(th5),
        D5*cos(th1)*sin(th5) - D5*cos(th2 + th3 + th4)*cos(th5)*sin(th1),
        -D5*(sin(th2 + th3 + th4 - th5)/2 + sin(th2 + th3 + th4 + th5)/2),
        sin(th2 + th3 + th4)*cos(th1),
        sin(th2 + th3 + th4)*sin(th1),
        -cos(th2 + th3 + th4);
    J6<<0,
        0,
        0,
        cos(th5)*sin(th1) - cos(th2 + th3 + th4)*cos(th1)*sin(th5),
        -cos(th1)*cos(th5) - cos(th2 + th3 + th4)*sin(th1)*sin(th5),
        -sin(th2 + th3 + th4)*sin(th5);
    MatrixXd J(6,6);
    J<<J1,J2,J3,J4,J5,J6;
    return J;
}

Vector3d f_pd(Vector3d &xef, Vector3d &xe0, double t){
    if(t>1){
        return xef;
    }else{
        return t*xef + (1-t)*xe0;
    }
}
Vector3d f_phid(Vector3d &phief, Vector3d &phie0, double t){
    if(t>1){
        return phief;
    }else{
        return t*phief + (1-t)*phie0;
    }
}
//                                              6x1           3x1          3x1          3x1         3x1            3x1             3x1              3x3            3x3
MatrixXd invDiffKinematicControlComplete(VectorXd q, VectorXd xe, VectorXd xd, VectorXd vd,VectorXd phie, VectorXd phid, VectorXd phiddot, MatrixXd Kp, MatrixXd Kphi){
    MatrixXd J = Jac(q);
    double alpha = phie(2);
    double beta = phie(1);
    double gamma = phie(0);
    MatrixXd T(3,3);
    T<<cos(beta)*cos(gamma), -sin(gamma), 0,
        cos(beta)*sin(gamma), cos(gamma), 0,
        -sin(beta), 0, 1;
    MatrixXd Ta(6,6);
    Ta.topLeftCorner(3,3) = MatrixXd::Identity(3,3);
    Ta.topRightCorner(3,3) = MatrixXd::Zero(3,3);
    Ta.bottomLeftCorner(3,3) = MatrixXd::Zero(3,3);
    Ta.bottomRightCorner(3,3) = T;
    MatrixXd Ja = Ta.inverse()*J;
    MatrixXd prod(6,1);
    prod<<vd+Kp*(xd-xe), //3x1
          phiddot+Kphi*(phid-phie); //3x1
    MatrixXd dotQ = Ja.inverse()*prod;
    return dotQ; 
}

MatrixXd invDiffKinematicControlSimComplete(VectorXd &TH0, MatrixXd &Kp, MatrixXd &Kphi, double minT, double maxT, double Dt){
    int dim = (maxT-minT)/Dt;
    double vet[dim];
    vet[0]=Dt;
    for(int i=1;i<dim;i++)
        vet[i] = vet[i-1] + Dt;
    VectorXd qk = TH0;
    VectorXd q = qk;
    double t;
    Vector3d phie;
    Vector3d phiddot;
    Vector3d vd;
    VectorXd dotqk(6);
    VectorXd qk1(6);
    MatrixXd qm(dim+1,6);
    qm.row(0)=q;
    Ur5 u;
    for(int i=0;i<dim;i++){
        t=vet[i];
        ur5Direct(u,qk);
        phie = rot2eul(u.p.Re);
        vd = (1/Dt)*f_pd(robot::xef,robot::xe0,t)-(1/Dt)*f_pd(robot::xef,robot::xe0,t-Dt);
        phiddot = (1/Dt)*(f_phid(robot::phief,robot::phie0,t))-f_phid(robot::phief,robot::phie0,t-Dt)*(1/Dt);
        dotqk = invDiffKinematicControlComplete(qk,u.p.pe,f_pd(robot::xef,robot::xe0,t),vd,phie,f_phid(robot::phief,robot::phie0,t),phiddot,Kp,Kphi);
        qk1 = qk + dotqk*Dt;
        qm.row(i+1)=qk1;
        qk = qk1;
    }
    return qm;
}
#endif
