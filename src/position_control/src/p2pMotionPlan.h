#ifndef __P2PMOTION_H
#define __P2PMOTION_H

#define _USE_MATH_DEFINES
#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <complex>
#include "ur5_inverse.h"
#include "ur5_direct.h"
#include "eul2rot.h"

using namespace Eigen;
MatrixXd boundary_control(MatrixXd m){
    if(m(0,0) > M_PI) //boundary control of robot base, to handle the y-negative movements
        m(0,0) = m(0,0) - 2*M_PI;
    if(m(0,0) < -M_PI)
        m(0,0) = m(0,0) + 2*M_PI;
    if(m(0,5) > M_PI) //boundary control of gripper rotation
        m(0,5) = m(0,5) - M_PI;
    if(m(0,5) < -M_PI)
        m(0,5) = m(0,5) + M_PI;
    return m;
}
MatrixXd calibration_high_to_ground(MatrixXd qEs){ //change from vertical position to horizontal
    qEs(0,5) = M_PI;
    qEs(0,4) = M_PI_2;
    qEs(0,3) += M_PI_2;
    return qEs;
}
MatrixXd calibration_ground_to_high(MatrixXd qEs){ //change from horizontal to vertical position
    qEs(0,5) = qEs(0,0);
    qEs(0,4) = -M_PI_2;
    qEs(0,3) -= M_PI_2;
    return qEs;
}
MatrixXd calibration_ground(VectorXd xEf, VectorXd phiEf, MatrixXd qEs, double yaw, bool def){ //calibration for horizontal position
    if(def){ //default position
        MatrixXd qDef (1,6);
        qDef<<qEs(0,0),robot::default_pos_ground[1],robot::default_pos_ground[2],robot::default_pos_ground[3],robot::default_pos_ground[4],robot::default_pos_ground[5];
        return qDef;
    }
    Ur5 u;
    Position p;
    p.pe = xEf;
    p.Re = eul2rot(phiEf);
    MatrixXd qEfi = ur5Inverse(u,p); //compute the inverse
    qEfi(0,4) = M_PI_2;
    qEfi(0,5) = M_PI;
    double Delbow = qEfi(0,2)-qEs(0,2); //Delta elbow
    double Dshoulder = qEfi(0,1)-qEs(0,1); //Delta shoulder lift
    double sum = Delbow + Dshoulder;
    qEfi(0,3) = qEs(0,3) - sum;
    qEfi = boundary_control(qEfi);
    qEfi(0,4) = qEfi(0,0) + M_PI_2;
    
    


    MatrixXd qEf = qEfi.row(0);
    Ur5 u_end;
    VectorXd th(6);
    th<<qEf(0,0),qEf(0,1),qEf(0,2),qEf(0,3),qEf(0,4),qEf(0,5);
    ur5Direct(u_end,th);
    u_end.p.pe=p.pe-(u_end.p.pe-p.pe); //final position with right rotation of wrist 
    MatrixXd qEfi_final = ur5Inverse(u_end,u_end.p);
    qEfi_final(0,4) = M_PI_2;
    qEfi_final(0,5) = M_PI;
    Delbow = qEfi_final(0,2)-qEs(0,2);
    Dshoulder = qEfi_final(0,1)-qEs(0,1);
    sum = Delbow + Dshoulder;
    qEfi_final(0,3) = qEs(0,3) - sum;
    qEfi_final = boundary_control(qEfi_final);
    qEfi_final(0,4) = qEfi_final(0,0) + M_PI_2;
    return qEfi_final.row(0);
}
MatrixXd calibration(VectorXd xEf, VectorXd phiEf, MatrixXd qEs, double yaw, bool def){
    if(def){ //default position
        MatrixXd qDef (1,6);
        qDef<<qEs(0,0),robot::default_pos_high[1],robot::default_pos_high[2],robot::default_pos_high[3],robot::default_pos_high[4],robot::default_pos_high[5] - (M_PI_2 - qEs(0,0)) ;
        return boundary_control(qDef);
    }
    Ur5 u;
    Position p;
    p.pe = xEf;
    p.Re = eul2rot(phiEf);
    MatrixXd qEfi = ur5Inverse(u,p); //Inverse without calibration
    qEfi = boundary_control(qEfi);
    qEfi(0,4) = -M_PI_2; //set wrist2
    double Delbow = qEfi(0,2)-qEs(0,2); //Delta elbow
    double Dshoulder = qEfi(0,1)-qEs(0,1); //Delta shoulder lift
    double sum = Delbow + Dshoulder;
    qEfi(0,3) = qEs(0,3)-sum; // Calibrate wrist1
    qEfi(0,5) = qEfi(0,0) - yaw + M_PI_2; //Calibrate wrist3 (rotation of gripper)
    qEfi = boundary_control(qEfi);
    

    MatrixXd qEf = qEfi.row(0);
    Ur5 u_end;
    VectorXd th(6);
    th<<qEf(0,0),qEf(0,1),qEf(0,2),qEf(0,3),qEf(0,4),qEf(0,5);
    ur5Direct(u_end,th);
    u_end.p.pe=p.pe-(u_end.p.pe-p.pe); //final position with right rotation of wrist 
    MatrixXd qEfi_final = ur5Inverse(u_end,u_end.p);
    qEfi_final = boundary_control(qEfi_final);
    qEfi_final(0,4) = -1.57;
    Delbow = qEfi_final(0,2)-qEs(0,2);
    Dshoulder = qEfi_final(0,1)-qEs(0,1);
    sum = Delbow + Dshoulder;
    qEfi_final(0,3) = qEs(0,3)-sum;
    qEfi_final(0,5) = qEfi(0,0) - yaw + M_PI_2;
    qEfi_final = boundary_control(qEfi_final);
    return qEfi_final.row(0);
}
MatrixXd p2pMotion(VectorXd jnt_pos_start, VectorXd xEf, VectorXd phiEf, double yaw,double minT, double maxT, double dt, bool def, int setting){
    Ur5 u;
    MatrixXd qEs (1,6);
    qEs<<jnt_pos_start[0],jnt_pos_start[1],jnt_pos_start[2],jnt_pos_start[3],jnt_pos_start[4],jnt_pos_start[5];
    MatrixXd qEf;
    switch(setting){
        case 0:
            qEf = calibration_ground(xEf,phiEf,qEs,yaw,def);
            break;
        case 1:
            qEf = calibration(xEf,phiEf,qEs,yaw,def);
            break;
        case 2:
            qEf = calibration_ground_to_high(qEs);
            break;
        case 3:
            qEf = calibration_high_to_ground(qEs);
            break;
    }
    //std::cout<<qEf.row(0)<<std::endl<<std::endl;
    MatrixXd M(4,4);
    M<<1,minT,minT*minT,minT*minT*minT,
       0,1,2*minT,3*minT*minT,
       1,maxT,maxT*maxT,maxT*maxT*maxT,
       0,1,2*maxT,3*maxT*maxT;
    MatrixXd b(4,1);
    MatrixXd a(4,1);
    MatrixXd A(6,4);
    for(int i=0;i<6;i++){
        b<<qEs(i),0,qEf(i),0;
        
        a = M.inverse()*b;
        A.row(i)=a.transpose();
    }
    int dim=ceil((maxT-minT)/dt);
    double vet[dim+1];
    vet[0]=0;
    for(int i=1;i<dim+1;i++)
        vet[i] = vet[i-1] + dt;
    MatrixXd Th(dim+1,7);
    MatrixXd xE(dim+1,4);
    MatrixXd phiE(dim+1,4);
    VectorXd th(7);
    VectorXd xEvet(4);
    VectorXd phiEvet(4);
    double q;
    double t=0;
    for(int i=0;i<dim+1;i++){
        th(0)=t;
        for(int j=0;j<6;j++){
            q = A(j,0)+A(j,1)*t+A(j,2)*t*t+A(j,3)*t*t*t;
            th(j+1)=q;
        }
        Th.row(i)=th;
        ur5Direct(u,th.segment(1,6));
        xEvet(0)=t;
        xEvet.segment(1,3)=u.p.pe;
        xE.row(i) = xEvet;
        phiEvet(0)=t;
        phiEvet.segment(1,3)=rot2eul(u.p.Re);
        phiE.row(i)=phiEvet;
        t+=dt;
    }
    return Th;
}

#endif
