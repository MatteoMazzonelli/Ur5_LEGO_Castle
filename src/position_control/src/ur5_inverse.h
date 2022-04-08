#ifndef __INVERSE_H
#define __INVERSE_H

#define _USE_MATH_DEFINES
#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <complex>


//FUNCTIONS FOR INVERSE KINEMATICS

using namespace Eigen;
void completeMat(MatrixXd &mat, double x, int type)
{
    switch (type)
    {
    case 10 /* constant-expression */:
        mat << cos(x), -sin(x), 0, 0,
            sin(x), cos(x), 0, 0,
            0, 0, 1, robot::D[0],
            0, 0, 0, 1;
        break;
    case 21 /* constant-expression */:
        mat << cos(x), -sin(x), 0, 0,
            0, 0, -1, 0,
            sin(x), cos(x), 0, 0,
            0, 0, 0, 1;
        break;
    case 32 /* constant-expression */:
        mat << cos(x), -sin(x), 0, robot::A[1],
            sin(x), cos(x), 0, 0,
            0, 0, 1, robot::D[2],
            0, 0, 0, 1;
        break;
    case 43 /* constant-expression */:
        mat << cos(x), -sin(x), 0, robot::A[2],
            sin(x), cos(x), 0, 0,
            0, 0, 1, robot::D[3],
            0, 0, 0, 1;
        break;
    case 54 /* constant-expression */:
        mat << cos(x), -sin(x), 0, 0,
            0, 0, -1, -robot::D[4],
            sin(x), cos(x), 0, 0,
            0, 0, 0, 1;
        break;
    case 65 /* constant-expression */:
        mat << cos(x), -sin(x), 0, 0,
            0, 0, 1, robot::D[5],
            -sin(x), -cos(x), 0, 0,
            0, 0, 0, 1;
        break;
    default:
        break;
    }
}

double nanControl(double x)
{
    return std::isnan(x) ?  0 : x ; 
}

double createTh4(MatrixXd &T43m, MatrixXd &T32, MatrixXd &T21, MatrixXd &T10, MatrixXd &T60, MatrixXd &T65, MatrixXd &T54)
{
    T43m = T32.inverse() * T21.inverse() * T10.inverse() * T60 * T65.inverse() * T54.inverse();
    Vector3d Xhat43(T43m(0, 0), T43m(1, 0), T43m(2, 0));
    return (atan2(Xhat43(1), Xhat43(0)));
}

MatrixXd ur5Inverse(Ur5 &u, Position ee_pos)
{
    MatrixXd R60(3, 3);
    MatrixXd p60(1, 3);
    R60 = ee_pos.Re;
    p60 = ee_pos.pe;

    //Create T60
    MatrixXd horzcat(R60.rows(), R60.cols() + p60.cols());
    horzcat << R60, p60;
    MatrixXd temp(1, 4);
    temp << 0, 0, 0, 1;

    MatrixXd T60(horzcat.rows() + temp.rows(), horzcat.cols());
    T60 << horzcat, temp;

    MatrixXd T10(4, 4);
    MatrixXd T21(4, 4);
    MatrixXd T32(4, 4);
    MatrixXd T43(4, 4);
    MatrixXd T54(4, 4);
    MatrixXd T65(4, 4);
    MatrixXd T06(4, 4);

    //finding th1
    VectorXd p50(4);
    p50 << 0, 0, -robot::D[5], 1;
    p50 = T60 * p50;
    double th1_1 = nanControl(atan2(p50(1), p50(0)) + acos(robot::D[3] / hypot(p50(1), p50(0)))) + M_PI / 2;
    double th1_2 = nanControl(atan2(p50(1), p50(0)) - acos(robot::D[3] / hypot(p50(1), p50(0)))) + M_PI / 2;

    //finding th5
    double th5_1 = nanControl(acos((p60(0) * sin(th1_1) - p60(1) * cos(th1_1) - robot::D[3]) / robot::D[5]));
    double th5_2 = -nanControl(acos((p60(0) * sin(th1_1) - p60(1) * cos(th1_1) - robot::D[3]) / robot::D[5]));
    double th5_3 = nanControl(acos((p60(0) * sin(th1_2) - p60(1) * cos(th1_2) - robot::D[3]) / robot::D[5]));
    double th5_4 = -nanControl(acos((p60(0) * sin(th1_2) - p60(1) * cos(th1_2) - robot::D[3]) / robot::D[5]));

    //related to th11 a th51
    T06 = T60.inverse();
    Vector3d Xhat(T06(0, 0), T06(1, 0), T06(2, 0));
    Vector3d Yhat(T06(0, 1), T06(1, 1), T06(2, 1));

    double th6_1 = nanControl(atan2(((-Xhat(1) * sin(th1_1) + Yhat(1) * cos(th1_1))) / sin(th5_1), ((Xhat(0) * sin(th1_1) - Yhat(0) * cos(th1_1))) / sin(th5_1)));
    //related to th11 a th52
    double th6_2 = nanControl(atan2(((-Xhat(1) * sin(th1_1) + Yhat(1) * cos(th1_1)) / sin(th5_2)), ((Xhat(0) * sin(th1_1) - Yhat(0) * cos(th1_1)) / sin(th5_2))));
    //related to th12 a th53
    double th6_3 = nanControl(atan2(((-Xhat(1) * sin(th1_2) + Yhat(1) * cos(th1_2)) / sin(th5_3)), ((Xhat(0) * sin(th1_2) - Yhat(0) * cos(th1_2)) / sin(th5_3))));
    //related to th12 a th54
    double th6_4 = nanControl(atan2(((-Xhat(1) * sin(th1_2) + Yhat(1) * cos(th1_2)) / sin(th5_4)), ((Xhat(0) * sin(th1_2) - Yhat(0) * cos(th1_2)) / sin(th5_4))));

    MatrixXd T41m(4, 4);
    completeMat(T10, th1_1, 10);
    completeMat(T65, th6_1, 65);
    completeMat(T54, th5_1, 54);
    T41m = T10.inverse() * T60 * T65.inverse() * T54.inverse();
    Vector3d p41_1(T41m(0, 3), T41m(1, 3), T41m(2, 3));
    double p41xz_1 = hypot(p41_1(0), p41_1(2));

    completeMat(T65, th6_2, 65);
    completeMat(T54, th5_2, 54);
    T41m = T10.inverse() * T60 * T65.inverse() * T54.inverse();
    Vector3d p41_2(T41m(0, 3), T41m(1, 3), T41m(2, 3));
    double p41xz_2 = hypot(p41_2(0), p41_2(2));

    completeMat(T10, th1_2, 10);
    completeMat(T65, th6_3, 65);
    completeMat(T54, th5_3, 54);
    T41m = T10.inverse() * T60 * T65.inverse() * T54.inverse();
    Vector3d p41_3(T41m(0, 3), T41m(1, 3), T41m(2, 3));
    double p41xz_3 = hypot(p41_3(0), p41_3(2));

    completeMat(T65, th6_4, 65);
    completeMat(T54, th5_4, 54);
    T41m = T10.inverse() * T60 * T65.inverse() * T54.inverse();
    Vector3d p41_4(T41m(0, 3), T41m(1, 3), T41m(2, 3));
    double p41xz_4 = hypot(p41_4(0), p41_4(2));

    //Computation of eight possible value for th3
    double th3_1 = nanControl((acos(((p41xz_1 * p41xz_1) - (robot::A[1] * robot::A[1]) - (robot::A[2] * robot::A[2])) / (2 * robot::A[1] * robot::A[2]))));
    double th3_2 = nanControl((acos(((p41xz_2 * p41xz_2) - (robot::A[1] * robot::A[1]) - (robot::A[2] * robot::A[2])) / (2 * robot::A[1] * robot::A[2]))));
    double th3_3 = nanControl((acos(((p41xz_3 * p41xz_3) - (robot::A[1] * robot::A[1]) - (robot::A[2] * robot::A[2])) / (2 * robot::A[1] * robot::A[2]))));
    double th3_4 = nanControl((acos(((p41xz_4 * p41xz_4) - (robot::A[1] * robot::A[1]) - (robot::A[2] * robot::A[2])) / (2 * robot::A[1] * robot::A[2]))));
    double th3_5 = -th3_1;
    double th3_6 = -th3_2;
    double th3_7 = -th3_3;
    double th3_8 = -th3_4;

    //Computation of eight possible value for th2
    double th2_1 = nanControl(atan2(-p41_1(2), -p41_1(0)) - asin((-robot::A[2] * sin(th3_1)) / p41xz_1));
    double th2_2 = nanControl(atan2(-p41_2(2), -p41_2(0)) - asin((-robot::A[2] * sin(th3_2)) / p41xz_2));
    double th2_3 = nanControl(atan2(-p41_3(2), -p41_3(0)) - asin((-robot::A[2] * sin(th3_3)) / p41xz_3));
    double th2_4 = nanControl(atan2(-p41_4(2), -p41_4(0)) - asin((-robot::A[2] * sin(th3_4)) / p41xz_4));
    double th2_5 = nanControl(atan2(-p41_1(2), -p41_1(0)) - asin((robot::A[2] * sin(th3_1)) / p41xz_1));
    double th2_6 = nanControl(atan2(-p41_2(2), -p41_2(0)) - asin((robot::A[2] * sin(th3_2)) / p41xz_2));
    double th2_7 = nanControl(atan2(-p41_3(2), -p41_3(0)) - asin((robot::A[2] * sin(th3_3)) / p41xz_3));
    double th2_8 = nanControl(atan2(-p41_4(2), -p41_4(0)) - asin((robot::A[2] * sin(th3_4)) / p41xz_4));


    //Computation of eight possible value for th4
    MatrixXd T43m(4, 4);
    completeMat(T32, th3_1, 32);
    completeMat(T21, th2_1, 21);
    completeMat(T10, th1_1, 10);
    completeMat(T65, th6_1, 65);
    completeMat(T54, th5_1, 54);
    double th4_1 = createTh4(T43m, T32, T21, T10, T60, T65, T54);

    completeMat(T32, th3_2, 32);
    completeMat(T21, th2_2, 21);
    completeMat(T65, th6_2, 65);
    completeMat(T54, th5_2, 54);
    double th4_2 = createTh4(T43m, T32, T21, T10, T60, T65, T54);

    completeMat(T32, th3_3, 32);
    completeMat(T21, th2_3, 21);
    completeMat(T65, th6_3, 65);
    completeMat(T54, th5_3, 54);
    completeMat(T10, th1_2, 10);
    double th4_3 = createTh4(T43m, T32, T21, T10, T60, T65, T54);

    completeMat(T32, th3_4, 32);
    completeMat(T21, th2_4, 21);
    completeMat(T65, th6_4, 65);
    completeMat(T54, th5_4, 54);
    double th4_4 = createTh4(T43m, T32, T21, T10, T60, T65, T54);

    completeMat(T32, th3_5, 32);
    completeMat(T21, th2_5, 21);
    completeMat(T65, th6_1, 65);
    completeMat(T54, th5_1, 54);
    completeMat(T10, th1_1, 10);
    double th4_5 = createTh4(T43m, T32, T21, T10, T60, T65, T54);

    completeMat(T32, th3_6, 32);
    completeMat(T21, th2_6, 21);
    completeMat(T65, th6_2, 65);
    completeMat(T54, th5_2, 54);
    double th4_6 = createTh4(T43m, T32, T21, T10, T60, T65, T54);

    completeMat(T32, th3_7, 32);
    completeMat(T21, th2_7, 21);
    completeMat(T65, th6_3, 65);
    completeMat(T54, th5_3, 54);
    completeMat(T10, th1_2, 10);
    double th4_7 = createTh4(T43m, T32, T21, T10, T60, T65, T54);

    completeMat(T32, th3_8, 32);
    completeMat(T21, th2_8, 21);
    completeMat(T65, th6_4, 65);
    completeMat(T54, th5_4, 54);
    double th4_8 = createTh4(T43m, T32, T21, T10, T60, T65, T54);

    MatrixXd Th(8, 6);
    Th << th1_1, th2_1, th3_1, th4_1, th5_1, th6_1,
        th1_1, th2_2, th3_2, th4_2, th5_2, th6_2,
        th1_2, th2_3, th3_3, th4_3, th5_3, th6_3,
        th1_2, th2_4, th3_4, th4_4, th5_4, th6_4,
        th1_1, th2_5, th3_5, th4_5, th5_1, th6_1,
        th1_1, th2_6, th3_6, th4_6, th5_2, th6_2,
        th1_2, th2_7, th3_7, th4_7, th5_3, th6_3,
        th1_2, th2_8, th3_8, th4_8, th5_4, th6_4;

    return Th;
}
 
#endif
