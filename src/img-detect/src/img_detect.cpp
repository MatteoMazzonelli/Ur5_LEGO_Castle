#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64MultiArray.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <cmath>
#include <cstring>
#include <vector>
#include <thread>
#include <Python.h>
#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

using namespace Eigen;
class ObjPos
{
public:
    float x;
    float y;
    char nome[30];
    ObjPos(char *n, float cx, float cy)
    {
        strcpy(nome, n);
        x = cx;
        y = cy;
    }
    friend std::ostream &operator<<(std::ostream &os, const ObjPos &o);
};
std::ostream &operator<<(std::ostream &os, const ObjPos &o)
{
    return os << o.nome << " " << o.x << " " << o.y << std::endl;
}

//Global constants
char *posfile = "/tmp/position/positions.txt";
std::vector<ObjPos> oggetti;
std_msgs::Float64MultiArray cord_x;
std_msgs::Float64MultiArray cord_y;
std_msgs::Int64MultiArray type;

//Transform String name into Int name
int assegnaTipo(char *nome)
{

    if (!strcmp(nome, "X1-Y1-Z2"))
        return 1120;
    if (!strcmp(nome, "X1-Y2-Z2-TWINFILLET"))
        return 1222;
    if (!strcmp(nome, "X1-Y2-Z1"))
        return 1210;
    if (!strcmp(nome, "X1-Y2-Z2"))
        return 1220;
    if (!strcmp(nome, "X1-Y2-Z2-CHAMFER"))
        return 1223;
    if (!strcmp(nome, "X1-Y3-Z2-FILLET"))
        return 1321;
    if (!strcmp(nome, "X2-Y2-Z2-FILLET"))
        return 2221;
    if (!strcmp(nome, "X1-Y4-Z1"))
        return 1410;
    if (!strcmp(nome, "X1-Y3-Z2"))
        return 1320;
    if (!strcmp(nome, "X1-Y4-Z2"))
        return 1420;
    if (!strcmp(nome, "X2-Y2-Z2"))
        return 2220;
}

//Return coordinates for the real gazebo world using image coordinates and PrespectiveTransform
Vector2f getDstPoint(cv::Mat pt, Vector3f src_point)
{

    Matrix3f ptransf;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            ptransf(i, j) = pt.at<_Float64>(i, j);
        }
    }
    Vector3f tmp_point = ptransf * src_point;
    Vector2f dst_point(tmp_point(0) / tmp_point(2), tmp_point(1) / tmp_point(2));
    return dst_point;
}

//read positions from position.txt file and save them into Arrays for publishing
int leggiPosizioni(cv::Mat pt)
{
    std::ifstream input_file(posfile);
    if (!input_file.is_open())
    {
        std::cerr << "Impossibile aprire il file - '"
                  << posfile << "'" << std::endl;
        return EXIT_FAILURE;
    }
    float x, y;
    char nome[30];
    int count = 0;
    cord_x.data.clear();
    cord_y.data.clear();
    type.data.clear();
    while (input_file >> nome >> x >> y)
    {
        Vector3f src_point(x, y, 1);
        Vector2f dst_point = getDstPoint(pt, src_point);

        oggetti.push_back(ObjPos(nome, dst_point(0), dst_point(1)));
        cord_x.data.push_back(dst_point(0));
        cord_y.data.push_back(dst_point(1));
        type.data.push_back(assegnaTipo(nome));
    }

    return 0;
}

int main(int argc, char **argv)
{

    //values for PrespectiveMatrix
    cv::Point2f ad(0, -0.5);
    cv::Point2f bd(0, 0.5);
    cv::Point2f cd(0.5, -0.5);
    cv::Point2f dd(0.5, 0.5);

    cv::Point2f as(465, 792);
    cv::Point2f bs(1577, 792);
    cv::Point2f cs(250, 1082);
    cv::Point2f ds(1789, 1082);
    
    cv::Point2f dst_points[4] = {ad, bd, cd, dd};
    cv::Point2f src_points[4] = {as, bs, cs, ds};
    cv::Mat pt = getPerspectiveTransform(src_points, dst_points);

    //Create publishers
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher chatter_pub_x = n.advertise<std_msgs::Float64MultiArray>("cord_x", 1000);
    ros::Publisher chatter_pub_y = n.advertise<std_msgs::Float64MultiArray>("cord_y", 1000);
    ros::Publisher chatter_pub_type = n.advertise<std_msgs::Int64MultiArray>("name", 1000);
    ros::Publisher chatter_pub_angles = n.advertise<std_msgs::Float64MultiArray>("angles", 1000);
    ros::Rate loop_rate(100);

    //Publishing loop
    while (ros::ok())
    {

        if (leggiPosizioni(pt) == 0)
        {
            chatter_pub_x.publish(cord_x);
            chatter_pub_y.publish(cord_y);
            chatter_pub_type.publish(type);
            ros::spinOnce();
        }
        loop_rate.sleep();
    }

    return 0;
}
