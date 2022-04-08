#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int64MultiArray.h"
#include "std_msgs/Int16.h"
#include "control_msgs/JointControllerState.h"
#include "control_msgs/GripperCommandActionGoal.h"
#include "ur5_direct.h"
#include "ur5_inverse.h"
#include "eul2rot.h"
#include "rot2eul.h"
#include "p2pMotionPlan.h"
#include "ur5Jac.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>
#include <map>
#include <cmath>
#include <cstring>
#include <fstream>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <ignition/math/Pose3.hh>
#include "gazebo_ros_link_attacher/gazebo_ros_link_attacher.h"
#include "gazebo_ros_link_attacher/Attach.h"
#include "gazebo_ros_link_attacher/AttachRequest.h"
#include "gazebo_ros_link_attacher/AttachResponse.h"
#include <unistd.h>

#define DEF_TIME 10
#define DEF_STEP 0.1
#define H_UP 0.4
#define H_DOWN 0.18
#define DELTA_GROUND 0.155
#define MAX_EFFORT 0.00001
#define OPEN_GRIP 0.1
#define CLOSE_GRIP 0.38

using namespace Eigen;
/// ROS subscribers:
ros::Subscriber joint_com_sub[6];
ros::Publisher joint_com_pub[6];
ros::Publisher gripper_com_pub;
ros::Subscriber gripper_com_sub;
ros::Subscriber cord_x_sub;
ros::Subscriber cord_y_sub;
ros::Subscriber cord_z_sub;
ros::Subscriber name_sub;
ros::Subscriber angle_sub;
ros::Subscriber l1_sub;
ros::Subscriber l2_sub;
ros::Subscriber orientation_sub;
ros::Subscriber blocks_x_sub;
ros::Subscriber blocks_y_sub;
ros::Subscriber spawner_n_sub;

VectorXd jnt_pos_start(6);
///data from rotation-ctrl
std::vector<double> cord_x;
std::vector<double> cord_y;
std::vector<double> cord_z;
std::vector<double> angle;
std::vector<double> l1;
std::vector<double> l2;
std::vector<long int> orientation;
std::vector<long int> name;
///data from spawn_blocks
std::vector<double> blocks_x;
std::vector<double> blocks_y;
int spawner_n;

control_msgs::GripperCommandActionGoal gripper_pos;

gazebo_ros_link_attacher::AttachRequest req;
gazebo_ros_link_attacher::AttachResponse res;

///final position for assignment 1, 2, 3
std::map<int,int> final_pos_index = {{1210,0},{1220,1},{1223,2},{1222,3},{2221,4},{2220,5},{1420,6},{1321,7},{1320,8},{1410,9},{1120,10}};
const std::vector<double> final_pos_x = {-0.05, -0.17, -0.28, -0.38, -0.45, -0.5, -0.45, -0.37, -0.26, -0.13, -0.03};
const std::vector<double> final_pos_y = {0.5, 0.41, 0.33, 0.24, 0.13, 0, -0.12, -0.23, -0.32, -0.41, -0.5};
const std::vector<double> final_pos_yaw = {M_PI_2, M_PI_2, M_PI_2, M_PI_2, 0, 0, M_PI_2, M_PI_2, M_PI_2, M_PI_2, 0};
std::vector<int> final_pos_occupation = {0,0,0,0,0,0,0,0,0,0,0};

///rotate positions
const std::vector<double> rotate_x = {0.5, 0.5, 0.5};
const std::vector<double> rotate_y = {-0.15,0,0.15};
const std::vector<double> delta_rotate = {0.041,0.018,0};
std::vector<bool> rotate_occupation = {false,false,false};
std::vector<double> rotate_pos_return_x = {0,0,0};
std::vector<double> rotate_pos_return_y = {0,0,0};

///blocks dimension
const std::vector<double> block_lenght = {0.031, 0.031, 0.031, 0.031, 0.063, 0.063, 0.031, 0.031, 0.031, 0.031, 0.031};
const std::vector<double> block_width = {0.063, 0.063, 0.063, 0.063, 0.063, 0.063, 0.127, 0.095, 0.095, 0.127, 0.031};
const std::vector<double> block_height = {0.037, 0.055, 0.055, 0.055, 0.055, 0.055, 0.055, 0.055, 0.055, 0.037, 0.055};

///castles for assignment 4
const std::vector<int> castle_1_name = {2220,2220,1420,1120,1120};
const std::vector<double> castle_1_x= {-0.2,-0.2,-0.215,-0.215,-0.215};
const std::vector<double> castle_1_y= {-0.395,-0.325,-0.36,-0.407,-0.312};
const std::vector<double> castle_1_z= {0,0,0.055,0.1,0.1};
const std::vector<int> castle_2_name = {1320,1210,1210,1320,1120};
const std::vector<double> castle_2_x= {-0.2,-0.2,-0.2,-0.2,-0.2};
const std::vector<double> castle_2_y= {0.35,0.433,0.433,0.385,0.385};
const std::vector<double> castle_2_z= {0,0,0.04,0.07,0.11};

///keep count of block detected (Assignment 4)
std::vector<int> right_detect_name = {};
std::vector<double> right_detect_x = {};
std::vector<double> right_detect_y = {};

bool first_time = true;
/// callback functions 
void get_cord_x(const std_msgs::Float64MultiArray::ConstPtr &msg);
void get_cord_y(const std_msgs::Float64MultiArray::ConstPtr &msg);
void get_cord_z(const std_msgs::Float64MultiArray::ConstPtr &msg);
void get_name(const std_msgs::Int64MultiArray::ConstPtr &msg);
void get_angle(const std_msgs::Float64MultiArray::ConstPtr &msg);
void get_blocks_x(const std_msgs::Float64MultiArray::ConstPtr &msg);
void get_blocks_y(const std_msgs::Float64MultiArray::ConstPtr &msg);
void get_spawner_n(const std_msgs::Int16::ConstPtr &msg);
void get_l1(const std_msgs::Float64MultiArray::ConstPtr &msg);
void get_l2(const std_msgs::Float64MultiArray::ConstPtr &msg);
void get_orientation(const std_msgs::Int64MultiArray::ConstPtr &msg);
void get_shoulder_pan_joint_position(const control_msgs::JointControllerState::ConstPtr &ctr_msg);
void get_shoulder_lift_joint_position(const control_msgs::JointControllerState::ConstPtr &ctr_msg);
void get_elbow_joint_position(const control_msgs::JointControllerState::ConstPtr &ctr_msg);
void get_wrist_1_joint_position(const control_msgs::JointControllerState::ConstPtr &ctr_msg);
void get_wrist_2_joint_position(const control_msgs::JointControllerState::ConstPtr &ctr_msg);
void get_wrist_3_joint_position(const control_msgs::JointControllerState::ConstPtr &ctr_msg);
void get_gripper_position(const control_msgs::GripperCommandActionGoal::ConstPtr &ctr_msg);

/// move robot to a certain position
void go_to_position(float xp, float yp, float zp, double yaw, ros::NodeHandle &n, ros::Rate &loop_rate, bool action, bool def, int setting);
void default_pos(ros::NodeHandle &n, ros::Rate &loop_rate, int setting);
void use_gripper(float g_effort, float g_pos, ros::NodeHandle &n, ros::Rate &loop_rate);
std::string assign_name(int n); ///associate numbers with block names
void get_detected_blocks(ros::NodeHandle &n, ros::Rate &loop_rate); //get blocks positions from OpenCV and YOLO
void get_detected_blocks_file();
void get_blocks_pos(ros::NodeHandle &n, ros::Rate &loop_rate); //get blocks positions from spawn for dynamic joint
void attach(std::string block); //Dynamic joint - attach
void detach(std::string block); //Dynamic joint - detach
double distance(double x1, double y1, double x2, double y2); //Euclidean distance
int nearest(double x, double y); //nearest block to attach or detach
int free_position(); //returns an index if there's a rotate position free, otherwise returns -1
int nearest_cv(double x, double y); //nearest block to pick up, using OpenCV coordinates
bool already_taken(double x, double y); //Assignment 4 - check if blocks has already been detected

///Sequence of motions of the robot
void to_final_position(int id, double x, double y, double l, double w, double h, double yaw, int freepos, ros::NodeHandle &n, ros::Rate &loop_rate); //go to final position (Silhouette)
void to_final_position_ass4(double x, double y, double xf, double yf, double zf, ros::NodeHandle &n, ros::Rate &loop_rate); //go to final position to build the castle
void to_initial_position(int id, double x, double y, double l, double w, double h, double yaw, int freepos, ros::NodeHandle &n, ros::Rate &loop_rate); //After a block is recognized, returns to its initial position
void rotate_PI_4(double x, double y, double l, double w, double h, double yaw, int freepos, ros::NodeHandle &n, ros::Rate &loop_rate); //rotation of pi/4 in order to increase the accuracy of yolo detection
void orientation0(double x, double y, double l, double w, double h, double yaw, int freepos, ros::NodeHandle &n, ros::Rate &loop_rate); //if block is on it's natural position
void orientation1(double x, double y, double l, double w, double h, double yaw, int freepos, ros::NodeHandle &n, ros::Rate &loop_rate); //if block is upside down
void orientation2(double x, double y, double l, double w, double h, double yaw, int freepos, ros::NodeHandle &n, ros::Rate &loop_rate); //if block is on the side
void orientation2_double(double x, double y, double l, double w, double h, double yaw, int freepos, ros::NodeHandle &n, ros::Rate &loop_rate); //already executed orientation2 and still on the side. After this function a block is in its natural position or upside down
int main(int argc, char **argv)
{
	ros::init(argc, argv, "position_talker"); //initialize node
	ros::NodeHandle n;
	ros::Rate loop_rate(DEF_TIME/DEF_STEP);
	get_blocks_pos(n,loop_rate); //get position of spawned blocks
	default_pos(n, loop_rate,1); //move robot in default position
	double x,y,l,w,h,a;
	int index;
	int orient;
	if(spawner_n != 0){ //ASSIGNMENT 4
		while(1){
			sleep(2);
			get_detected_blocks(n,loop_rate);
			get_detected_blocks(n,loop_rate);
			if(cord_x.size()==right_detect_name.size()){ //all blocks have been detected
				std::cout<<"DETECTED BLOCKS - NAME :";
				for(auto i : right_detect_name)
					std::cout<<i<<" ";
				std::cout<<std::endl;
				std::cout<<"DETECTED BLOCKS - X :";
				for(auto i : right_detect_x)
					std::cout<<i<<" ";
				std::cout<<std::endl;
				std::cout<<"DETECTED BLOCKS - Y :";
				for(auto i : right_detect_y)
					std::cout<<i<<" ";
				std::cout<<std::endl<<std::endl;
				break;
			}
			if(cord_x[0]==-1){ //end of program
				return 0;
			}
			int dim_cycle = cord_x.size();
			std::cout<<"ROTATE POSITIONS: ";
			for (auto i : rotate_occupation)
				std::cout<<i<<" ";
			std::cout<<std::endl;

			std::cout<<"ORIENTATION: ";
			for (auto i : orientation)
				std::cout<<i<<" ";
			std::cout<<std::endl;
			
			for(int i=0;i<3;i++){
				if(rotate_occupation[i]==true){ //at first, work on the blocks in rotate position and try to get to their natural 
					index = nearest_cv(rotate_x[i],rotate_y[i]);
					x = cord_x[index];
					y = cord_y[index];
					l=l1[index];
					w=l2[index];
					h=cord_z[index];
					orient=orientation[index];
					a=angle[index];
					switch (orient){
						case 0:
							
							if( (a<0.2 and a > -0.2) or ((a<M_PI_2 + 0.2) and (a > M_PI_2 - 0.2)) or ((a<-M_PI_2 + 0.2) and (a > -M_PI_2 - 0.2)) or ((a<M_PI + 0.2) and (a > M_PI - 0.2))){
								std::cout<<"FROM POSITION "<<i<<" ROTATE M_PI/4"<<std::endl;
								rotate_PI_4(x,y,l,w,h,M_PI_2,i,n,loop_rate);
							}
							else{
								std::cout<<"BLOCK "<<assign_name(name[index])<<" FROM POSITION "<<i<<" TO INITIAL POSITION"<<std::endl;
								to_initial_position(name[index],x,y,l,w,h,a,i,n,loop_rate);
								
							}
							orientation[index] = -1;
							
							break;
						case 1:
							std::cout<<"FROM POSITION "<<i<<" ROTATE ORIENTATION 1"<<std::endl;
							orientation1(x,y,l,w,h,a,i,n,loop_rate);
							if(w>l)
								rotate_PI_4(x+w/2,y,l,w,h,M_PI_2,i,n,loop_rate);
							else
								rotate_PI_4(x+l/2,y,l,w,h,M_PI_2,i,n,loop_rate);
							orientation[index] = -1;
							break;
						case 2:
							if((w>0.075 || l> 0.075) && h < 0.05){
								std::cout<<"FROM POSITION "<<i<<" ROTATE ORIENTATION 2 DOUBLE"<<std::endl;
								orientation2_double(x,y,l,w,h,a,i,n,loop_rate);
							}
							else{
								std::cout<<"FROM POSITION "<<i<<" ROTATE ORIENTATION 2"<<std::endl;
								orientation2(x,y,l,w,h,a + M_PI_2,i,n,loop_rate);
							}
							orientation[index] = -1;
							break;
					}
					default_pos(n, loop_rate,1);
				}
			}
			
			
			
			
			for (int i = 0; i < dim_cycle; i++)
			{
				if(orientation[i]!=1 || orientation[i]==-1)
					continue;
				int freepos = free_position();
				if(freepos == -1)
					continue;

				std::cout<<"MOVE TO ROTATE POSITION "<<freepos<<" ORIENTATION 1"<<std::endl;
				x = cord_x[i];
				y = cord_y[i];
				l=l1[i];
				w=l2[i];
				h=cord_z[i];
				rotate_pos_return_x[freepos] = x;
				rotate_pos_return_y[freepos] = y;
				orientation1(x,y,l,w,h,angle[i],freepos,n,loop_rate);
				if(w>l)
					rotate_PI_4(rotate_x[freepos]+w/2,rotate_y[freepos],l,w,h,M_PI_2,freepos,n,loop_rate);
				else
					rotate_PI_4(rotate_x[freepos]+l/2,rotate_y[freepos],l,w,h,M_PI_2,freepos,n,loop_rate);
				orientation[i] = -1;
				default_pos(n, loop_rate,1);
			}
			for (int i = 0; i < dim_cycle; i++)
			{
				if(orientation[i]!=2 || orientation[i]==-1)
					continue;
				int freepos = free_position();
				if(freepos == -1)
					continue;
				std::cout<<"MOVE TO ROTATE POSITION "<<freepos<<" ORIENTATION 2"<<std::endl;
				x = cord_x[i];
				y = cord_y[i];
				l=l1[i];
				w=l2[i];
				h=cord_z[i];
				rotate_pos_return_x[freepos] = x;
				rotate_pos_return_y[freepos] = y;
				orientation2(x,y,l,w,h,angle[i],freepos,n,loop_rate);
				orientation[i] = -1;
				default_pos(n, loop_rate,1);
			}
			for (int i = 0; i < dim_cycle; i++)
			{
				if(orientation[i]==-1 || already_taken(cord_x[i],cord_y[i]))
					continue;
				int freepos = free_position();
				if(freepos == -1)
					continue;
				x = cord_x[i];
				y = cord_y[i];
				l=l1[i];
				w=l2[i];
				h=cord_z[i];
				rotate_pos_return_x[freepos] = x;
				rotate_pos_return_y[freepos] = y;
				std::cout<<"MOVE TO ROTATE POSITION "<<freepos<<" ORIENTATION 0"<<std::endl;
				orientation0(x,y,l,w,h,angle[i],freepos,n,loop_rate);
				default_pos(n, loop_rate,1);
			}
			
			go_to_position(0,0.5,H_UP,0,n,loop_rate, 1, 0, 1);
			default_pos(n, loop_rate,1);
			std::cout<<std::endl<<"------------------------------------------------------"<<std::endl<<std::endl;
		}
		int index;
		switch(spawner_n){
			case 1:
				index = 0;
				for(int i=0;i<castle_1_name.size();i++){
					for(int j=0;j<right_detect_name.size();j++){
						if(castle_1_name[i] == right_detect_name[j]){
							index = j;
							break;
						}
					}
					std::cout<<"BLOCK "<<assign_name(right_detect_name[index])<<" TO FINAL POSITION"<<std::endl;
					to_final_position_ass4(right_detect_x[index],right_detect_y[index],castle_1_x[i],castle_1_y[i],castle_1_z[i],n,loop_rate);
					right_detect_name.erase(right_detect_name.begin()+index);
					right_detect_x.erase(right_detect_x.begin()+index);
					right_detect_y.erase(right_detect_y.begin()+index);
				}
				break;
			case 2:
				index = 0;
				for(int i=0;i<castle_2_name.size();i++){
					for(int j=0;j<right_detect_name.size();j++){
						if(castle_2_name[i] == right_detect_name[j]){
							index = j;
							break;
						}
					}
					std::cout<<"BLOCK "<<assign_name(right_detect_name[index])<<" TO FINAL POSITION"<<std::endl;
					to_final_position_ass4(right_detect_x[index],right_detect_y[index],castle_2_x[i],castle_2_y[i],castle_2_z[i],n,loop_rate);
					right_detect_name.erase(right_detect_name.begin()+index);
					right_detect_x.erase(right_detect_x.begin()+index);
					right_detect_y.erase(right_detect_y.begin()+index);
				}
				break;
		}
		std::cout<<std::endl<<"PROGRAM HAS ENDED"<<std::endl;
	}else{
		while(1){
			sleep(2);
			get_detected_blocks(n,loop_rate);
			get_detected_blocks(n,loop_rate);
			if(cord_x[0]==-1){
				std::cout<<std::endl<<"PROGRAM HAS ENDED\n";
				return 0;
			}
			int dim_cycle = cord_x.size();
			std::cout<<"ROTATE POSITIONS: ";
			for (auto i : rotate_occupation)
				std::cout<<i<<" ";
			std::cout<<std::endl;

			std::cout<<"ORIENTATION: ";
			for (auto i : orientation)
				std::cout<<i<<" ";
			std::cout<<std::endl;
			
			for(int i=0;i<3;i++){
				if(rotate_occupation[i]==true){
					index = nearest_cv(rotate_x[i],rotate_y[i]);
					x = cord_x[index];
					y = cord_y[index];
					l=l1[index];
					w=l2[index];
					h=cord_z[index];
					orient=orientation[index];
					a=angle[index];
					switch (orient){
						case 0:
							
							if( (a<0.2 and a > -0.2) or ((a<M_PI_2 + 0.2) and (a > M_PI_2 - 0.2)) or ((a<-M_PI_2 + 0.2) and (a > -M_PI_2 - 0.2)) or ((a<M_PI + 0.2) and (a > M_PI - 0.2))){
								std::cout<<"FROM POSITION "<<i<<" ROTATE M_PI/4"<<std::endl;
								rotate_PI_4(x,y,l,w,h,M_PI_2,i,n,loop_rate);
							}
							else{
								std::cout<<"BLOCK "<<assign_name(name[index])<<" FROM POSITION "<<i<<" TO FINAL POSITION"<<std::endl;
								to_final_position(name[index],x,y,l,w,h,a,i,n,loop_rate);
								
							}
							orientation[index] = -1;
							
							break;
						case 1:
							std::cout<<"FROM POSITION "<<i<<" ROTATE ORIENTATION 1"<<std::endl;
							orientation1(x,y,l,w,h,a,i,n,loop_rate);
							if(w>l)
								rotate_PI_4(x+w/2,y,l,w,h,M_PI_2,i,n,loop_rate);
							else
								rotate_PI_4(x+l/2,y,l,w,h,M_PI_2,i,n,loop_rate);
							orientation[index] = -1;
							break;
						case 2:
							if((w>0.075 || l> 0.075) && h < 0.05){
								std::cout<<"FROM POSITION "<<i<<" ROTATE ORIENTATION 2 DOUBLE"<<std::endl;
								orientation2_double(x,y,l,w,h,a,i,n,loop_rate);
							}
							else{
								std::cout<<"FROM POSITION "<<i<<" ROTATE ORIENTATION 2"<<std::endl;
								orientation2(x,y,l,w,h,a + M_PI_2,i,n,loop_rate);
							}
							orientation[index] = -1;
							break;
					}
					default_pos(n, loop_rate,1);
				}
			}
			
			
			
			
			for (int i = 0; i < dim_cycle; i++)
			{
				if(orientation[i]!=1 || orientation[i]==-1)
					continue;
				int freepos = free_position();
				if(freepos == -1)
					continue;
				std::cout<<"MOVE TO ROTATE POSITION "<<freepos<<" ORIENTATION 1"<<std::endl;
				x = cord_x[i];
				y = cord_y[i];
				l=l1[i];
				w=l2[i];
				h=cord_z[i];
				orientation1(x,y,l,w,h,angle[i],freepos,n,loop_rate);
				if(w>l)
					rotate_PI_4(rotate_x[freepos]+w/2,rotate_y[freepos],l,w,h,M_PI_2,freepos,n,loop_rate);
				else
					rotate_PI_4(rotate_x[freepos]+l/2,rotate_y[freepos],l,w,h,M_PI_2,freepos,n,loop_rate);
				orientation[i] = -1;
				default_pos(n, loop_rate,1);
			}
			for (int i = 0; i < dim_cycle; i++)
			{
				if(orientation[i]!=2 || orientation[i]==-1)
					continue;
				int freepos = free_position();
				if(freepos == -1)
					continue;
				std::cout<<"MOVE TO ROTATE POSITION "<<freepos<<" ORIENTATION 2"<<std::endl;
				x = cord_x[i];
				y = cord_y[i];
				l=l1[i];
				w=l2[i];
				h=cord_z[i];
				orientation2(x,y,l,w,h,angle[i],freepos,n,loop_rate);
				orientation[i] = -1;
				default_pos(n, loop_rate,1);
			}
			for (int i = 0; i < dim_cycle; i++)
			{
				if(orientation[i]==-1)
					continue;
				x = cord_x[i];
				y = cord_y[i];
				l=l1[i];
				w=l2[i];
				h=cord_z[i];
				int freepos = free_position();
				if(freepos == -1)
					continue;
				std::cout<<"MOVE TO ROTATE POSITION "<<freepos<<" ORIENTATION 0"<<std::endl;
				orientation0(x,y,l,w,h,angle[i],freepos,n,loop_rate);
				default_pos(n, loop_rate,1);
			}
			
			go_to_position(0,0.5,H_UP,0,n,loop_rate, 1, 0, 1);
			default_pos(n, loop_rate,1);
			std::cout<<std::endl<<"------------------------------------------------------"<<std::endl<<std::endl;
		}
	}
	return 0;
}

void get_cord_x(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
	cord_x.clear();
	for (auto i : msg->data)
		cord_x.push_back(i);
}
void get_cord_y(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
	cord_y.clear();
	for (auto i : msg->data)
		cord_y.push_back(i);
}
void get_cord_z(const std_msgs::Float64MultiArray::ConstPtr &msg){
	cord_z.clear();
	for (auto i : msg->data)
		cord_z.push_back(i);
}
void get_name(const std_msgs::Int64MultiArray::ConstPtr &msg)
{
	name.clear();
	for (auto i : msg->data)
		name.push_back(i);
}
void get_angle(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
	angle.clear();
	for (auto i : msg->data)
		angle.push_back(i);
}
void get_blocks_x(const std_msgs::Float64MultiArray::ConstPtr &msg){
	blocks_x.clear();
	for (auto i : msg->data)
		blocks_x.push_back(i);
}
void get_blocks_y(const std_msgs::Float64MultiArray::ConstPtr &msg){
	blocks_y.clear();
	for (auto i : msg->data)
		blocks_y.push_back(i);
}
void get_spawner_n(const std_msgs::Int16::ConstPtr &msg){
	spawner_n = msg->data;
}
void get_l1(const std_msgs::Float64MultiArray::ConstPtr &msg){
	l1.clear();
	for (auto i : msg->data)
		l1.push_back(i);
}
void get_l2(const std_msgs::Float64MultiArray::ConstPtr &msg){
	l2.clear();
	for (auto i : msg->data)
		l2.push_back(i);
}
void get_orientation(const std_msgs::Int64MultiArray::ConstPtr &msg)
{
	orientation.clear();
	for (auto i : msg->data)
		orientation.push_back(i);
}
void get_shoulder_pan_joint_position(const control_msgs::JointControllerState::ConstPtr &ctr_msg)
{
	jnt_pos_start(0) = ctr_msg->process_value;
}
void get_shoulder_lift_joint_position(const control_msgs::JointControllerState::ConstPtr &ctr_msg)
{
	jnt_pos_start(1) = ctr_msg->process_value;
}
void get_elbow_joint_position(const control_msgs::JointControllerState::ConstPtr &ctr_msg)
{
	jnt_pos_start(2) = ctr_msg->process_value;
}
void get_wrist_1_joint_position(const control_msgs::JointControllerState::ConstPtr &ctr_msg)
{
	jnt_pos_start(3) = ctr_msg->process_value;
}
void get_wrist_2_joint_position(const control_msgs::JointControllerState::ConstPtr &ctr_msg)
{
	jnt_pos_start(4) = ctr_msg->process_value;
}
void get_wrist_3_joint_position(const control_msgs::JointControllerState::ConstPtr &ctr_msg)
{
	jnt_pos_start(5) = ctr_msg->process_value;
}
void get_gripper_position(const control_msgs::GripperCommandActionGoal::ConstPtr &ctr_msg)
{
	gripper_pos = *(ctr_msg);
}
void go_to_position(float xp, float yp, float zp, double yaw,ros::NodeHandle &n, ros::Rate &loop_rate, bool action, bool def,int setting)
{
	
	joint_com_sub[0] = n.subscribe("/shoulder_pan_joint_position_controller/state", 1, get_shoulder_pan_joint_position);
	joint_com_sub[1] = n.subscribe("/shoulder_lift_joint_position_controller/state", 1, get_shoulder_lift_joint_position);
	joint_com_sub[2] = n.subscribe("/elbow_joint_position_controller/state", 1, get_elbow_joint_position);
	joint_com_sub[3] = n.subscribe("/wrist_1_joint_position_controller/state", 1, get_wrist_1_joint_position);
	joint_com_sub[4] = n.subscribe("/wrist_2_joint_position_controller/state", 1, get_wrist_2_joint_position);
	joint_com_sub[5] = n.subscribe("/wrist_3_joint_position_controller/state", 1, get_wrist_3_joint_position);
	if(first_time){
		first_time=false;
		for (int i = 0; i < 100; i++)
		{
			ros::spinOnce();
			loop_rate.sleep();
		}
	}else{
		for (int i = 0; i < 5; i++)
		{
			ros::spinOnce();
			loop_rate.sleep();
		}
	}
	Ur5 u1;
	ur5Direct(u1, jnt_pos_start);
	joint_com_pub[0] = n.advertise<std_msgs::Float64>("/shoulder_pan_joint_position_controller/command", 1000);
	joint_com_pub[1] = n.advertise<std_msgs::Float64>("/shoulder_lift_joint_position_controller/command", 1000);
	joint_com_pub[2] = n.advertise<std_msgs::Float64>("/elbow_joint_position_controller/command", 1000);
	joint_com_pub[3] = n.advertise<std_msgs::Float64>("/wrist_1_joint_position_controller/command", 1000);
	joint_com_pub[4] = n.advertise<std_msgs::Float64>("/wrist_2_joint_position_controller/command", 1000);
	joint_com_pub[5] = n.advertise<std_msgs::Float64>("/wrist_3_joint_position_controller/command", 1000);
	Position pf;
	pf.pe << -xp, -yp, zp;

	int dim = ceil(DEF_TIME / DEF_STEP);
	MatrixXd m;
	// for(auto i : jnt_pos_start)
	// 	std::cout<<i<<"  ";
	// std::cout<<std::endl;
	m = p2pMotion(jnt_pos_start, pf.pe, rot2eul(u1.p.Re), yaw,0, DEF_TIME, DEF_STEP, def,setting);

	for (int i = 0; i < 2; i++)
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	int count = 0;
	std_msgs::Float64 position[6];
	while (ros::ok())
	{
		while (count < dim)
		{
			for (int i = 0; i < 6; i++)
			{

				position[i].data = m(count, i + 1);
				joint_com_pub[i].publish(position[i]);
			}

			ros::spinOnce();
			loop_rate.sleep();
			++count;
		}
		for (int i = 0; i < 100; i++)
		{
			ros::spinOnce();
			loop_rate.sleep();
		}
		// if (def)
		// 	std::cout << "Moved to default position" << std::endl;
		// else
		// 	std::cout << "Moved to [" << xp << "," << yp << "," << zp << "] --- ACTION = ";
		break;
	}
	
}
void default_pos(ros::NodeHandle &n, ros::Rate &loop_rate,int setting)
{
	go_to_position(0, 0, 0, 0, n, loop_rate, 1, 1,setting);
}
void use_gripper(float g_effort, float g_pos, ros::NodeHandle &n, ros::Rate &loop_rate)
{
	gripper_com_sub = n.subscribe("/gripper_controller/gripper_cmd/goal", 1, get_gripper_position);
	for (int i = 0; i < 5; i++)
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	gripper_com_pub = n.advertise<control_msgs::GripperCommandActionGoal>("/gripper_controller/gripper_cmd/goal", 1000);
	gripper_pos.goal.command.max_effort = g_effort;
	gripper_pos.goal.command.position = g_pos;
	for (int i = 0; i < 2; i++)
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	int c = 0;
	while (ros::ok())
	{
		gripper_com_pub.publish(gripper_pos);
		for (int i = 0; i < 5; i++)
		{
			ros::spinOnce();
			loop_rate.sleep();
		}
		c++;
		if (c > 5)
			break;
	}
	std::cout << "Gripper used [effort:" << g_effort << " , position:" << g_pos << "]" << std::endl;
	sleep(2);
}
std::string assign_name(int n)
{
	std::string name;
	switch (n)
	{
	case 1120:
		name = "X1-Y1-Z2";
		break;
	case 1222:
		name = "X1-Y2-Z2-TWINFILLET";
		break;
	case 1210:
		name = "X1-Y2-Z1";
		break;
	case 1220:
		name = "X1-Y2-Z2";
		break;
	case 1223:
		name = "X1-Y2-Z2-CHAMFER";
		break;
	case 1321:
		name = "X1-Y3-Z2-FILLET";
		break;
	case 2221:
		name = "X2-Y2-Z2-FILLET";
		break;
	case 1410:
		name = "X1-Y4-Z1";
		break;
	case 1320:
		name = "X1-Y3-Z2";
		break;
	case 1420:
		name = "X1-Y4-Z2";
		break;
	case 2220:
		name = "X2-Y2-Z2";
		break;
	default:
		break;
	}
	return name;
}
void check_get_data(std::vector<bool> &v){
	if(v[0]==false&&cord_x.size()!=0){
		v[0]=true;
		cord_x_sub.shutdown();
	}
	if(v[1]==false&&cord_y.size()!=0){
		v[1]=true;
		cord_y_sub.shutdown();
	}
	if(v[2]==false&&cord_z.size()!=0){
		v[2]=true;
		cord_z_sub.shutdown();
	}
	if(v[3]==false&&angle.size()!=0){
		v[3]=true;
		angle_sub.shutdown();
	}
	if(v[4]==false&&l1.size()!=0){
		v[4]=true;
		l1_sub.shutdown();
	}
	if(v[5]==false&&l2.size()!=0){
		v[5]=true;
		l2_sub.shutdown();
	}
	if(v[6]==false&&orientation.size()!=0){
		v[6]=true;
		orientation_sub.shutdown();
	}
}
void get_detected_blocks(ros::NodeHandle &n, ros::Rate &loop_rate){
	std::vector<bool> get_data = {false,false,false,false,false,false,false};
	
	cord_x.clear();
	cord_y.clear();
	cord_z.clear();
	angle.clear();
	l1.clear();
	l2.clear();
	orientation.clear();

	cord_x_sub = n.subscribe("/cord_x_cv", 100, get_cord_x);
	cord_y_sub = n.subscribe("/cord_y_cv", 100, get_cord_y);
	cord_z_sub = n.subscribe("/cord_z_cv", 100, get_cord_z);
	angle_sub = n.subscribe("/angles", 100, get_angle);
	l1_sub = n.subscribe("/l1", 100, get_l1);
	l2_sub = n.subscribe("/l2", 100, get_l2);
	orientation_sub = n.subscribe("/orientation", 100, get_orientation);
	bool flag = true;

	while(flag){
		flag = false;
		ros::spinOnce();
		loop_rate.sleep();
		check_get_data(get_data);
		for (auto i: get_data ){
			if(i == false){
				flag = true;
				break;
			}	
		}		
	}

	// //CORD_X
	// std::cout << "Cord x : " << std::endl;
	// for (int i = 0; i < cord_x.size(); i++)
	// 	std::cout << "[" << i << "]=" << cord_x[i] << "  ";
	// std::cout << std::endl << std::endl;

	// //CORD_Y
	// std::cout << "Cord y : " << std::endl;
	// for (int i = 0; i < cord_y.size(); i++)
	// 	std::cout << "[" << i << "]=" << cord_y[i] << "  ";
	// std::cout << std::endl << std::endl;
	
	// //CORD_Z
	// std::cout << "Cord z : " << std::endl;
	// for (int i = 0; i < cord_z.size(); i++)
	// 	std::cout << "[" << i << "]=" << cord_z[i] << "  ";
	// std::cout << std::endl << std::endl;
	

	//NAME
	name.clear();
	name_sub = n.subscribe("/name", 100, get_name);
	for (int i = 0; i < 100; i++){
		ros::spinOnce();
		loop_rate.sleep();
		if(!name.empty())
			break;
	}
	name_sub.shutdown();
	// std::cout << "Name : " << std::endl;
	// for (int i = 0; i < name.size(); i++)
	// 	std::cout << "[" << i << "]=" << name[i] << "  ";
	// std::cout << std::endl << std::endl;

	// //ANGLE
	// std::cout << "angle : " << std::endl;
	// for (int i = 0; i < angle.size(); i++)
	// 	std::cout << "[" << i << "]=" << angle[i] << "  ";
	// std::cout << std::endl << std::endl;

	// //L1
	// std::cout << "l1 : " << std::endl;
	// for (int i = 0; i < l1.size(); i++)
	// 	std::cout << "[" << i << "]=" << l1[i] << "  ";
	// std::cout << std::endl << std::endl;
	
	// //L2
	// std::cout << "l2 : " << std::endl;
	// for (int i = 0; i < l2.size(); i++)
	// 	std::cout << "[" << i << "]=" << l2[i] << "  ";
	// std::cout << std::endl << std::endl;

	// //ORIENTATION
	// std::cout << "orientation : " << std::endl;
	// for (int i = 0; i < orientation.size(); i++)
	// 	std::cout << "[" << i << "]=" << orientation[i] << "  ";
	// std::cout << std::endl << std::endl;
}
void get_detected_blocks_file(){
	std::ifstream in("./src/position_control/src/input.txt");
	int N;
	in>>N;
	double xfile, yfile, anglefile;
	long int nfile;
	for(int i=0;i<N;i++){
		in>>nfile>>xfile>>yfile>>anglefile;
		name.push_back(nfile);
		cord_x.push_back(xfile);
		cord_y.push_back(yfile);
		angle.push_back(anglefile);
	}
}
void get_blocks_pos(ros::NodeHandle &n, ros::Rate &loop_rate){
	blocks_x.clear();
	blocks_x_sub = n.subscribe("/blocks_x", 100, get_blocks_x);
	for (int i = 0; i < 100; i++){
		ros::spinOnce();
		loop_rate.sleep();
		if(!blocks_x.empty())
			break;
	}
	blocks_x_sub.shutdown();
	// std::cout << "Cord x : " << std::endl;
	// for (int i = 0; i < blocks_x.size(); i++)
	// 	std::cout << "[" << i << "]=" << blocks_x[i] << "  ";
	// std::cout << std::endl << std::endl;
	blocks_y.clear();
	blocks_y_sub = n.subscribe("/blocks_y", 100, get_blocks_y);
	for (int i = 0; i < 100; i++){
		ros::spinOnce();
		loop_rate.sleep();
		if(!blocks_y.empty())
			break;
	}
	blocks_y_sub.shutdown();
	// std::cout << "Cord y : " << std::endl;
	// for (int i = 0; i < blocks_y.size(); i++)
	// 	std::cout << "[" << i << "]=" << blocks_y[i] << "  ";
	// std::cout << std::endl << std::endl;
	spawner_n_sub = n.subscribe("/spawner_n", 100, get_spawner_n);
	for (int i = 0; i < 100; i++){
		ros::spinOnce();
		loop_rate.sleep();
	}
	spawner_n_sub.shutdown();
	switch(spawner_n){
		case 0:
			std::cout<<"ASSIGNMENT 1-2-3"<<std::endl;
			break;
		default:
			std::cout<<"ASSIGNMENT 4 -- CASTLE "<<spawner_n<<std::endl;
	}
}
void attach(std::string block){
	req.model_name_1 = "robot";
	req.link_name_1 = "wrist_3_link";
	req.model_name_2 = block;
	req.link_name_2 = "link";
	ros::service::waitForService("/link_attacher_node/attach", ros::Duration(-1));
	ros::service::call("/link_attacher_node/attach", req, res);
	sleep(1);
}
void detach(std::string block){
	req.model_name_1 = "robot";
	req.link_name_1 = "wrist_3_link";
	req.model_name_2 = block;
	req.link_name_2 = "link";
	ros::service::waitForService("/link_attacher_node/detach", ros::Duration(-1));
	ros::service::call("/link_attacher_node/detach", req, res);
	sleep(1);
}
double distance(double x1, double y1, double x2, double y2){
	return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}
int nearest(double x, double y){
	int min = 0;
	double mindist = distance(x,y,blocks_x[0],blocks_y[0]);
	double dist;
	for(int i=1;i<blocks_x.size();i++){
		dist=distance(x,y,blocks_x[i],blocks_y[i]);
		if(dist<mindist){
			mindist=dist;
			min=i;
		}
	}
	return min;
}
int free_position(){
	for(int i=0;i<3;i++){
		if(!rotate_occupation[i]){
			rotate_occupation[i] = true;
			return i;
		}
	}
	return -1;
}
int nearest_cv(double x, double y){

	int min = 0;
	double mindist = distance(x,y,cord_x[0],cord_y[0]);
	double dist;
	for(int i=1;i<cord_x.size();i++){
		dist=distance(x,y,cord_x[i],cord_y[i]);
		if(dist<mindist){
			mindist=dist;
			min=i;
		}
	}
	return min;
}
bool already_taken(double x, double y){
	for(int j=0;j<right_detect_name.size();j++){
		if(distance(x,y,right_detect_x[j],right_detect_y[j]) < 0.07)
			return true;
	}
	return false;
}
void to_final_position(int id, double x, double y, double l, double w, double h, double yaw, int freepos, ros::NodeHandle &n, ros::Rate &loop_rate){
	rotate_occupation[freepos] = false;
	go_to_position(x, y, H_UP, yaw, n, loop_rate, 1, 0, 1);
	go_to_position(x, y, H_DOWN,yaw, n, loop_rate, 1, 0, 1);
	int index = nearest(x,y);
	attach("block" + std::to_string(index));
	go_to_position(x, y, H_UP,yaw, n, loop_rate, 1, 0, 1);
	go_to_position(final_pos_x[final_pos_index[id]],final_pos_y[final_pos_index[id]],H_UP,final_pos_yaw[final_pos_index[id]],n,loop_rate, 1, 0, 1);
	go_to_position(final_pos_x[final_pos_index[id]],final_pos_y[final_pos_index[id]],H_DOWN + final_pos_occupation[final_pos_index[id]]*block_height[final_pos_index[id]],final_pos_yaw[final_pos_index[id]],n,loop_rate, 1, 0, 1);
	
	blocks_x[index] = final_pos_x[final_pos_index[id]];
	blocks_y[index] = final_pos_y[final_pos_index[id]];
	detach("block" + std::to_string(index));

	go_to_position(final_pos_x[final_pos_index[id]],final_pos_y[final_pos_index[id]],H_UP,final_pos_yaw[final_pos_index[id]],n,loop_rate, 1, 0, 1);
	//default_pos(n, loop_rate,1);
	final_pos_occupation[final_pos_index[id]]++;
}
void to_final_position_ass4(double x, double y, double xf, double yf, double zf, ros::NodeHandle &n, ros::Rate &loop_rate){
	go_to_position(x, y, H_UP, 0, n, loop_rate, 1, 0, 1);
	go_to_position(x, y, H_DOWN,0, n, loop_rate, 1, 0, 1);
	int index = nearest(x,y);
	attach("block" + std::to_string(index));
	go_to_position(x, y, H_UP,0, n, loop_rate, 1, 0, 1);
	go_to_position(xf,yf,H_UP,0,n,loop_rate, 1, 0, 1);
	go_to_position(xf,yf,H_DOWN +zf ,0,n,loop_rate, 1, 0, 1);
	
	blocks_x[index] = xf;
	blocks_y[index] = yf;
	detach("block" + std::to_string(index));

	go_to_position(xf,yf,H_UP,0,n,loop_rate, 1, 0, 1);
	//default_pos(n, loop_rate,1);
}
void to_initial_position(int id, double x, double y, double l, double w, double h, double yaw, int freepos, ros::NodeHandle &n, ros::Rate &loop_rate){
	rotate_occupation[freepos] = false;
	go_to_position(x, y, H_UP, yaw, n, loop_rate, 1, 0, 1);
	go_to_position(x, y, H_DOWN,yaw, n, loop_rate, 1, 0, 1);
	int index = nearest(x,y);
	attach("block" + std::to_string(index));
	go_to_position(x, y, H_UP,yaw, n, loop_rate, 1, 0, 1);
	go_to_position(rotate_pos_return_x[freepos],rotate_pos_return_y[freepos],H_UP,0,n,loop_rate, 1, 0, 1);
	go_to_position(rotate_pos_return_x[freepos],rotate_pos_return_y[freepos],H_DOWN,0,n,loop_rate, 1, 0, 1);	

	blocks_x[index] = rotate_pos_return_x[freepos];
	blocks_y[index] = rotate_pos_return_y[freepos];
	detach("block" + std::to_string(index));
	right_detect_name.push_back(id);
	right_detect_x.push_back(rotate_pos_return_x[freepos]);
	right_detect_y.push_back(rotate_pos_return_y[freepos]);
	go_to_position(rotate_pos_return_x[freepos],rotate_pos_return_y[freepos],H_UP,0,n,loop_rate, 1, 0, 1);
	//default_pos(n, loop_rate,1);
}
void rotate_PI_4(double x, double y, double l, double w, double h, double yaw, int freepos, ros::NodeHandle &n, ros::Rate &loop_rate){

	go_to_position(x, y, H_UP, yaw, n, loop_rate, 1, 0, 1);
	go_to_position(x, y, H_DOWN,yaw, n, loop_rate, 1, 0, 1);
	int index = nearest(x,y);
	attach("block" + std::to_string(index));

	go_to_position(x, y, H_DOWN + 0.05,yaw, n, loop_rate, 1, 0, 1);
	go_to_position(rotate_x[freepos],rotate_y[freepos],H_DOWN + 0.05, M_PI_4,n,loop_rate,1,0,1);
	go_to_position(rotate_x[freepos],rotate_y[freepos],H_DOWN, M_PI_4,n,loop_rate,1,0,1);
	blocks_x[index] = rotate_x[freepos];
	blocks_y[index] = rotate_y[freepos];
	detach("block" + std::to_string(index));
	
	go_to_position(rotate_x[freepos],rotate_y[freepos],H_UP, M_PI_4,n,loop_rate,1,0,1);
}
void orientation0(double x, double y, double l, double w, double h, double yaw, int freepos, ros::NodeHandle &n, ros::Rate &loop_rate){
	go_to_position(x, y, H_UP, yaw, n, loop_rate, 1, 0, 1);
	go_to_position(x, y, H_DOWN,yaw, n, loop_rate, 1, 0, 1);
	int index = nearest(x,y);
	attach("block" + std::to_string(index));

	go_to_position(x, y, H_UP,yaw, n, loop_rate, 1, 0, 1);
	go_to_position(rotate_x[freepos],rotate_y[freepos],H_UP, M_PI_4,n,loop_rate,1,0,1);
	go_to_position(rotate_x[freepos],rotate_y[freepos],H_DOWN, M_PI_4,n,loop_rate,1,0,1);
	blocks_x[index] = rotate_x[freepos];
	blocks_y[index] = rotate_y[freepos];
	detach("block" + std::to_string(index));
	
	go_to_position(rotate_x[freepos],rotate_y[freepos],H_UP, M_PI_4,n,loop_rate,1,0,1);
	//default_pos(n, loop_rate,1);
}
void orientation1(double x, double y, double l, double w, double h, double yaw, int freepos, ros::NodeHandle &n, ros::Rate &loop_rate){
	double delta_h = l;
	if(w>l)
		delta_h = w;
	go_to_position(x, y, H_UP, yaw, n, loop_rate, 1, 0, 1);
	go_to_position(x, y, H_DOWN,yaw, n, loop_rate, 1, 0, 1);

	int index = nearest(x,y);
	attach("block" + std::to_string(index));
	go_to_position(x, y, H_UP, yaw, n, loop_rate, 1, 0, 1);
	go_to_position(rotate_x[freepos], rotate_y[freepos]+delta_rotate[freepos], H_UP, yaw, n, loop_rate, 1, 0, 1);

	//Change orientation
	go_to_position(rotate_x[freepos], rotate_y[freepos]+delta_rotate[freepos], H_UP, yaw, n, loop_rate, 1, 0, 3);
	// GRIPPER DOWN
	go_to_position(rotate_x[freepos]-DELTA_GROUND, rotate_y[freepos]+delta_rotate[freepos], delta_h/2 + 0.03, yaw, n, loop_rate, 1, 0, 0);

	//DETACH
	detach("block" + std::to_string(index));

	go_to_position(rotate_x[freepos]-DELTA_GROUND, rotate_y[freepos]+delta_rotate[freepos], H_UP, yaw, n, loop_rate, 1, 0, 0);
	//Change orientation
	go_to_position(rotate_x[freepos]-DELTA_GROUND, rotate_y[freepos]+delta_rotate[freepos], H_UP, yaw, n, loop_rate, 1, 0, 2);
	//default_pos(n, loop_rate,1);


	//SECOND rotation
	go_to_position(rotate_x[freepos], rotate_y[freepos], H_UP,M_PI_2, n, loop_rate, 1, 0, 1);
	go_to_position(rotate_x[freepos], rotate_y[freepos], H_DOWN + delta_h - 0.055, M_PI_2, n, loop_rate, 1, 0, 1);
	attach("block" + std::to_string(index));
	go_to_position(rotate_x[freepos], rotate_y[freepos], H_UP,M_PI_2, n, loop_rate, 1, 0, 1);

	//Change orientation
	go_to_position(rotate_x[freepos], rotate_y[freepos]+delta_rotate[freepos], H_UP, yaw, n, loop_rate, 1, 0, 3);
	// GRIPPER DOWN
	go_to_position(rotate_x[freepos]-DELTA_GROUND, rotate_y[freepos]+delta_rotate[freepos], 0.05, yaw, n, loop_rate, 1, 0, 0);

	//UPDATE AND DETACH
	blocks_x[index] = rotate_x[freepos];
	blocks_y[index] = rotate_y[freepos];
	detach("block" + std::to_string(index));

	go_to_position(rotate_x[freepos]-DELTA_GROUND, rotate_y[freepos]+delta_rotate[freepos], H_UP, yaw, n, loop_rate, 1, 0, 0);
	//Change orientation
	go_to_position(rotate_x[freepos]-DELTA_GROUND, rotate_y[freepos]+delta_rotate[freepos], H_UP, yaw, n, loop_rate, 1, 0, 2);
	//default_pos(n, loop_rate,1);
}
void orientation2(double x, double y, double l, double w, double h, double yaw, int freepos, ros::NodeHandle &n, ros::Rate &loop_rate){
	double delta_h = l;
	double delta_a = 0;
	if(w>l)
		delta_h = w;
	if(h > 0.063)
		delta_a = M_PI_2;
	go_to_position(x, y, H_UP, yaw + delta_a, n, loop_rate, 1, 0, 1);
	go_to_position(x, y, H_DOWN + h - 0.05,yaw + delta_a, n, loop_rate, 1, 0, 1);

	int index = nearest(x,y);
	attach("block" + std::to_string(index));
	go_to_position(x, y, H_UP, yaw + delta_a, n, loop_rate, 1, 0, 1);
	go_to_position(rotate_x[freepos], rotate_y[freepos]+delta_rotate[freepos], H_UP, yaw, n, loop_rate, 1, 0, 1);

	//Change orientation
	go_to_position(rotate_x[freepos], rotate_y[freepos]+delta_rotate[freepos], H_UP, yaw, n, loop_rate, 1, 0, 3);
	// GRIPPER DOWN
	go_to_position(rotate_x[freepos]-DELTA_GROUND, rotate_y[freepos]+delta_rotate[freepos], delta_h/2, yaw, n, loop_rate, 1, 0, 0);

	//DETACH
	blocks_x[index] = rotate_x[freepos];
	blocks_y[index] = rotate_y[freepos];
	detach("block" + std::to_string(index));

	go_to_position(rotate_x[freepos]-DELTA_GROUND, rotate_y[freepos]+delta_rotate[freepos], H_UP, yaw, n, loop_rate, 1, 0, 0);
	//Change orientation
	go_to_position(rotate_x[freepos]-DELTA_GROUND, rotate_y[freepos]+delta_rotate[freepos], H_UP, yaw, n, loop_rate, 1, 0, 2);
	//default_pos(n, loop_rate,1);
}
void orientation2_double(double x, double y, double l, double w, double h, double yaw, int freepos, ros::NodeHandle &n, ros::Rate &loop_rate){
	double delta_h = h;
	double delta_a = 0;

	go_to_position(x, y, H_UP, yaw + delta_a, n, loop_rate, 1, 0, 1);
	go_to_position(x, y, H_DOWN + h - 0.055,yaw + delta_a, n, loop_rate, 1, 0, 1);

	int index = nearest(x,y);
	attach("block" + std::to_string(index));
	go_to_position(x, y, H_UP, yaw + delta_a, n, loop_rate, 1, 0, 1);
	go_to_position(rotate_x[freepos], rotate_y[freepos]+delta_rotate[freepos], H_UP, yaw, n, loop_rate, 1, 0, 1);

	//Change orientation
	go_to_position(rotate_x[freepos], rotate_y[freepos]+delta_rotate[freepos], H_UP, yaw, n, loop_rate, 1, 0, 3);
	// GRIPPER DOWN
	go_to_position(rotate_x[freepos]-DELTA_GROUND, rotate_y[freepos]+delta_rotate[freepos], delta_h/2 + 0.05, yaw, n, loop_rate, 1, 0, 0);

	//DETACH
	detach("block" + std::to_string(index));

	go_to_position(rotate_x[freepos]-DELTA_GROUND, rotate_y[freepos]+delta_rotate[freepos], H_UP, yaw, n, loop_rate, 1, 0, 0);
	//Change orientation
	go_to_position(rotate_x[freepos]-DELTA_GROUND, rotate_y[freepos]+delta_rotate[freepos], H_UP, yaw, n, loop_rate, 1, 0, 2);
	//default_pos(n, loop_rate,1);


	//SECOND ROTATION
	delta_a += M_PI_2;
	if(w>l)
		delta_h=w;
	else
		delta_h=l;
	// go_to_position(x, y, H_UP, yaw + delta_a, n, loop_rate, 1, 0, 1);
	// go_to_position(x, y, H_DOWN + h - 0.055,yaw + delta_a, n, loop_rate, 1, 0, 1);

	// attach("block" + std::to_string(index));
	// go_to_position(x, y, H_UP, yaw + delta_a, n, loop_rate, 1, 0, 1);

	go_to_position(rotate_x[freepos], rotate_y[freepos], H_UP, yaw + delta_a, n, loop_rate, 1, 0, 1);
	go_to_position(rotate_x[freepos], rotate_y[freepos], H_DOWN + delta_h - 0.055,  yaw + delta_a, n, loop_rate, 1, 0, 1);
	attach("block" + std::to_string(index));
	go_to_position(rotate_x[freepos], rotate_y[freepos], H_UP, yaw + delta_a, n, loop_rate, 1, 0, 1);


	go_to_position(rotate_x[freepos], rotate_y[freepos]+delta_rotate[freepos], H_UP, yaw, n, loop_rate, 1, 0, 1);

	//Change orientation
	go_to_position(rotate_x[freepos], rotate_y[freepos]+delta_rotate[freepos], H_UP, yaw, n, loop_rate, 1, 0, 3);
	// GRIPPER DOWN
	go_to_position(rotate_x[freepos]-DELTA_GROUND, rotate_y[freepos]+delta_rotate[freepos], delta_h/2 + 0.03, yaw, n, loop_rate, 1, 0, 0);

	//DETACH
	blocks_x[index] = rotate_x[freepos];
	blocks_y[index] = rotate_y[freepos];
	detach("block" + std::to_string(index));

	go_to_position(rotate_x[freepos]-DELTA_GROUND, rotate_y[freepos]+delta_rotate[freepos], H_UP, yaw, n, loop_rate, 1, 0, 0);
	//Change orientation
	go_to_position(rotate_x[freepos]-DELTA_GROUND, rotate_y[freepos]+delta_rotate[freepos], H_UP, yaw, n, loop_rate, 1, 0, 2);
	//default_pos(n, loop_rate,1);
}
