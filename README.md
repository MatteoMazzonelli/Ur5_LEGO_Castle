# Build

````
mkdir -p d_robot/catkin_ws
cd d_robot/catkin_ws
unzip src.zip
catkin_make
source devel/setup.bash
````


# Launch

Choose between setup files: 1, 2, 3, 4_1, 4_2
````
roslaunch ur5_gazebo ur5_setup1.launch
roslaunch yolo-detection yolo_detect_launcher.launch
````


# Run demo

````
rosrun position_control position_controller
````

# Assignment 3
![image text](docs/img/Assignment 3.png "Assignment 3")
# Assignment 4
![image text](docs/img/Assignment 4.png "Assignment 4")