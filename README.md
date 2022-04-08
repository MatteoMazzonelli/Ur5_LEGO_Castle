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

# Assignment 1
There is only one object in the initial stand, which is positioned with its base “naturally” in contact with the ground. The object can be of any of the classes specified by the project.
Each class has an assigned position on the final stand, which is marked by a coloured shape representing the silhouette of the object. The robot has to detect the position of the object and move it from its initial and to its final position.

# Assignment 2
There are multiple objects on the initial stand, one for each class, with their base “naturally” in contact with the ground. Each object has to be picked up and stored in the position prescribed for its class and marked by the object’s silhouette.

# Assignment 3
There are multiple objects on the initial stand, and there can be more than one object for each class. The objects are positioned randomly on the stand. An object could be lying on
one of its lateral sides or on its top. Each object has to be stored in the position prescribed by its class. Objects of the same class have to be stacked up to form a tower.

<img src=docs/img/Assignment_3.png width="500" />

# Assignment 4
The objects on the initial stand are those needed to create a composite object with a known design (e.g. a castle). The objects are positioned randomly on the stand. An object could be lying on one of its lateral sides or on its top. The manipulator has to pick them up in sequence and create the desired composite object on the final stand.

<img src=docs/img/Assignment_4.png width="500" />