#!/usr/bin/env python3
import random
import pathlib
import os
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
import numpy as np
import rospkg
from tf.transformations import quaternion_from_euler
import xml.etree.ElementTree as ET
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int16
import rospkg 

blocks_x = []
blocks_y = []
def funzione(pkgpath):
    global blocks_x,blocks_y
    initial_pose = Pose()
    initial_pose.position.x = 0
    initial_pose.position.y = 0
    initial_pose.position.z = 0

    roll = 0
    pitch = 0
    yaw = 0
    [xq,yq,zq,wq]=quaternion_from_euler(roll,pitch,yaw)
    #Conversion from  roll-pitch-yaw to quaternions
    initial_pose.orientation.x = xq
    initial_pose.orientation.y = yq
    initial_pose.orientation.z = zq
    initial_pose.orientation.w = wq

    listPossiblePos = [[0.15, -0.45], [0.15, -0.3], [0.15, 0.3], [0.15, 0.45],
                    [0.3, -0.45], [0.3, -0.3], [0.3, 0.3], [0.3, 0.45],
                    [0.45, -0.45], [0.45, -0.3], [0.45, 0.3], [0.45, 0.45]]

    listroll = [0]

    listpitch = [0]


    #number of blocks 
    n=11
    i=0
    
    path = pkgpath+"/models"

    listPos= random.sample(listPossiblePos, n)
    listBlocks = random.sample(range(11),n)

    for x in listBlocks:
        nm = x+1
        file_path = f"{path}"+"/model"+str(nm)+".sdf"
        tree = ET.parse(file_path)
        root = tree.getroot()
       
        root[0][0][3][2][0].text = "1 1 1 1" #ambient 
        root[0][0][3][2][1].text = "1 1 1 1" #diffuse 
        sdff = ET.tostring(root, encoding='unicode', method='xml')

        r = i+1
        nblock = "block"+str(i)

        rospy.wait_for_service('gazebo/spawn_sdf_model')
        spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
        initial_pose.position.x = listPos[i][0]
        initial_pose.position.y = listPos[i][1]
        blocks_x.append(listPos[i][0])
        blocks_y.append(listPos[i][1])
        roll = random.choice(listroll)
        pitch = random.choice(listpitch)
        yaw = random.uniform(-3.14,3.14)
        [xq,yq,zq,wq]=quaternion_from_euler(roll,pitch,yaw)
        initial_pose.orientation.x = xq
        initial_pose.orientation.y = yq
        initial_pose.orientation.z = zq
        initial_pose.orientation.w = wq
        spawn_model_prox(nblock, sdff, "", initial_pose, "world")
        i+=1

def publishBlocks(): 
    pub_x = rospy.Publisher('blocks_x',Float64MultiArray, queue_size=10)
    pub_y = rospy.Publisher('blocks_y',Float64MultiArray, queue_size=10)
    pub_n = rospy.Publisher('spawner_n',Int16, queue_size=10)
    data_x = Float64MultiArray()
    data_y = Float64MultiArray()
    data_n = Int16()
    data_x.data = blocks_x
    data_y.data = blocks_y
    data_n.data= 0
    pub_x.publish(data_x)
    pub_y.publish(data_y)
    pub_n.publish(data_n)


if __name__ == '__main__':
    rospy.init_node('spawn_manager', anonymous=True)
    rate = rospy.Rate(200)
    rospack = rospkg.RosPack()
    funzione(rospack.get_path('spawn_blocks'))
    while not rospy.is_shutdown():
        publishBlocks()
        rate.sleep()