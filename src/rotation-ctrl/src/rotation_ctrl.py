#!/usr/bin/env python3
from re import A
from time import sleep
from turtle import window_height
import cv2
import numpy as np
from matplotlib import pyplot as plt
import rospy
import math
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int64MultiArray
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
import sensor_msgs.point_cloud2 as pc2
from pylab import array, plot, show, axis, arange, figure, uint8 
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()

#given 4 src points in pixel coordinates and the related 4 real world coordinates return the perspective transform which can be used to transform any pixel coordinate into real world coordinate
src_points = np.float32([[246, 330],[776, 330],[246, 682],[776, 682]])
dst_points = np.float32([[0.5,0.3],[0.5,-0.3],[0.1,0.3],[0.1,-0.3]])
pt = cv2.getPerspectiveTransform(src_points,dst_points)

#Lists of data of yolo and opencv
not_sorted_angles = [] #List of angles detected by openCv, not yet sorted
sorted_angles = [] #List of angles sorted by blocks names give by yolo
points_cv = [] #list of coordinates of bboxes detected by opencv
cord_x_cv = [] #list of final cord x calculated by opencv
cord_y_cv =[] #list of final cord y calculated by opencv
cord_z_cv = [] #list of final cord z calculated by opencv
l1_list = [] #list of lengths of first side of detected bbox
l2_list = []#list of lengths of second side of detected bbox
orientation = []#list of blocks orientation (1 = upside down, 2 = on one side, 0 = normal position)
circles_centers = []#list of centers of detected circles
rect_list = []#list of detected rectangles
rect_box_list = []#list of detected bboxes
contour_list = []#list of contours
points_yolo = [] #list of coordinates of blocks detected by yolo
cord_x_yolo = [] #list of cord x given by yolo
cord_y_yolo = [] #list of cord y given by yolo


xyz = [] #list of pcl

KINECT_H = 1.02 #kinect z coordinate

#kinect image dimesions
P_LIMIT = 760 #pixel limit 
HEIGHT = P_LIMIT #image H
WIDTH = 1024 #image W

W_H_MIN_PIXEL = 15#min dimesion of bbox

point_cloud = PointCloud2()#create pcloud variable
image_angles = np.zeros((HEIGHT,WIDTH),np.uint8)#variable that will rappresent tthe kinect image

#Given the pcl list and a pixel point return the pcl value of the pixel point
def pixelTo3DPoint(u, v):
    global xyz
    length = len(xyz)    
    index = v*1024 + u 
    if(index<length):
        return xyz[index]
    else:
        return (0,0,0)

#Using perspective transform calculates the real world coordinate from pixel coordinate
def getRealPoint(p):
    global pt

    (x,y) = p

    point = np.float32([x,y,1])
    tmp_point = np.dot(pt,point)
    real_point = [tmp_point[0]/tmp_point[2],tmp_point[1]/tmp_point[2]]

    return real_point


#From a given rotated rectangle returns the angle in Gazebo format
def getAngle(calculatedRect):

    (x,y),(width,height),angle = calculatedRect
    if(angle == 90):
        if(height>width):
            angle = 0
    else:
        if(width<height):
            angle = 90-angle
        else:
            angle = -angle

    angle = np.radians(angle)
    return angle


#Controls if a point is inside a given contour
def isInside(p,rect):
    global rect_box_list
    i = rect_list.index(rect) 

    contour = contour_list[i]
    res = cv2.pointPolygonTest(contour,p,False)

    if res>=0:
        return True
    return False


#Distance of two points
def distance(p1,p2):
    x1 = p1[0]
    x2 = p2[0]
    y1 = p1[1]
    y2 = p2[1]
    dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return dist

#Controls if the detetcted bboxes are bbox of a lego block and removes the wrong ones.
def rect_control():
    global rect_list,points_cv,not_sorted_angles,rect_box_list,contour_list

    to_remove_list = []
    for rect in rect_list:
        (x,y),(w,h),angle = rect
        for r in rect_list:                 
            if(r != rect):
                if (r[1][0]<W_H_MIN_PIXEL or r[1][1]<W_H_MIN_PIXEL) or ((w*h > r[1][0]*r[1][1]) and isInside(r[0],rect)):
                    to_remove_list.append(rect_list.index(r))
    
    to_remove_list = list(dict.fromkeys(to_remove_list))
        
    for i in sorted(to_remove_list, reverse=True):
        rect_list.pop(i)
        points_cv.pop(i)
        not_sorted_angles.pop(i)
        rect_box_list.pop(i)
        contour_list.pop(i)

#----------------------------------------------------------------------------------------------------

#Detects circles in img_angles.jpg. We use them for orientation control
def detect_circles():
    global circles_centers,image_angles
    circles_centers.clear()
    
    #gray = cv2.imread('img_circles.jpg')
    img = image_angles

    #img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)

    #Splitting the LAB image to different channels
    l, a, b = cv2.split(lab)
    
    #Applying CLAHE to L-channel
    clahe = cv2.createCLAHE(clipLimit=8.0, tileGridSize=(1000,1000))
    cl = clahe.apply(l)

    #Merge the CLAHE enhanced L-channel with the a and b channel
    limg = cv2.merge((cl,a,b))

    #Converting image from LAB Color model to RGB model
    final = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)

    gray =  cv2.cvtColor(final, cv2.COLOR_BGR2GRAY)
    gray = cv2.blur(gray,(3,3))

    detected_circles = cv2.HoughCircles(gray, 
                    cv2.HOUGH_GRADIENT, dp=1.8,minDist=1, param1 = 30,
                param2 = 50, minRadius = 1, maxRadius = 20)
    
    # Draw circles that are detected.
    if detected_circles is not None:
    
        # Convert the circle parameters a, b and r to integers.
        detected_circles = np.uint16(np.around(detected_circles))
    
        for pt in detected_circles[0, :]:
            a, b, r = pt[0], pt[1], pt[2]
            circles_centers.append([a,b])

            #--->
            cv2.circle(gray, (a, b), r, (0, 255, 255), )
    
    cv2.imwrite("image_c.jpg",gray)
    #--->
    
#Take the rectangle detected and calculate the orientation and the z coordinate
def has_circle_inside(rect):
    global circles_centers,rect_list,xyz

    max_h = 0
    for c in circles_centers: 
        if(isInside(c,rect) ):
            #if the orientation has been chosen exit the loop
            p_cloud_pos = pixelTo3DPoint(c[0],c[1])
            d = p_cloud_pos[2]
            if(d>max_h):
                max_h = d
            if(max_h>=KINECT_H - 0.015 and max_h<=KINECT_H + 0.015):
                return (1,0.04)

    h = ctrl_highest_px(rect)
    z = KINECT_H - h
    if h<0.96 or  h>0.985:
        return (2,z)
    else:
        return (0,z)
                
        

#return lowest (so the highest z block coordinate) pcl z value inside a given bbox
def ctrl_highest_px(rect):
    global xyz, rect_list,rect_box_list
    index = rect_list.index(rect)
    box = rect_box_list[index]
    x_list = [i for i , j in box]
    y_list = [j for i , j in box]
    h_list = []
    for u in range(min(x_list),max(x_list)):
        for v in range(min(y_list),max(y_list)):
            if isInside((u,v),rect):
                p_cloud_pos = pixelTo3DPoint(u,v)
                h = p_cloud_pos[2]
                h_list.append(h)

    return min(h_list)
    




#check for every rectangle detetcted the orientation and the z coordinate
def detect_orientation():
    global rect_list,points_cv,point_cloud,xyz
    xyz = pc2.read_points(point_cloud,skip_nans=True)
    xyz = list(xyz) 
    
    for index,rect in enumerate(rect_list):
        
        ornt_and_z = has_circle_inside(rect)
        points_cv[index].append(ornt_and_z[0])
        points_cv[index].append(ornt_and_z[1])

    

#Detects bboxes in img_angles.jpg and puts the informations (angles,real points,bbox and rectangles) inside lists. Then calls the control functions
def detectShapes():
    global not_sorted_angles,rect_list,points_cv,rect_box_list,contour_list,image_angles
    not_sorted_angles.clear()
    rect_list.clear()
    points_cv.clear()
    rect_box_list.clear()
    contour_list.clear()
    
    #convert the image in hsv 
    img = image_angles

    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    COLOR_MIN = np.array([20, 80, 80],np.uint8)
    COLOR_MAX = np.array([40, 255, 255],np.uint8)
    
    frame_threshed = cv2.inRange(hsv_img, COLOR_MIN, COLOR_MAX)
    ret,thresh = cv2.threshold(frame_threshed,127,255,0)
    contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    areas = [cv2.contourArea(c) for c in contours]
    max_index = np.argmax(areas)
    

    for index,contour in enumerate(contours):
        # here we are ignoring first counter because 
        # findcontour function detects whole image as shape 
        if(index != max_index):
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            not_sorted_angles.append(getAngle(rect))
            points_cv.append(getRealPoint(rect[0]))
            rect_list.append(rect)
            rect_box_list.append(box)
            contour_list.append(contour)  
            
    rect_control()

    #--->
    for box in rect_box_list:
        cv2.drawContours(thresh,[box],0,(0,0,255),2)
    
    cv2.imwrite("image_a.jpg",thresh)
    #--->
    detect_circles()
    detect_orientation()

#sort all data in function of yolo detection order and add at the end the data of not detected blocks
def sortData():
    global points_yolo,points_cv,not_sorted_angles,sorted_angles,cord_x_cv,cord_y_cv,orientation,cord_z_cv,l1_list,l2_list

    orientation.clear()
    sorted_angles.clear()
    cord_x_cv.clear()
    cord_y_cv.clear()
    cord_z_cv.clear()
    l1_list.clear()
    l2_list.clear()
    indices = []

    p_len = len(points_cv)
    d_min = 100
    i = 100
   
    for p in points_yolo:  
        for q in points_cv: 
            if(distance(p,q)<d_min):
                d_min = distance(p,q)
                i = points_cv.index(q)
        
        if(p_len != 0):
            sorted_angles.append(not_sorted_angles[i])
            cord_x_cv.append(points_cv[i][0])
            cord_y_cv.append(points_cv[i][1])
            orientation.append(points_cv[i][2])
            cord_z_cv.append(points_cv[i][3])
            p1 = getRealPoint(rect_box_list[i][0])
            p2 = getRealPoint(rect_box_list[i][1]) 
            p3 = getRealPoint(rect_box_list[i][2])        
            l1_list.append(distance(p1,p2))
            l2_list.append(distance(p2,p3))
            indices.append(i)
            
        d_min = 100
        i = 100
    
    #Add not detected blocks data
    for q in points_cv:  
        i = points_cv.index(q)
        if(i not in indices): 
            sorted_angles.append(not_sorted_angles[i])
            cord_x_cv.append(q[0])
            cord_y_cv.append(q[1])
            orientation.append(q[2])
            cord_z_cv.append(q[3])
            p1 = getRealPoint(rect_box_list[i][0])
            p2 = getRealPoint(rect_box_list[i][1]) 
            p3 = getRealPoint(rect_box_list[i][2])        
            l1_list.append(distance(p1,p2))
            l2_list.append(distance(p2,p3))

#Publish all data
def publishData():
    global cord_y_cv,cord_x_cv,cord_z_cv,orientation,l1_list,l2_list,sorted_angles

    pub_ang = rospy.Publisher('angles',Float64MultiArray, queue_size=10)
    pub_x = rospy.Publisher('cord_x_cv',Float64MultiArray, queue_size=10)
    pub_y = rospy.Publisher('cord_y_cv',Float64MultiArray, queue_size=10)
    pub_z = rospy.Publisher('cord_z_cv',Float64MultiArray, queue_size=10)
    pub_l1 = rospy.Publisher('l1',Float64MultiArray, queue_size=10)
    pub_l2 = rospy.Publisher('l2',Float64MultiArray, queue_size=10)
    pub_orientation = rospy.Publisher('orientation',Int64MultiArray, queue_size=10)

    data_ang = Float64MultiArray()
    data_x = Float64MultiArray()
    data_y = Float64MultiArray()
    data_z = Float64MultiArray()
    data_l1 = Float64MultiArray()
    data_l2 = Float64MultiArray()
    data_or = Int64MultiArray()

    if len(cord_x_cv) == 0:
        cord_x_cv.append(-1)
        cord_y_cv.append(-1)
        cord_z_cv.append(-1)
        sorted_angles.append(-1)
        orientation.append(-1)
        l1_list.append(-1)
        l2_list.append(-1)



    data_x.data = cord_x_cv
    data_y.data = cord_y_cv
    data_z.data = cord_z_cv
    data_ang.data = sorted_angles
    data_or.data = orientation
    data_l1.data = l1_list
    data_l2.data = l2_list

    pub_x.publish(data_x)
    pub_y.publish(data_y)
    pub_z.publish(data_z)
    pub_ang.publish(data_ang)
    pub_orientation.publish(data_or)
    pub_l1.publish(data_l1)
    pub_l2.publish(data_l2)

#get cord_x from yolo
def getX(data):
    global cord_x_yolo
    cord_x_yolo = data.data

#get cord_y from yolo
def getY(data):
    global cord_y_yolo
    cord_y_yolo = data.data

#get pcl from yolo
def getP(data):
    global point_cloud
    point_cloud = data

#get image from the second kinect, the ortogonal one
def getImageAngles(data):
    global image_angles
    try:
        # Convert your ROS Image message to OpenCV2
        image_angles = bridge.imgmsg_to_cv2(data, "bgr8")
        image_angles = image_angles[:P_LIMIT,:]
    except CvBridgeError as e:
        print(e)
        # Save your OpenCV2 image as a jpeg 


if __name__ == '__main__':
    rospy.init_node('pos_manager', anonymous=True)
    rate = rospy.Rate(100)

    subX = rospy.Subscriber("cord_x", Float64MultiArray, getX)
    subY = rospy.Subscriber("cord_y", Float64MultiArray, getY)
    subImageAngles = rospy.Subscriber("/camera/color/image_raw_2", Image, getImageAngles)
    subP = rospy.Subscriber("/camera/depth/points_2", PointCloud2, getP)

    subX.callback
    subY.callback
    subP.callback
    subImageAngles.callback
    

    sleep(2)
        
    while not rospy.is_shutdown():
        #sleep(1)
        points_yolo = list(zip(cord_x_yolo,cord_y_yolo))
        detectShapes()
        sortData()
        publishData()
        
        #----Print results
        print("\nX_cv: ")
        round_cord_x_cv = [round(num,3) for num in cord_x_cv]
        print(round_cord_x_cv)

        print("\nY_cv: ")
        round_cord_y_cv = [round(num,3) for num in cord_y_cv]
        print(round_cord_y_cv)
    
        print("\nOrientation: ")
        print(orientation)
        print("---------------------------------\n\n")
        #----

        rate.sleep()
        
    
