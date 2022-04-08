#!/usr/bin/env python3
import argparse
import os
from pickletools import uint8
import time
from time import sleep
from pathlib import Path
import torch
import torch.backends.cudnn as cudnn
from numpy import histogram, random
import sys
import rospy
import rospkg
from models.experimental import attempt_load
from utils.datasets import LoadStreams, LoadImages
from utils.general import check_img_size, check_requirements, check_imshow, non_max_suppression, apply_classifier, \
    scale_coords, xyxy2xywh, strip_optimizer, set_logging, increment_path
from utils.plots import plot_one_box
from utils.torch_utils import select_device, load_classifier, time_synchronized
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()


starttime = time.time()
rpaths =  rospkg.get_ros_package_path().replace(":/opt/ros/noetic/share","")
weights =rpaths +'/yolo-detection/src/yolov5/best.pt'
save_dir = Path('/tmp/position')
source = 'img.jpg'
save_txt = True
imgsz = 2048
small_add = 10
big_add = 20
HEIGHT = 2048
WIDTH = 2048
P_LIMIT = 815
HIST_CONF = 20
image = np.zeros((WIDTH,HEIGHT),np.uint8)
image_old = np.zeros((WIDTH,HEIGHT),np.uint8)
scene_ctrl = 1000


#function used by yolo
def letterbox(img, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True, stride=32):
    # Resize and pad image while meeting stride-multiple constraints
    shape = img.shape[:2]  # current shape [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    # Scale ratio (new / old)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    if not scaleup:  # only scale down, do not scale up (for better test mAP)
        r = min(r, 1.0)

    # Compute padding
    ratio = r, r  # width, height ratios
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding
    if auto:  # minimum rectangle
        dw, dh = np.mod(dw, stride), np.mod(dh, stride)  # wh padding
    elif scaleFill:  # stretch
        dw, dh = 0.0, 0.0
        new_unpad = (new_shape[1], new_shape[0])
        ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]  # width, height ratios

    dw /= 2  # divide padding into 2 sides
    dh /= 2

    if shape[::-1] != new_unpad:  # resize
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
    return img, ratio, (dw, dh)



def detect():
    
    #source, weights, save_txt, imgsz = opt.source, opt.weights, opt.save_txt, opt.img_size
    global save_dir, weights,source,imgsz,small_add,big_add
    (save_dir).mkdir(parents=True, exist_ok=True)
    global save_txt 
    # Initialize
    set_logging()
    device = select_device(opt.device)
    half = device.type != 'cpu'  # half precision only supported on CUDA

    # Load model
    model = attempt_load(weights, map_location=device)  # load FP32 model
    stride = int(model.stride.max())  # model stride
    imgsz = check_img_size(imgsz, s=stride)  # check img_size
    if half:
        model.half()  # to FP16

    # Second-stage classifier
    classify = False
    if classify:
        modelc = load_classifier(name='resnet101', n=2)  # initialize
        modelc.load_state_dict(torch.load(
            'weights/resnet101.pt', map_location=device)['model']).to(device).eval()

    # Set Dataloader
    #dataset = LoadImages(source, img_size=imgsz, stride=stride)
    im0s = image
    img = letterbox(im0s, imgsz, stride)[0]
    # Convert
    img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
    img = np.ascontiguousarray(img)

    # Get names and colors
    names = model.module.names if hasattr(model, 'module') else model.names

    # Run inference
    if device.type != 'cpu':
        model(torch.zeros(1, 3, imgsz, imgsz).to(device).type_as(
            next(model.parameters())))  # run once
    t0 = time.time()
    #for path, img, im0s, vid_cap in dataset:
    img = torch.from_numpy(img).to(device)
    img = img.half() if half else img.float()  # uint8 to fp16/32
    img /= 255.0  # 0 - 255 to 0.0 - 1.0
    if img.ndimension() == 3:
        img = img.unsqueeze(0)

    # Inference
    t1 = time_synchronized()
    pred = model(img, augment=opt.augment)[0]
    opt.agnostic_nms = True
    # Apply NMS
    pred = non_max_suppression(
        pred, opt.conf_thres, opt.iou_thres, classes=opt.classes, agnostic=opt.agnostic_nms)
    t2 = time_synchronized()

    # Apply Classifier
    if classify:
        pred = apply_classifier(pred, modelc, img, im0s)

    # Process detections
    for i, det in enumerate(pred):  # detections per image
        im0 = im0s
        txt_path = str(save_dir)+'/positions'
        # normalization gain whwh
        gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]

        # clear file
        with open(txt_path + '.txt', 'w') as f:
            f.close()

        if len(det):
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_coords(
                img.shape[2:], det[:, :4], im0.shape).round()

            # Write results
            opt.save_conf = True
            for *xyxy, conf, cls in reversed(det):
                if save_txt:  # Write to file
                    xywh = (xyxy2xywh(torch.tensor(xyxy).view(
                        1, 4)) / gn).view(-1).tolist()
                    line = (
                        cls, *xywh, conf) if opt.save_conf else (cls, *xywh)

                    x = line[1]*WIDTH 
                    y = ((line[2]*HEIGHT)+small_add) if (names[int(cls)] == "X1-Y2-Z1" or names[int(cls)] == "X1-Y4-Z1") else ((line[2]*HEIGHT) + big_add) 
                    
                    if(y>P_LIMIT):
                        # print(names[int(cls)])
                        with open(txt_path + '.txt', 'a') as f:
                            f.write('%s %f %f' %
                                    (names[int(cls)], x, y) + '\n')

def getImageYolo(data):
    global scene_ctrl,image,image_old
    histogram0 = []
    histogram1 = []
    
    try:
        
        image_old = image

        zero = np.zeros((WIDTH,HEIGHT),np.uint8)
        if(not np.array_equal(image_old,zero)):
            gray_image1 = cv2.cvtColor(image_old, cv2.COLOR_BGR2GRAY)
            gray_image1 = gray_image1[P_LIMIT:, :]
            histogram1 = cv2.calcHist([gray_image1], [0],
                                    None, [256], [0, 256])

        image = bridge.imgmsg_to_cv2(data, "bgr8")
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray_image = gray_image[P_LIMIT:, :]
        histogram0 = cv2.calcHist([gray_image], [0],
                                None, [256], [0, 256])
        if scene_ctrl<HIST_CONF:
            i = 0
            while i<len(histogram0) and i<len(histogram1):
                scene_ctrl+=(histogram0[i]-histogram1[i])**2
                i+= 1
            scene_ctrl = scene_ctrl**(1 / 2)

        cv2.imwrite("img.jpg",image)
        sleep(2)
        
    except CvBridgeError as e:
        print(e)



if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', nargs='+', type=str,
                        default='best.pt', help='model.pt path(s)')
    parser.add_argument('--source', type=str, default='img.jpg',
                        help='source')  # file/folder, 0 for webcam
    parser.add_argument('--img-size', type=int, default=2048,
                        help='inference size (pixels)')
    parser.add_argument('--conf-thres', type=float,
                        default=0.4, help='object confidence threshold')
    parser.add_argument('--iou-thres', type=float,
                        default=0.45, help='IOU threshold for NMS')
    parser.add_argument('--device', default='',
                        help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--view-img', action='store_true',
                        help='display results')
    parser.add_argument('--save-txt', action='store_true',
                        help='save results to *.txt')
    parser.add_argument('--save-conf', action='store_true',
                        help='save confidences in --save-txt labels')
    parser.add_argument('--classes', nargs='+', type=int,
                        help='filter by class: --class 0, or --class 0 2 3')
    parser.add_argument('--agnostic-nms', action='store_true',
                        help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true',
                        help='augmented inference')
    parser.add_argument('--update', action='store_true',
                        help='update all models')
    parser.add_argument('--project', default='runs/detect',
                        help='save results to project/name')
    parser.add_argument('--name', default='exp',
                        help='save results to project/name')
    parser.add_argument('--exist-ok', action='store_true',
                        help='existing project/name ok, do not increment')
    opt = parser.parse_args()
    
    rospy.init_node('pos_manager', anonymous=True)
    rate = rospy.Rate(100)
    subImageYolo = rospy.Subscriber("/camera/color/image_raw", Image, getImageYolo)
    subImageYolo.callback
    sleep(5)


    with torch.no_grad():
        
        detect()
        detect()
        print("YOLO HAS STARTED")
        n_detect = 1
        while True:
            while n_detect>0:
                detect()
                n_detect -= 1
                
            if(scene_ctrl>HIST_CONF):
                #print("\nDetection start")
                n_detect = 1
                scene_ctrl = 0
            
       
        
                
        