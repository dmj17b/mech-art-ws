#!/bin/bash

# Install YOLOv3
wget https://pjreddie.com/media/files/yolov3.weights
wget https://raw.githubusercontent.com/pjreddie/darknet/master/cfg/yolov3.cfg

# Install YOLOv3-tiny
wget https://pjreddie.com/media/files/yolov3-tiny.weights
wget https://raw.githubusercontent.com/pjreddie/darknet/master/cfg/yolov3-tiny.cfg

# Install COCO dataset
wget https://raw.githubusercontent.com/pjreddie/darknet/master/data/coco.names