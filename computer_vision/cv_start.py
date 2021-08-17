#!/usr/bin/env python
import os

#Pathing to necessary file to run the cv_main.py script


path = os.path.dirname(os.path.abspath(__file__))

weights = path + "/yolov4_files/yolov4-tiny.weights"

cfg = path + "/yolov4_files/yolov4-camerabox.cfg"

data = path + "/yolov4_files/camerabox.data"

so = path + "/yolov4_files/"

os.environ["DARKNET_PATH"] = so


from cv_scripts.cv_main import main


if __name__ == '__main__':
    main(weights, cfg, data)
