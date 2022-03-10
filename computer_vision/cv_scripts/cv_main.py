#!/usr/bin/env python

import os
import cv2
import time
import robosub_darknet
import argparse
import random
import rospy
from computer_vision.msg import target
from parameters import *

"""
    Modifier: HERIBERTO GONZALEZ (gonzo-32), RICARDO MEDINA ()
    Main script (Non-Multithreading Version) for running the darknet+yolov4 Convolutional Neural Network:
        - import os:
        - import cv2:
        - import time:
        - import robosub_darknet:
        - import argparse:
        - import random:

        def parser():
        def check_arguments_errors(args):
        def str2int(video_path):
        def set_saved_video(input_video, output_video, size):

    To close the script press: ' q and Esc '
        
"""
def parser(weights, cfg, yoloData):
    parser = argparse.ArgumentParser(description="YOLO Object Detection")
    parser.add_argument("--input", type=str, default=-1,
                        help="video source. If empty, uses webcam 0 stream")

    parser.add_argument("--out_filename", type=str, default="outfile",
                        help="inference video name. Not saved if empty")

    parser.add_argument("--weights", default=weights,
                        help="yolo weights path")

    parser.add_argument("--dont_show", action='store_true',
                        help="windown inference display. For headless systems")

    parser.add_argument("--ext_output", action='store_true',
                        help="display bbox coordinates of detected objects")

    parser.add_argument("--config_file", default=cfg,
                        help="path to config file")

    parser.add_argument("--data_file", default=yoloData,
                        help="path to data file")

    parser.add_argument("--thresh", type=float, default=.55,
                        help="remove detections with confidence below this value")
    return parser.parse_args()

def check_arguments_errors(args):
    assert 0 < args.thresh < 1, "Threshold should be a float between zero and one (non-inclusive)"
    if not os.path.exists(args.config_file):
        raise(ValueError("Invalid config path {}".format(os.path.abspath(args.config_file))))
    if not os.path.exists(args.weights):
        raise(ValueError("Invalid weight path {}".format(os.path.abspath(args.weights))))
    if not os.path.exists(args.data_file):
        raise(ValueError("Invalid data file path {}".format(os.path.abspath(args.data_file))))
    if str2int(args.input) == str and not os.path.exists(args.input):
        raise(ValueError("Invalid video path {}".format(os.path.abspath(args.input))))


def str2int(video_path):
    """
    argparse returns and string althout webcam uses int (0, 1 ...)
    Cast to int if needed
    """
    try:
        return int(video_path)
    except ValueError:
        return video_path


def set_saved_video(input_video, output_video, size):
    fourcc = cv2.VideoWriter_fourcc(*"MJPG")
    fps = int(input_video.get(cv2.CAP_PROP_FPS))
    video = cv2.VideoWriter(output_video, fourcc, fps, size,)
    return video


def main(weights, cfg, yoloData):
    '''
        ROS INIT
    '''
    rospy.init_node('CV')
    cv_pub = rospy.Publisher('target', target, queue_size=10)
    data = target()
    '''
        ARGS passing in neccessary files 
    '''
    args = parser(weights, cfg, yoloData)

    check_arguments_errors(args)

    random.seed(5)  # deterministic bbox colors

    network, class_names, class_colors = robosub_darknet.load_network(
        args.config_file,
        args.data_file,
        args.weights
    )
    # robosub_darknet doesn't accept numpy images.
    # Create one with image we reuse for each detect
    width = robosub_darknet.network_width(network)
    height = robosub_darknet.network_height(network)
    print (width, height)
    robosub_darknet_image = robosub_darknet.make_image(width, height, 3)
    #aren:
    ScrCenter = [int(width/2), int(height/2)]
    degPpix = [(float(FOV_x)/ScrCenter[0]), (float(FOV_y)/ScrCenter[1])]
    noObjCounter = 9
    firstLoop = True

    input_path = str2int(args.input)
    cap = cv2.VideoCapture(input_path)
    video = set_saved_video(cap, args.out_filename, (width, height))

    while cap.isOpened():
        prev_time = time.time()
        ret, frame = cap.read()
        if not ret:
            break
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        frame_resized = cv2.resize(frame_rgb, (width, height),
                                   interpolation=cv2.INTER_LINEAR)

        robosub_darknet.copy_image_from_bytes(robosub_darknet_image, frame_resized.tobytes())
        detections = robosub_darknet.detect_image(network, class_names, robosub_darknet_image, thresh=args.thresh)
        image = robosub_darknet.draw_boxes(detections, frame_resized, class_colors)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        if args.out_filename is not None:
            video.write(image)
        fps = int(1/(time.time() - prev_time))
        if firstLoop:
            fps = 10
            firstLoop = False
        print("FPS: {}".format(fps))

        '''
            ROS OUTPUT
        '''
        #ros_output = robosub_darknet.ros_package(detections, True)
        ros_output = robosub_darknet.ros_package(detections,ScrCenter,degPpix,True)
	print (ros_output)
        #obj 1 will be buoy we want to detect, 2 will be other, 3 will be home base
        if ros_output is not None:
            noObjCounter = 0
        elif (noObjCounter < 4):
            ros_output = prevN, prevC, prevX, prevY, prevD
            noObjCounter = noObjCounter + 1
        try:
            if (ros_output[0] == "camera-box"):
                data.buoy1 = True
                data.buoy1x = ros_output[2]
                data.buoy1y = ros_output[3]
                data.buoy1_distance = ros_output[4]
            elif (ros_output[0] == "camera-box2"):
                data.buoy2 = True
                data.buoy2x = ros_output[2]
                data.buoy2y = ros_output[3]
                data.buoy2_distance = ros_output[4]
            elif (ros_output[0] == "camera-box3"):
                data.buoy3 = True
                data.buoy3x = ros_output[2]
                data.buoy3y = ros_output[3]
                data.buoy3_distance = ros_output[4]
            prevN, prevC, prevX, prevY, prevD = ros_output
            cv_pub.publish(data)
        except:
            print "no"
            data.buoy1 = False
            data.buoy2 = False
            data.buoy3 = False
            cv_pub.publish(data)

        if not args.dont_show:
            cv2.imshow('Inference', image)
            cv2.waitKey(fps)
        k = cv2.waitKey(10) & 0xFF
        if k == 27:
            break

    cv2.destroyAllWindows()
    cap.release()
    video.release()


if __name__ == "__main__":
    main()
