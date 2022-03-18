# Get Started with Jetson Nano

## How to Install ROS

1. sudo apt-get -y install nano
2. sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
3. sudo apt install curl
4. curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
5. sudo apt update
6. sudo apt install ros-melodic-desktop
7. echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
8. source ~/.bashrc
9. sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
10. sudo apt install python-rosdep
11. sudo rosdep init
12. rosdep update  

&emsp;**Check if ROS is Installed:**   
&emsp;&emsp; printenv | grep ROS



---
## How to Install OpenCV

1. sudo apt-get purge \*libopencv\*
2. sudo apt update
3. sudo apt install python-opencv



---
## How to Install Darknet & YOLOv4-Tiny

1. cd ~/ai_class_ws/src/
2. git clone https://github.com/AlexeyAB/darknet.git 
3. nano Makefile
	* Set OpenCV -> 1
4. make
5. sudo apt-get install libopencv-dev
6. wget 
https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v4_pre/yolov4-tiny.weights
7. wget https://raw.githubusercontent.com/AlexeyAB/darknet/master/cfg/yolov4-tiny.cfg
8. nano Makefile
	* Set LIBSO -> 1
9. make

&emsp;**Check if YOLO is Installed:**   
&emsp;&emsp; ./darknet detector demo cfg/coco.data cfg/yolov3.cfg yolov3.weights -c 0



---
## How to Install Smach Viewer

1. sudo apt-get install python-gi-cairo
2. cd ~/ai_class_ws/src/
3. git clone https://github.com/ros-visualization/executive_smach_visualization.git



---
## How to Install Arduino & Rosserial

1. *Open Internet & Download:*
	* https://downloads.arduino.cc/arduino-1.8.19-linuxaarch64.tar.xz
2. sudo bash install.sh
3. *In Arduino IDE Library Manager Install:*
	* "PID" by Brett Beauregard
	* "MS5837" by Blue Robotics
	* "BNO055" by Adafruit **+ Sub-Libraries**



---
## How to Install this Repository

1. mkdir ~/ai_class_ws/src/  
2. cd ~/ai_class_ws  
3. catkin_make  
4. rm -rf src/  
5. git clone https://github.com/ArenPetrossian/Autonomous_Robotics_Class.git
6. mv Artificial_Intelligence_Class-master src/  
	* *Replace first argument with the name of the folder you download*
7. catkin_make  



---
## How to Run and View all Nodes

1. cd ~/ai_class_ws/src/launchers/   
	**Option 1:**   
	* bash node_launcher.bash   

	**Option 2:**   
	* roslaunch node_launcher.launch

2. rosrun rqt_graph rqt_graph   

*The output should look like the following:*   

![RQT_Graph](rqt_graph.png)