# How to Install this Repository

1. mkdir ~/ai_class_ws/src/  
2. cd ~/ai_class_ws  
3. catkin_make  
4. rm -rf src/  
5. *Download entire repository into ai_class_ws/*  
6. mv Artificial_Intelligence_Class-master src/  
	* *Replace first argument with the name of the folder you download*
7. catkin_make  
   
---
# How to Run and View all Nodes

1. cd ~/ai_class_ws/src/launchers/   
* **Option 1:**   
	*bash node_launcher.bash   
* **Option 2:**   
	*roslaunch node_launcher.launch   
2. rosrun rqt_graph rqt_graph   
3. *The output should look like the following:*   
![RQT_Graph](rqt_graph.png)
