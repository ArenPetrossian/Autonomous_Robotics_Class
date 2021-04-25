#!/bin/bash
#Use this to see all 5 nodes in separate terminals
gnome-terminal -- sh -c "cd ..; cd computer_vision/src/; python cv_node.py; bash"
gnome-terminal -- sh -c "cd ..; cd graphical_user_interface/src/; python gui_node.py; bash"
gnome-terminal -- sh -c "cd ..; cd guidance_navigation_control/src/; python gnc_node.py; bash"
gnome-terminal -- sh -c "cd ..; cd sensing_and_actuation/src/; python sensorActuator_node.py; bash"
gnome-terminal -- sh -c "cd ..; cd state_machine/src/; python smach_node.py; bash"
