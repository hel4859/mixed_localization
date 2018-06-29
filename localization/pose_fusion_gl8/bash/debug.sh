#!/bin/bash
gnome-terminal "comm" -x bash -c "rosrun communication rec_and_send_gps"
gnome-terminal "comm1" -x bash -c "rosrun communication communicate_to_cao 192.168.0.154 10004"
gnome-terminal "debug" -x bash -c "roslaunch pose_fusion debug.launch"
