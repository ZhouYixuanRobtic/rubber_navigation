#!/bin/bash

rosservice call /write_state "{filename: '/home/$USER/catkin_ws/src/rubber_tapping_robot/rubber_navigation/config/map/map.pbstream', include_unfinished_submaps: true}"

rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=/home/$USER/catkin_ws/src/rubber_tapping_robot/rubber_navigation/config/map/map -pbstream_filename=/home/$USER/catkin_ws/src/rubber_tapping_robot/rubber_navigation/config/map/map.pbstream -resolution=0.05

