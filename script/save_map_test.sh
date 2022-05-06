#!/bin/bash

rosservice call /write_state "{filename: '/home/$USER/test_map/map.pbstream', include_unfinished_submaps: true}"

rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=/home/$USER/test_map/map -pbstream_filename=/home/$USER/test_map/map.pbstream -resolution=0.05

