#!/bin/bash

if [ "$#" -gt 0 ] && [ "$1" == "-b" ]; then
   colcon build --packages-select path_search
fi

source install/setup.bash

ros2 launch path_search tracker.launch.py

