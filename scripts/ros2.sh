#!/bin/bash

cd ~/code || exit 1
pixi run -e ros bash -c "
    source ~/code/.pixi/envs/ros/local_setup.bash &&
    source ~/code/ros2_ws/install/setup.sh &&
    source ~/code/ros2_ws/install/local_setup.sh &&
    ros2 \"\$@\"
" bash "$@"

