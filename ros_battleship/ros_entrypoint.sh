#!/bin/bash
set -e

# setup ros environment
source "/root/catkin_ws/devel/setup.bash"

# add launch command here
roslaunch ros_battleship start_game.launch

exec "$@"