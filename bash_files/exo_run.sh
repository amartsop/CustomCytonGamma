# Cd to the parent(catkin_ws) directory
cd $HOME/catkin_ws_cyton

# Run catkin_make from catkin_ws
catkin_make

# Source workspace setup file
source $HOME/catkin_ws_cyton/devel/setup.bash

rosrun cyton_control cyton_control_node 