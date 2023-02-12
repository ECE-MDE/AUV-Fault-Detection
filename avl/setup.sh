#!/usr/bin/bash

git config --global credential.helper store
# ln -s /workspaces/AUV-Fault-Detection/avl /root/avl
wstool init src avl.rosinstall
rosdep install --from-paths . --ignore-src -y
echo "alias avl='/workspaces/AUV-Fault-Detection/avl/src/avl_tools/scripts/avl.sh'" >> ~/.bashrc
# echo "alias find='find -L'" >> ~/.bashrc
source ~/.bashrc
source /opt/ros/noetic/setup.bash
