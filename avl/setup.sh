#!/bin/bash

git config --global credential.helper store
wstool init src avl.rosinstall
rosdep install --from-paths . --ignore-src -y
echo "alias avl='/workspaces/AUV-Fault-Detection/avl/src/avl_tools/scripts/avl.sh'" >> ~/.bashrc && source ~/.bashrc