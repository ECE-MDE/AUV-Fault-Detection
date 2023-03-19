#!/usr/bin/bash

git config --global credential.helper store
wstool init src avl.rosinstall
sudo apt update
sudo apt install -y python3-pip
pip install roslibpy
rosdep install --from-paths . --ignore-src -y
