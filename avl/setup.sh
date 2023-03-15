#!/usr/bin/bash

git config --global credential.helper store
wstool init src avl.rosinstall
sudo apt update
rosdep install --from-paths . --ignore-src -y
