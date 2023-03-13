# AUV Fault Detection

## Setting up the simulation
- Open VSCode
- Install the Dev Containers extension in VSCode
- Install Docker from [here](https://docs.docker.com/get-docker/) or through `F1 > Dev Containers: Install Docker`
- Open the `AUV-Fault-Detection` folder in VSCode. When prompted select "Reopen Container" popup or `F1 > Dev Containers: Rebuild and Reopen in Container`
    - VSCode will create a Docker container with ROS installed, share your local `AUV-Fault-Detection` folder with the container, then connect to the container. This allows you to edit and run code inside the container as if it was your local computer.
- In the container, `cd avl` and `bash setup.sh`
    - VSCode will ask for your CMAR credentials, then pull the necessary git repos and their dependencies
    - The `avl` command will also be added to bash
- Build the AVL code: `catkin_make`
- Reload `.bashrc`: `source ~/.bashrc`
- Activate the catkin workspace: `source devel/setup.sh`

## Running the simulation
Syntax: `avl start <package_name> <launch_file_name>`

To start a full system simulation with the `avl_fault_detection` package:
- `source /opt/ros/noetic/setup.bash`
- `source /workspaces/AUV-Fault-Detection/avl/devel/setup.bash`
- `avl setup avl_690`
- `avl start avl_fault_detection full_system_simulation.launch`

`F1 > ROS: Show Status` will show all current node configurations, topics, and publishers

## Setting up Mission Control
Follow the instructions [here](https://cmar.ece.vt.edu/avl/user-interface/avl_mission_control)

## To do
[] Run setup.sh automatically
[] Install Mission Control automatically
[] Fix fluxbox toolbar font being white
[] Allow setup.sh to run multiple times
[] `setup.sh` line endings