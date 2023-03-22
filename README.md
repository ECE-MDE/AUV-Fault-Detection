# AUV Fault Detection

## Setting up the simulation
- Open VSCode
- Install the Dev Containers extension in VSCode
- Install Docker from [here](https://docs.docker.com/get-docker/) or through `F1 > Dev Containers: Install Docker`
- Open the `AUV-Fault-Detection` folder in VSCode. When prompted select "Reopen Container" popup or `F1 > Dev Containers: Rebuild and Reopen in Container`
    - VSCode will create a Docker container with ROS installed, share your local `AUV-Fault-Detection` folder with the container, then connect to the container. This allows you to edit and run code inside the container as if it was your local computer.
- ~~In the container, `cd avl` and `bash setup.sh`~~ This *should* happen automatically now.
    - VSCode will ask for your CMAR credentials, then pull the necessary git repos and their dependencies
    - The `avl` command will also be added to bash
- Build the AVL code: `catkin_make`
- Activate the catkin workspace: `source devel/setup.sh`

## Useful commands
- `source /opt/ros/noetic/setup.bash`: Makes ROS commands available in your shell
- `source avl/devel/setup.bash`: Allows ROS tools like `roslaunch` and `rosrun` to find your packages and nodes

## Logs
- Logs are stored in `/var/avl_logs/<timestamp>`
- `/var/avl_logs/current` is symlinked to latest log, so `cd /var/avl_logs/current` to access latest log
- `File > Add folder to Workspace` > `/var/avl_logs` to add the log folder to the file tree

## Running data collection
- See the top of `data_collection_launcher.py` to change trial length, when fault triggers, etc.
- Start ROS core in a new terminal window: `roscore`
- `rosrun avl_fault_detection data_collection_launcher.py`
- Stop ROS core: Ctrl+C in terminal window
- `data_collection.log` in `avl_logs` contains vehicle state data and fault labels in tabular form

## Running the simulation
Syntax: `avl start <package_name> <launch_file_name>`

To start a full system simulation with the `avl_fault_detection` package:
- `source /opt/ros/noetic/setup.bash`
- `source /workspaces/AUV-Fault-Detection/avl/devel/setup.bash`
- `avl setup avl_690`
- `avl start avl_fault_detection full_system_simulation.launch`

`F1 > ROS: Show Status` will show all current node configurations, topics, and publishers

## Updating the Workspace
- Save any work, then delete everything under `avl/`
- Re-run `setup.sh`

## Setting up Mission Control
Follow the instructions [here](https://cmar.ece.vt.edu/avl/user-interface/avl_mission_control)


## To do
[X] Run setup.sh automatically
[] Install Mission Control automatically
[] Fix fluxbox toolbar font being white
[X] Allow setup.sh to run multiple times
[] `setup.sh` line endings