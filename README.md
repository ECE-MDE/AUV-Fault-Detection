# AUV Fault Detection

## Setting up the simulation
- Open VSCode
- Install the Dev Containers extension
- Install Docker from [here](https://docs.docker.com/get-docker/) or through `F1 > Dev Containers: Install Docker`
- When prompted select "Reopen Container" popup or `F1 > Dev Containers: Rebuild and Reopen in Container`
    - VSCode will create a Docker container with ROS installed, share your local `AUV-Fault-Detection` folder with the container, then connect to the container. This allows you to edit and run code inside the container as if it was your local computer.
- In the container, `cd avl` and `./setup.sh`
    - VSCode will ask for your CMAR credentials, then pull the necessary git repos and their dependencies
    - The `avl` command will also be added to bash
- Build the AVL code: `catkin_make`
- Activate the catkin workspace: `source devel/setup.sh`

## Setting up Mission Control
Follow the instructions [here](https://cmar.ece.vt.edu/avl/user-interface/avl_mission_control)