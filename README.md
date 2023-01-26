# AUV Fault Detection

## Setting up the simulation
- Open VSCode
- Install the Dev Containers extension
- When prompted select "Reopen Container"
    - VSCode will create a Docker container with ROS installed, share your local `AUV-Fault-Detection` folder with the container, then connect to the container. This allows you to edit and run code inside the container as if it was your local computer.
- In the container, `cd avl` and `./setup.sh`
    - VSCode will ask for your CMAR credentials, then pull the necessary git repos and their dependencies
    - The `avl` command will also be added to bash
    