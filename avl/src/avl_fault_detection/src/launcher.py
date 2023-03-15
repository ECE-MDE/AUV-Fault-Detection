from pathlib import Path
import roslaunch
import time
import subprocess

SRC_PATH = "/workspaces/AUV-Fault-Detection/avl/src"

def main():
    n_trials = 10
    trial_duration_s = 30
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)

    for n_trial in range(n_trials):
        roslaunch.configure_logging(uuid)
        # launch = roslaunch.scriptapi.ROSLaunch()
        launch = roslaunch.parent.ROSLaunchParent(uuid, [f"{SRC_PATH}/avl_fault_detection/launch/data_collection.launch"], is_core=True)

        # Start another node
        # node = roslaunch.core.Node(package, executable)
        # launch.launch(node)

        print(f"Starting trial {n_trial}")
        # Create AVL log directories. This is what avl start does before running roslaunch.
        subprocess.run(f"{SRC_PATH}/avl_tools/scripts/avl.sh split".split(" "), stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True, text=True)
        # Launch .launch file
        launch.start()
        start_time = time.time()

        while time.time() - start_time < trial_duration_s:
            launch.spin_once()
        
        print(f"Ending trial {n_trial}")
        launch.shutdown()

        # For whatever reason /var/avl_logs/current isn't deleted when we shut down the simulation via the API
        # Keeping this allows the logger to append to the same log
        # Deleting this forces the logger to create a new folder for logs
        Path("/var/avl_logs/current").unlink()

if __name__ == '__main__':
    main()