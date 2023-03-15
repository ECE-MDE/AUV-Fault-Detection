#!/bin/bash
#===============================================================================
# Autonomous Vehicle Library
#
# Description: This bash script handles various commands for setup and
#              start/stop of AVL. To most easily use this script, add an alias
#              into your bashrc file with:
#
# echo "alias avl='~/avl/src/avl_tools/scripts/avl.sh'" >> ~/.bashrc && source ~/.bashrc
#
##             then simply call the script with :
#                  avl [command] [arguments]
#===============================================================================

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Settings
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# Directory paths
AVL_DIR="$HOME/avl"
CONFIG_DIR="/var/avl_config"
CAL_DIR="/var/avl_calibration"
LOG_DIR="/var/avl_logs"

# Launch file package and filename for start and restart
DEFAULT_LAUNCH_PACKAGE="avl_simulation"
DEFAULT_LAUNCH_FILE="full_system_simulation.launch"

# Color escape characters
RED='\033[0;31m'
CYAN='\033[0;36m'
GOLD='\033[0;33m'
NC='\033[0m'

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# AVL help command
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
cmd_help()
{
    echo -e "Usage: avl <subcommand> [options]"
    echo -e "Subcommands:"
    echo -e "    ${CYAN}setup    ${NC} Copies config files from the AVL repository to the AVL config directory (${CONFIG_DIR})."
    echo -e "    ${CYAN}start    ${NC} Creates a new log folder and starts AVL in the foreground."
    echo -e "    ${CYAN}stop     ${NC} Stops AVL if it is currently running."
    echo -e "    ${CYAN}restart  ${NC} Stops AVL and starts it again in the foreground."
    echo -e "    ${CYAN}view     ${NC} Views the AVL log output in real time. Used when AVL is run through a systemd service."
    echo -e "    ${CYAN}status   ${NC} Views the git status of every AVL reposiory in the AVL directory (${AVL_DIR})."
    echo -e "    ${CYAN}compress ${NC} Checks the given directory and its subdirectories for AVL log folders and zips them."
    echo -e "    ${CYAN}erase    ${NC} Permanently deletes the AVL log folder (${LOG_DIR})."
    echo -e "    ${CYAN}shutdown ${NC} Shuts down AVL and the computer. Should be done before switching power off."
    echo -e ""
}

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# AVL setup command
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
cmd_setup()
{

    echo "Running AVL setup..."

    # Check to see if /var/avl_config directory exists. If not, make it
    if !  [[ -d ${CONFIG_DIR} ]] ; then
        echo "    creating new ${CONFIG_DIR} directory"
        sudo mkdir ${CONFIG_DIR}
        sudo chown -R "${USER}:${USER}" ${CONFIG_DIR}
        echo "    created ${CONFIG_DIR} directory"
    else
        echo "    ${CONFIG_DIR} directory already exists, removing"
        sudo rm -r ${CONFIG_DIR}
        echo "    creating new ${CONFIG_DIR} directory"
        sudo mkdir ${CONFIG_DIR}
        sudo chown -R "${USER}:${USER}" ${CONFIG_DIR}
        echo "    created ${CONFIG_DIR} directory"
    fi

    # Check to see if /var/avl_calibration directory exists. If not, make it
    if !  [[ -d ${CAL_DIR} ]] ; then
        echo "    creating new ${CAL_DIR} directory"
        sudo mkdir ${CAL_DIR}
        sudo chown -R "${USER}:${USER}" ${CAL_DIR}
        echo "c    reated ${CAL_DIR} directory"
    else
        echo "    ${CAL_DIR} directory already exists"
    fi

    # The find command finds all files with the .config extension in the
    # AVL directory and its subdirectories
    #   -type f sets the search type to files
    #   -iname *.config finds all files with the .config extension
    #   -print0 prints a null character instead of the default newline
    # The xargs takes the output of the pipe and passes the results to the
    # cp command
    #   -0 indicates items are null terminated
    #   -n 1 specifies one argument per command line
    #   -Ito replaces "to" with the items from the find results
    echo "    copying config files from ${AVL_DIR} to ${CONFIG_DIR}"
    find ${AVL_DIR} -type f -iname *.config -print0 | xargs -0 -n 1 -Ito cp to ${CONFIG_DIR}
    echo "    copied config files from ${AVL_DIR} to ${CONFIG_DIR}"

    echo "AVL setup complete!"
    echo ""

}

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# AVL start command
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
cmd_start()
{

    echo "Starting AVL..."

    # Check to see if roslaunch is already running
    if (ps -e | grep roslaunch); then
        echo "Roslaunch process found running. Try restart instead."
        exit 1
    fi

    # Create a new log folder
    cmd_split

    # Source in case the user hasn't already
    source "${AVL_DIR}/devel/setup.bash"

    # Launch the ROS launch file with roslaunch. If there are arguments, pass
    # them to roslaunch. Otherwise, use the default package and launch file
    echo "starting roslaunch"
    if [[ "$#" -eq 2 ]]; then
        roslaunch $1 $2
    else
        roslaunch ${DEFAULT_LAUNCH_PACKAGE} ${DEFAULT_LAUNCH_FILE}
    fi

    echo "AVL execution finished!"
    echo ""

}

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# AVL stop command
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
cmd_stop()
{

    echo "Stopping AVL..."

    ROSLAUNCH_PID=$(ps -e | pgrep roslaunch)

    if [[ -z ${ROSLAUNCH_PID} ]]; then
          echo "    no roslaunch process to kill"
    else

          echo "    killing roslaunch process"
          kill ${ROSLAUNCH_PID}

          echo "    waiting for ROS to shut down..."
          while [ -e /proc/${ROSLAUNCH_PID} ]; do sleep 1; done
          echo "AVL stopped!"

    fi

    echo ""

}

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# AVL restart command
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
cmd_restart()
{
    echo "Restarting AVL..."
    cmd_stop
    cmd_start
    echo "AVL restarted!"
    echo ""
}

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# AVL view command
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
cmd_view()
{

    # Check to see if /var/avl_logs/avl.log file exists
    if [ -f "${LOG_DIR}/avl.log" ]
    then
        tail -f "${LOG_DIR}/avl.log"
    else
        echo "No log file to view, is AVL running?"
    fi

}

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# AVL status command
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
cmd_status()
{

    echo -e "${CYAN}===================================================================================${NC}"

    # Run the find command to find all directories and run git status in them
    find "${AVL_DIR}/src" \
        -maxdepth 1 \
        -mindepth 1 \
        -type d \
        -exec sh -c '(echo "'${GOLD}'{}'${NC}'" &&
                      cd {} &&
                      git -c color.status=always status &&
                      echo &&
                      echo "'${CYAN}'==================================================================================='${NC}'")' \;

}

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# AVL compress command
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
cmd_compress()
{

    # Requires a path to a parent directory
    if [[ "$#" -eq 1 ]]; then

        # CD into the parent directory
        cd "${1}"

        # Loop through all directories in the parent directory and check if they
        # contain a "log" directory. Any directory that contains a "log"
        # directory is an AVL log directory to be zipped
        find . -maxdepth 1 -type d |
        while read DIR; do
            if [[ -d "${DIR}/log" ]]; then
                zip -r "${DIR}.zip" "${DIR}"
            fi
        done

    else
        echo "usage: avl compress [path]"
    fi

}

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# AVL erase command
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
cmd_erase()
{

    echo -e "${GOLD}!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!${NC}"
    echo -e "${GOLD}WARNING: THIS WILL PERMANENTLY DELETE ALL OF THE FOLLOWING LOG FOLDERS FROM ${LOG_DIR}:${NC}"
    echo -e "${GOLD}!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!${NC}"
    ls ${LOG_DIR}
    echo -e "${GOLD}!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!${NC}"

    while true; do
        echo -e "WARNING: THIS WILL ${RED}PERMANENTLY DELETE${NC} ALL AVL LOG FOLDERS FROM (${LOG_DIR})!"
        read -p "Are you sure? (y/n): " yn
        case $yn in
            [Yy]* )
                while true; do
                    echo -e "WARNING: No really, these ${RED}CANNOT BE RECOVERED${NC} if deleted!"
                    read -p "Delete all log folders? (y/n): " yn
                    case $yn in
                        [Yy]* ) sudo rm -r "${LOG_DIR}/"*; echo "AVL log files deleted!"; exit;;
                        [Nn]* ) exit;;
                        * ) echo "Please answer y or n.";;
                    esac
                done;;
            [Nn]* ) exit;;
            * ) echo "Please answer y or n.";;
        esac
    done

}

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# AVL shutdown command
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
cmd_shutdown()
{
    echo "Shutting down AVL control computer..."
    cmd_stop
    sudo shutdown now
}

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# AVL split command
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
cmd_split()
{

    echo "Creating new AVL log folder..."

    # Check to see if the AVL log directory exists. If not, make it
    if !  [[ -d ${LOG_DIR} ]] ; then
        echo "    creating ${LOG_DIR} directory"
        sudo mkdir ${LOG_DIR}
        sudo chown -R "${USER}:${USER}" ${LOG_DIR}
        echo "    created ${LOG_DIR} directory"
    else
        echo "    ${LOG_DIR} already directory exists"
    fi

    # Grab the current date and time formatted as
    #     YYYY-MM-DD_HH.MM.SS
    # and create a new log folder in the AVL log directory whose name is the
    # data and time. In the log folder, create the log and config folders
    echo "    creating new log folder"
    DATEANDTIME=`date +%F_%H.%M.%S`
    mkdir "${LOG_DIR}/${DATEANDTIME}"
    mkdir "${LOG_DIR}/${DATEANDTIME}/log"
    mkdir "${LOG_DIR}/${DATEANDTIME}/config"
    echo "    created ${LOG_DIR}/${DATEANDTIME}"
    echo "    created ${LOG_DIR}/${DATEANDTIME}/log"
    echo "    created ${LOG_DIR}/${DATEANDTIME}/config"

    # Create a symbolic link that points from current to the timestamped log
    # folder that we just created.
    #    -s creates a symbolic link
    #    -f removes the existing destination if it exists
    #    -n treats the destination as a normal file instead of a link
    ln -sfn "${LOG_DIR}/${DATEANDTIME}" "${LOG_DIR}/current"
    echo "    created symbolic link (${LOG_DIR}/current -> ${LOG_DIR}/${DATEANDTIME})"

    # Copy all config files into the avl_logs
    echo "    copying config files from ${CONFIG_DIR} to ${LOG_DIR}/${DATEANDTIME}/config"
    cp -a "${CONFIG_DIR}/." "${LOG_DIR}/current/config"
    echo "    copied config files from ${CONFIG_DIR} to ${LOG_DIR}/${DATEANDTIME}/config"

    echo "New AVL log folder created!"

}

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Main path
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
subcommand=$1
case $subcommand in
    "" | "-h" | "--help")
        cmd_help
        ;;
    *)
        shift
        cmd_${subcommand} "${@}"
        if [ $? = 127 ]; then
            echo "Error: '$subcommand' is not a known subcommand." >&2
            echo "       Run '$script_name --help' for a list of known subcommands." >&2
            exit 1
        fi
        ;;
esac
