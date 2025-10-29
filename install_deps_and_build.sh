#!/bin/bash
set -e

show_help() {
    echo "$(basename "$0") [-h] [-d debians_path] -- Installs the dependencies of this project and builds it
        -h                  Show this help message
        -d debians_path     Force reinstall rr100 debians at path \$debians_path
    "
}

force_resintall=false
debians_path="debfiles"

while getopts ":h?d?:" opt; do
    case "$opt" in
        h|\?)
            show_help
            ;;
        d)
            debians_path=$OPTARG
            force_resintall=true
            ;;
        :)
            echo "Option -$OPTARG requires an argument."
            exit 1
            ;;
    esac
done

echo "Checking if rr100_* ros packages are installed..."
rospack find rr100_control 1> /dev/null 2> /dev/null
found=$?
if [[ $found -ne 0 ]] || [[ $force_resintall = true ]] ; then
    echo "Could not find rr100_control package, installing debfiles..."
    cd "$debians_path"
    sudo python deploy_debians_noetic.py .
    cd ..
fi

rosdep install --from-paths src --ignore-src --rosdistro noetic -r -y
catkin build