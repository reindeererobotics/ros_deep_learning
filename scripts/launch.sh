#!/bin/sh

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# pass in the number of cameras you have connected to the NX default is 2 and the domain id is 3
num_cameras=2
domain_id=3

unset OPTARG OPTIND
while getopts ":n:d:" opt; do
    case $opt in
        n) num_cameras="$OPTARG"
        ;;
        d) domain_id="$OPTARG"
        ;;
        \?) echo "Invalid option -$OPTARG" >&2
        ;;
    esac
done

echo "num_cameras=$num_cameras, domain_id=$domain_id"

# if RMW_IMPLEMENTATION is not set or does not equal rmw_fastrtps_cpp, default to rmw_fastrtps_cpp
if [ -z "$RMW_IMPLEMENTATION" ] || [ "$RMW_IMPLEMENTATION" != "rmw_fastrtps_cpp" ]; then
    export RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
    echo "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
fi

export ROS_DOMAIN_ID=$domain_id

echo "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION, ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

source ~/.bashrc

# source ROS environment
source /opt/ros/${ROS_DISTRO}/install/setup.bash

# source the workspace
source ./install/local_setup.bash

# launch the node
ros2 launch ros_deep_learning dasher_nx.launch.py  num_cameras:=$num_cameras