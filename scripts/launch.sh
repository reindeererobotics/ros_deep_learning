#!/bin/sh
cd /jetson-inference/ros/ 
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# pass in the number of cameras you have connected to the NX default is 2 and the domain id is 3
num_cameras=2
domain_id=3
all=false

cam0=0
cam1=1
cam2=2
cam3=3
cam4=4
cam5=5

unset OPTARG OPTIND
while getopts ":n:d:a" opt; do
    case $opt in
        a) 
            all=true
        ;;
        n) 
            IFS=',' read -ra num_cameras <<< "$OPTARG"
        ;;
        d) domain_id="$OPTARG"
        ;;
        \?) echo "Invalid option -$OPTARG" >&2
        ;;
    esac
done

echo "num_cameras=$num_cameras, domain_id=$domain_id"

# if RMW_IMPLEMENTATION does not equal rmw_fastrtps_cpp set it to rmw_fastrtps_cpp
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

if [ "$all" = true ]; then
    echo "Launching all cameras"
    ros2 launch ros_deep_learning dasher_nx.launch.py  cam0:=0 cam1:=1 cam2:=2 cam3:=3 cam4:=4 cam5:=5
fi

# for cam in "${num_cameras[@]}"
# do
#     echo "$cam"
#     if [ "$cam" -eq -1 ]; then
#         echo "Launching 1 camera"
#         ros2 launch ros_deep_learning dasher_nx.launch.py  cam0:=$cam0
#         ros2 launch ros_deep_learning dasher_nx.launch.py  cam0:=3 cam1:=2
#         exit 0
#     fi
# done



# launch the node
# ros2 launch ros_deep_learning dasher_nx.launch.py  num_cameras:=$num_cameras