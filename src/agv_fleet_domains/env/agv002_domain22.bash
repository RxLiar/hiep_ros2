# Source this file in every terminal before starting the corresponding process.
_WS="${HIEP_ROS2_WS:-$HOME/hiep_ros2}"

source /opt/ros/jazzy/setup.bash
if [ -f "$_WS/install/setup.bash" ]; then
    source "$_WS/install/setup.bash"
fi

export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_LOCALHOST_ONLY

export ROS_DOMAIN_ID=22
export AGV_ROBOT_ID=AGV002

echo "[AGV002] ROS_DOMAIN_ID=$ROS_DOMAIN_ID, AGV_ROBOT_ID=$AGV_ROBOT_ID"
