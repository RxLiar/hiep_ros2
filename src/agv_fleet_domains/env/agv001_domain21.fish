set -l ws $HOME/hiep_ros2
if set -q HIEP_ROS2_WS
    set ws $HIEP_ROS2_WS
end

source /opt/ros/jazzy/setup.fish
if test -f $ws/install/setup.fish
    source $ws/install/setup.fish
end

set -gx ROS_AUTOMATIC_DISCOVERY_RANGE SUBNET
set -gx RMW_IMPLEMENTATION rmw_fastrtps_cpp
set -e ROS_LOCALHOST_ONLY

set -gx ROS_DOMAIN_ID 21
set -gx AGV_ROBOT_ID AGV001

echo "[AGV001] ROS_DOMAIN_ID=$ROS_DOMAIN_ID, AGV_ROBOT_ID=$AGV_ROBOT_ID"
