source /opt/ros/jazzy/setup.bash
source "$HOME/hiep_ros2/install/setup.bash"

export ROS_DOMAIN_ID=25
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

unset ROS_LOCALHOST_ONLY
unset RMW_IMPLEMENTATION
export AGV_ROBOT_ID=AGV001
