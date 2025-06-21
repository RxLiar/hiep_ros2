# hiep_ros2

gedit ~/.bashrc 

source /opt/ros/jazzy/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
export QT_QPA_PLATFORM=xcb
export LIBGL_ALWAYS_SOFTWARE=1
export QT_QPA_PLATFORM=xcb
source ~/hiep_ros2/install/setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/gazebo_models/gazebo_models
source ~/ros2_ws/install/setup.bash
