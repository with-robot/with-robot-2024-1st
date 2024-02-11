# Install ROS2
- 참고: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
- sudo apt install software-properties-common
- sudo add-apt-repository universe
- sudo apt update && sudo apt install curl -y
- sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
- echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
- sudo apt update && sudo apt upgrade -y
- sudo apt install ros-humble-desktop -y
- sudo apt install ros-dev-tools -y
- sudo apt install ros-humble-xacro
- echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Install Gazebo
- sudo apt update && sudo apt upgrade -y
- sudo apt install -y git python3-pip
- sudo apt install -y python3-colcon-common-extensions \
ros-humble-joint-state-publisher-gui \
ros-humble-gazebo-plugins \
ros-humble-joint-state-publisher \
ros-humble-gazebo-ros

# jetauto_description
- cd ~/Workspace/ros_ws/src
- ros2 pkg create --build-type ament_cmake jetauto_description
- cd jetauto_description
- mkdir config launch meshes urdf rviz worlds

# Run
- killall -9 gzserver; colcon build; source install/setup.bash; ros2 launch jetauto_description rivz.launch.py
- killall -9 gzserver; colcon build; source install/setup.bash; ros2 launch jetauto_description gazebo.launch.py world:=src/jetauto_description/worlds/construction.sdf
- ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=jetauto_car/cmd_vel
