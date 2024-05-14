#bridge NS-ROS
mkdir -p ~/ns-ros-bridge/catkin_ws/src/ && cd ~/ns-ros-bridge/catkin_ws/src/

git clone https://github.com/malintha/rosns3

cd ~/ns-ros-bridge/catkin_ws/ && catkin build rosns3_client

echo "source ~/ns-ros-bridge/catkin_ws/devel/setup.bash" >> ~/.bashrc