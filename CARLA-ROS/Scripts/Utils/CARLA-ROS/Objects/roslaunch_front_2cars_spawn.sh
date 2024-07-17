#!/bin/bash
open_terminal() {
    gnome-terminal -- bash -c "$1; exec bash" &
}

user=$(whoami)
open_terminal "cd ~/CARLA_0.9.11 && ./CarlaUE4.sh windowed -opengl -quality-level=Low"
open_terminal "sleep 5 && roslaunch carla_ros_bridge carla_ros_bridge.launch timeout:=300 town:=Town03"
open_terminal "sleep 7 && roslaunch carla_spawn_objects carla_spawn_objects.launch objects_definition_file:=/home/$USER/Documents/Stage-2024/CARLA-ROS/Configs/front_car_1.json"
open_terminal "sleep 7 && roslaunch carla_spawn_objects carla_spawn_objects.launch objects_definition_file:=/home/$USER/Documents/Stage-2024/CARLA-ROS/Configs/front_car_2.json"
open_terminal "sleep 7 && roslaunch carla_spawn_objects carla_spawn_objects.launch objects_definition_file:=/home/$USER/Documents/Stage-2024/CARLA-ROS/Configs/car2_spawn.json"
# open_terminal "sleep 8 && rostopic pub /carla/ego_vehicle/vehicle_control_cmd carla_msgs/CarlaEgoVehicleControl '{throttle: 1.0, steer: 0.0}' -r 10"

cleanup() {
    kill $(ps aux | grep "/home/$user/CARLA_0.9.11/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping CarlaUE4 windowed -opengl -quality-level=Low" | awk 'NR==1{print $2}')
}

trap cleanup EXIT

while true; do
    read -p "Press enter to kill ALL terminals, CTRL+C to not" enter
    pkill gnome-terminal
    exit 0
done