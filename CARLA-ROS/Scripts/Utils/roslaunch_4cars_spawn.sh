#!/bin/bash

open_terminal() {
    gnome-terminal -- bash -c "$1; exec bash" &
}

open_terminal "cd ~/CARLA_0.9.11 && ./CarlaUE4.sh windowed -opengl -quality-level=Low"
open_terminal "sleep 5 && roslaunch carla_ros_bridge carla_ros_bridge.launch timeout:=300 town:=Town03"
# open_terminal "sleep 10 && roslaunch carla_spawn_objects carla_spawn_objects.launch objects_definition_file:=/home/gab/CARLA-ROS_SCRIPTS/Jsons/falling_car_spawn.json"
open_terminal "sleep 12 && roslaunch carla_spawn_objects carla_spawn_objects.launch objects_definition_file:=/home/gab/Documents/Stage-2024/CARLA-ROS/Jsons/falling_car_spawn.json"
open_terminal "sleep 14 && roslaunch carla_spawn_objects carla_spawn_objects.launch objects_definition_file:=/home/gab/Documents/Stage-2024/CARLA-ROS/Jsons/car2_spawn.json"
open_terminal "sleep 16 && roslaunch carla_spawn_objects carla_spawn_objects.launch objects_definition_file:=/home/gab/Documents/Stage-2024/CARLA-ROS/Jsons/car3_spawn.json"


while true; do
    read -p "Press enter to kill ALL terminals, CTRL+C to not" enter
    pkill gnome-terminal
    exit 0
done