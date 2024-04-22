#!/bin/bash

# Fonction pour tuer tous les processus fils
kill_children() {
    pkill -P $$  # Tuer tous les processus fils du processus actuel
}

# Intercepter les signaux de terminaison et appeler la fonction kill_children
trap 'kill_children' SIGINT SIGTERM EXIT

# Lancer les commandes en arrière-plan
cd ~/CARLA_0.9.11 && ./CarlaUE4.sh windowed -opengl -quality-level=Low &
sleep 5 && roslaunch carla_ros_bridge carla_ros_bridge.launch timeout:=300 town:=Town03 &
sleep 10 && roslaunch carla_spawn_objects carla_spawn_objects.launch objects_definition_file:=/home/gab/CARLA-ROS_SCRIPTS/Jsons/falling_car_spawn.json &
sleep 12 && roslaunch carla_spawn_objects carla_spawn_objects.launch objects_definition_file:=/home/gab/CARLA-ROS_SCRIPTS/Jsons/car1_spawn.json &
sleep 14 && roslaunch carla_spawn_objects carla_spawn_objects.launch objects_definition_file:=/home/gab/CARLA-ROS_SCRIPTS/Jsons/car2_spawn.json &
sleep 16 && roslaunch carla_spawn_objects carla_spawn_objects.launch objects_definition_file:=/home/gab/CARLA-ROS_SCRIPTS/Jsons/car3_spawn.json &

# Attendre que tous les processus fils se terminent
wait
