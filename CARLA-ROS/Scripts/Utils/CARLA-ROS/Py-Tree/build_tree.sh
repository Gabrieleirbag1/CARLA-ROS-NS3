#!/bin/bash

# Création d'un répertoire pour notre workspace et nos scripts
mkdir -p ~/Documents/CARLA-ROS/Scripts/catkin_ws/src

# Installation des dépendances nécessaires
sudo apt install ros-noetic-catkin python3-catkin-tools ros-noetic-rosbash

# Création du premier package
cd ~/Documents/CARLA-ROS/Scripts/catkin_ws && catkin build

# Configuration de l'environnement du package
echo source ~/Documents/CARLA-ROS/Scripts/catkin_ws/devel/setup.bash >> ~/.bashrc

# Construction du package avec les dépendances
cd src/ && catkin_create_pkg sub_pkg rospy std_msgs sensor_msgs

# Construction du sous-package avec les scripts
cd sub_pkg/ && catkin build

# Donner les droits d'exécution aux scripts
cd src/ && chmod +x *
