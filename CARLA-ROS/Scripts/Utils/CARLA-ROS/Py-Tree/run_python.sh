#!/bin/bash

# Renommer les scripts pour enlever les espaces
cd ~/Documents/CARLA-ROS/Scripts/catkin_ws/src/sub_pkg/src/
for f in *\ *; do mv "$f" "${f// /_}"; done

# Récupérer la liste des scripts
result=$(ls ~/Documents/CARLA-ROS/Scripts/catkin_ws/src/sub_pkg/src/)
list=($result)

# Afficher la liste des scripts
for ((i=0; i<${#list[@]}; i++)); do
    echo "$((i+1)). ${list[$i]}"
done

# Demander à l'utilisateur de choisir un script
selected_file=""
while [[ -z $selected_file ]]; do
    read -p "Enter the number of the file you want to run: " selected_index
    if [[ $selected_index =~ ^[0-9]+$ ]] && (( selected_index > 0 && selected_index <= ${#list[@]} )); then
        selected_index=$((selected_index-1)) # décrémenter selected_index de 1
        selected_file="${list[$selected_index]}"
    else
        echo "Invalid input. Please enter a valid number."
    fi
done

# Lancer le script sélectionné
rosrun sub_pkg "$selected_file"
