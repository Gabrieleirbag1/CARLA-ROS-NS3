#!/bin/bash

# Vérifier si un argument est donné
if [ $# -eq 0 ]; then
    # Si aucun argument n'est donné, utiliser le répertoire courant
    source_dir="."
else
    # Utiliser le répertoire spécifié en argument
    source_dir="$1"
fi

# Vérifier si le répertoire source existe
if [ ! -d "$source_dir" ]; then
    echo "Le répertoire source '$source_dir' n'existe pas."
    exit 1
fi

# Vérifier si le répertoire de destination existe, sinon le créer
destination_dir="$HOME/Documents/CARLA-ROS/Scripts/catkin_ws/src/sub_pkg/src"
mkdir -p "$destination_dir"

# Afficher la liste des fichiers python dans le répertoire source
echo "Liste des fichiers python dans le répertoire '$source_dir':"
files=($(find "$source_dir" -type f -name "*.py"))
for i in "${!files[@]}"; do
    echo "$((i+1)) ${files[$i]}"
done

# Demander à l'utilisateur de choisir un fichier à copier
read -p "Entrez le numéro du fichier à copier (ou '*' pour copier tous les fichiers) : " choice

# Vérifier le choix de l'utilisateur
if [ "$choice" = "*" ]; then
    # Copier tous les fichiers du répertoire source vers le répertoire de destination
    cp "$source_dir"/*.py "$destination_dir"
elif [[ "$choice" =~ ^[0-9]+$ ]] && [ "$choice" -gt 0 ] && [ "$choice" -le "${#files[@]}" ]; then
    # Décrémenter choice de 1
    choice=$((choice-1))
    # Copier le fichier sélectionné vers le répertoire de destination
    cp "${files[$choice]}" "$destination_dir"
else
    echo "Choix invalide."
    exit 1
fi

echo "Copie terminée."