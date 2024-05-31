
  # CARLA ROS NS3

  Durant mon stage en 2024, j'ai travaillé sur une solution pour relier CARLA à NS3, via ROS. 
  
  Pour ce faire j'ai utilisé le [CARLA-ROS Bridge](https://github.com/carla-simulator/ros-bridge?tab=readme-ov-file) sur Ubuntu 20.04, puis j'ai relié ROS et NS3 via le module [rosns3](https://github.com/malintha/rosns3_server/tree/master) qui me permet de communiquer par des sockets UDP entre ROS et NS3.
## Documentation d'utilisation  

⚠️ **L'utilisation de Ubuntu 20.04 est vivement recommandée.**

Pour mettre en place ce projet, suivez les instructions détaillées des documentations suivantes, si possible dans l'ordre.

### Essentiel
- [Installation ROS Noetic](https://github.com/Gabrieleirbag1/CARLA-ROS-NS3/blob/main/Documents/Proc%C3%A9dures%20d'installation/Installation%20Ros%201%20Noetic%20Ubuntu.pdf)
- [Installation de CARLA-ROS Bridge](https://github.com/Gabrieleirbag1/CARLA-ROS-NS3/blob/main/Documents/Proc%C3%A9dures%20d'installation/Installation%20carla-ros-bridge%20Ubuntu%2020.04.pdf)

#### Alternativement
- [Installation de CARLA-ROS Bridge via Docker](https://github.com/Gabrieleirbag1/CARLA-ROS-NS3/blob/main/Documents/Proc%C3%A9dures%20d'installation/Installation%20de%20carla-ros-bridge%20via%20Docker.pdf)

### Complément
- [Initialisation de l'arborescence python](https://github.com/Gabrieleirbag1/CARLA-ROS-NS3/blob/main/Documents/Proc%C3%A9dures%20d'installation/Initialisation%20de%20l'arborescence%20Python%20pour%20CARLA%20et%20ROS.pdf)
- [Utilisation de CARLA avec ROS](https://github.com/Gabrieleirbag1/CARLA-ROS-NS3/blob/main/Documents/Comprendre%20et%20utiliser/Utilisation%20de%20Carla%20avec%20ROS.pdf)
- [Fonctionnement de CARLA avec ROS](https://github.com/Gabrieleirbag1/CARLA-ROS-NS3/blob/main/Documents/Comprendre%20et%20utiliser/Fonctionnement%20de%20Carla%20et%20ROS.pdf)
## Screenshots
### CARLA Simulation
![CARLA](https://github.com/Gabrieleirbag1/CARLA-ROS-NS3/blob/main/CARLA-ROS/Medias/CARLA%20Server%20%26%20Manual%20Control.png?raw=true)  

![CARLA Simulation](https://github.com/Gabrieleirbag1/CARLA-ROS-NS3/blob/main/CARLA-ROS/Medias/CARLA%20Autopilot.gif?raw=true)  

### NS3
[![Netanim 2 Nodes](https://github.com/Gabrieleirbag1/CARLA-ROS-NS3/blob/main/CARLA-ROS/Medias/Netanim.jpg?raw=true)](https://youtu.be/kYeH5b_MS20)

### Schémas
![CARLA ROS BRIDGE](https://github.com/Gabrieleirbag1/CARLA-ROS-NS3/blob/main/CARLA-ROS/Medias/Sch%C3%A9ma%20CARLA-ROS-Bridge.drawio.png?raw=true)

![ROS](https://github.com/Gabrieleirbag1/CARLA-ROS-NS3/blob/main/CARLA-ROS/Medias/Sch%C3%A9ma%20CARLA%20Sensors%20light.drawio.png?raw=true)