
  # CARLA ROS NS3

  Durant mon stage en 2024, j'ai travaillé sur une solution pour relier CARLA Simulator à NS3, via ROS. 
  
  Pour ce faire j'ai utilisé le [CARLA-ROS Bridge](https://github.com/carla-simulator/ros-bridge?tab=readme-ov-file) sur Ubuntu 20.04, puis j'ai relié ROS et NS3 via le module [rosns3](https://github.com/malintha/rosns3_server/tree/master) qui me permet de communiquer par des sockets UDP entre ROS et NS3. Enfin j'ai mis en place des échanges réseaux entre les noeuds dans NS3 qui correspondants aux véhicules dans CARLA Simulator, et j'ai ajouté l'affichage de ces échanges directement dans CARLA Simulator.
  
## Documentation d'utilisation  

⚠️ **L'utilisation de Ubuntu 20.04 est vivement recommandée.**

Pour mettre en place ce projet, suivez les instructions détaillées des documentations suivantes, si possible dans l'ordre.

### Essentiel
- [Installation ROS Noetic](https://github.com/Gabrieleirbag1/CARLA-ROS-NS3/blob/main/Documents/Proc%C3%A9dures%20d'installation/Installation%20Ros%201%20Noetic%20Ubuntu.pdf)
- [Installation de CARLA-ROS Bridge](https://github.com/Gabrieleirbag1/CARLA-ROS-NS3/blob/main/Documents/Proc%C3%A9dures%20d'installation/Installation%20carla-ros-bridge%20Ubuntu%2020.04.pdf)
- [Installation de NS3](https://github.com/Gabrieleirbag1/CARLA-ROS-NS3/blob/main/Documents/Proc%C3%A9dures%20d'installation/Installation%20NS3%20pour%20CARLA%20et%20ROS.pdf).

#### Alternativement
- [Installation de CARLA-ROS Bridge via Docker](https://github.com/Gabrieleirbag1/CARLA-ROS-NS3/blob/main/Documents/Proc%C3%A9dures%20d'installation/Installation%20de%20carla-ros-bridge%20via%20Docker.pdf)

### Complément
- [Initialisation de l'arborescence python](https://github.com/Gabrieleirbag1/CARLA-ROS-NS3/blob/main/Documents/Proc%C3%A9dures%20d'installation/Initialisation%20de%20l'arborescence%20Python%20pour%20CARLA%20et%20ROS.pdf)
- [Utilisation de CARLA avec ROS](https://github.com/Gabrieleirbag1/CARLA-ROS-NS3/blob/main/Documents/Comprendre%20et%20utiliser/Utilisation%20de%20Carla%20avec%20ROS.pdf)
- [Fonctionnement de CARLA avec ROS](https://github.com/Gabrieleirbag1/CARLA-ROS-NS3/blob/main/Documents/Comprendre%20et%20utiliser/Fonctionnement%20de%20Carla%20et%20ROS.pdf)
- [Utilisation de NS3 avec ROS & CARLA Simulator](https://github.com/Gabrieleirbag1/CARLA-ROS-NS3/blob/main/Documents/Comprendre%20et%20utiliser/Utilisation%20de%20NS3%20avec%20ROS%20et%20CARLA%20Simulator.pdf)

## Matériel de démonstration
### CARLA Simulator
![CARLA](https://github.com/Gabrieleirbag1/CARLA-ROS-NS3/blob/main/CARLA-ROS/Medias/CARLA%20Server%20%26%20Manual%20Control.png?raw=true)

![Simulation from CARLA](https://github.com/Gabrieleirbag1/CARLA-ROS-NS3/blob/main/CARLA-ROS/Medias/CARLA%20Autopilot.gif?raw=true)

### NS3 & Netanim (Vidéos)
[![CARLA-ROS & NS3 Netanim - Waypoint Mobilty Model 2 Nodes ](https://github.com/Gabrieleirbag1/CARLA-ROS-NS3/blob/main/CARLA-ROS/Medias/NS3%20Netanim.png?raw=true)](https://youtu.be/OJwKWWmvIbc)

[![CARLA Simulator & NS3 - Adhoc Wi fi Network 4 Nodes Broadcast](https://github.com/Gabrieleirbag1/CARLA-ROS-NS3/blob/main/CARLA-ROS/Medias/NS3%20Broadcast.png?raw=true)](https://youtu.be/1ZguOScXNjs)

[![CARLA Simulator & NS3 - Adhoc Wi fi Network 4 Nodes Unicast](https://github.com/Gabrieleirbag1/CARLA-ROS-NS3/blob/main/CARLA-ROS/Medias/NS3%20Unicast.png?raw=true)](https://youtu.be/O8JQI8dgWJ0)

[![CARLA Simulator & NS3 - NS3 communication with ROS and CARLA Simulator](https://github.com/Gabrieleirbag1/CARLA-ROS-NS3/blob/main/CARLA-ROS/Medias/NS3%20Tutorial.png?raw=true)](https://youtu.be/O-cOLJThNY4)

### Schémas
![CARLA ROS BRIDGE](https://github.com/Gabrieleirbag1/CARLA-ROS-NS3/blob/main/CARLA-ROS/Medias/Sch%C3%A9ma%20CARLA-ROS-Bridge.drawio.png?raw=true)

![ROS](https://github.com/Gabrieleirbag1/CARLA-ROS-NS3/blob/main/CARLA-ROS/Medias/Sch%C3%A9ma%20CARLA%20Sensors%20light.drawio.png?raw=true)

![NS3 Wi-Fi & ROS](https://github.com/Gabrieleirbag1/CARLA-ROS-NS3/blob/main/CARLA-ROS/Medias/Sch%C3%A9ma%20NS3%20WI-FI%20avec%20ROS%20%26%20CARLA.drawio.png)

## Author
- [@Missclick](https://www.github.com/Gabrieleirbag1) (Developer) - E-mail : gabrielgarrone670@gmail.com
