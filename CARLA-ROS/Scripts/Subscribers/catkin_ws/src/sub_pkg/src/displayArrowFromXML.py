import glob
import os
import sys
import os.path
import rospy
import random
from nav_msgs.msg import Odometry
from os import path

# We are trying to get access to the Carla API
try:
    sys.path.append(glob.glob('~/CARLA_0.9.11/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

from carla import VehicleLightState as vls

import argparse

class DisplayArrow:
    def __init__(self):
        self.ELEVATION = 1.2
        self.ARROW_SIZE = 0.6
        self.END_ARROW_SIZE = 1.1
        self.STEP = 0.15
        self.ARROW_COLOR = carla.Color(255, 0, 0)

    def get_vehicles_carla(self, world):
        vehicles = []
        id_vehicles = []

        blueprint_library = world.get_blueprint_library()
        bp = random.choice(blueprint_library.filter('vehicle'))
        transform = random.choice(world.get_map().get_spawn_points())

        vehicles_id = world.get_vehicles_light_states()
        print("Nombre de véhicules : " + str(len(vehicles_id)))
        for key, value in vehicles_id.items():
            id_vehicles.append(key)
        id_vehicles.sort()

        for id_vehicle in id_vehicles:
            vehicles.append(world.get_actor(id_vehicle))
        return vehicles

    def location_vehicule_by_id(self, vehicles, id):
        if id != 0:
            loc = vehicles[id-1].get_location()
            loc.z = loc.z + self.ELEVATION
        else:
            loc = carla.Location(0, 0, 0)
        return loc

    def get_id_from_line(self, line):
        id = line.split("\"")[3]
        return int(id)

    def display_arrow(self, helper_debug, from_loc, to_loc):
        helper_debug.draw_arrow(from_loc, to_loc, self.ARROW_SIZE, self.END_ARROW_SIZE, self.ARROW_COLOR, self.STEP)

    def main(self):
        argparser = argparse.ArgumentParser(description=__doc__)
        argparser.add_argument('--host', metavar='H', default='127.0.0.1', help='IP of the host server (default: 127.0.0.1)')
        argparser.add_argument('-p', '--port', metavar='P', default=2000, type=int, help='TCP port to listen to (default: 2000)')
        argparser.add_argument('-f', '--file' , metavar='F', default="/home/gab/ns-allinone-3.30.1/ns-3.30.1/adhoc-wifi-realtime-animation.xml", help='Path to the XML file (default: /home/gab/ns-allinone-3.30.1/ns-3.30.1/adhoc-wifi-realtime-animation.xml)')
        args = argparser.parse_args()

        client = carla.Client(args.host, args.port)
        world = client.get_world()
        helper_debug = world.debug
        
        spectator = world.get_spectator()
        # Positionnement de la caméra pour la mobilité aléatoire (dézoom max)
        pov = carla.Transform(carla.Location(x=40, y=0, z=320), carla.Rotation(yaw=180, pitch=-90))
        # Positionnement de la caméra pour la mobilité fixe (focus sur le haut de la scène)
        # pov = carla.Transform(carla.Location(x=-40, y=0, z=190), carla.Rotation(yaw=180, pitch=-90))
        spectator.set_transform(pov)

        vehicles = self.get_vehicles_carla(world)
        if len(vehicles) == 0:
            print("No vehicles found in the Carla simulation")
            return 0

        file_path = args.file
        if path.exists(file_path):
            file = open(file_path, mode='r')
        else:
            print("No Ns3 XML file : " + file_path)
            return 0

        last_position = file.tell()
        line = file.readline()
        from_id = 0
        from_loc = self.location_vehicule_by_id(vehicles, from_id)

        while line[0:7] != "</anim>":
            if line == "" or line[-1] != '\n':
                file.seek(last_position)
            else:
                if line[0:4] == "<pr ":
                    from_id = self.get_id_from_line(line)
                    from_loc = self.location_vehicule_by_id(vehicles, from_id)
                if line[0:5] == "<wpr ":
                    to_id = self.get_id_from_line(line)
                    to_loc = self.location_vehicule_by_id(vehicles, to_id)
                    self.display_arrow(helper_debug, from_loc, to_loc)

            last_position = file.tell()
            line = file.readline()

        file.close()

        print("File XML finish")

if __name__ == "__main__":
    display_arrow = DisplayArrow()
    display_arrow.main()


    
