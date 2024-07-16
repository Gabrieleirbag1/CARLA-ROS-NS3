#!/usr/bin/env python3
import carla

class Map():
    def __init__(self) -> None:
        pass

    def get_map_size(self):
        client = carla.Client('localhost', 2000)
        world = client.get_world()
        map = world.get_map()

        spawn_points = map.get_spawn_points()

        # Get the maximum and minimum x and y coordinates
        self.min_x = min(spawn_point.location.x for spawn_point in spawn_points)
        self.min_y = min(spawn_point.location.y for spawn_point in spawn_points)
        self.min_z = min(spawn_point.location.z for spawn_point in spawn_points)

        return self.min_x, self.min_y, self.min_z

    def show_map_size(self):
        print('Map size: x : {} y: {} z: {}'.format(self.min_x, self.min_y, self.min_z))

if __name__ == '__main__':
    map = Map()
    map.get_map_size()
    map.show_map_size()