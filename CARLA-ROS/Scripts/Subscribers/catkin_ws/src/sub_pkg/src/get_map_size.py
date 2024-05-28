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
        max_x = max(spawn_point.location.x for spawn_point in spawn_points)
        min_x = min(spawn_point.location.x for spawn_point in spawn_points)
        max_y = max(spawn_point.location.y for spawn_point in spawn_points)
        min_y = min(spawn_point.location.y for spawn_point in spawn_points)

        # Calculate the size of the map
        self.map_width = max_x - min_x
        self.map_height = max_y - min_y

        return self.map_width, self.map_height

    def show_map_size(self):
        print('Map size: {} x {}'.format(self.map_width, self.map_height))

if __name__ == '__main__':
    map = Map()
    map.get_map_size()
    map.show_map_size()