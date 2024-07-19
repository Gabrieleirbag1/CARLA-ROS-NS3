#!/usr/bin/env python3
import rospy
from carla_msgs.msg import CarlaActorList

class VehicleList:
    def __init__(self) -> None:
        self.num_vehicles = 0

    def listener(self):
        msg = rospy.wait_for_message("/carla/actor_list", CarlaActorList, timeout=None)
        self.num_vehicles = sum(1 for actor in msg.actors if actor.type.startswith('vehicle.'))
        rospy.loginfo("Number of vehicles: %d", self.num_vehicles)

    def main(self):
        self.listener()
        return self.num_vehicles

if __name__ == '__main__':
    rospy.init_node('get_vehicle_list', anonymous=True)
    vehicle_list = VehicleList()
    vehicle_list.main()