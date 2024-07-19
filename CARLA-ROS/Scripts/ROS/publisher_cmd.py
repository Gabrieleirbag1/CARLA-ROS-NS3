#!/usr/bin/env python3

import rospy
from carla_msgs.msg import CarlaEgoVehicleControl

def publisher_cmd():
    rospy.init_node('publisher_cmd', anonymous=True)
    pub = rospy.Publisher('/carla/ego_vehicle/vehicle_control_cmd', CarlaEgoVehicleControl, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        control_cmd = CarlaEgoVehicleControl()
        control_cmd.throttle = 1.0
        control_cmd.steer = 0.0  # Set steer to 0.0 for straight movement
        pub.publish(control_cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher_cmd()
    except rospy.ROSInterruptException:
        pass
