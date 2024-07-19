#!/usr/bin/env python3
import rospy, socket, time
from rosns3_client.msg import Waypoint
from nav_msgs.msg import Odometry
from get_map_size import Map
from get_vehicle_list import VehicleList

class GNSS():
    def __init__(self):
        self.sock : socket.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.NS3_SERVER_IP : str = '127.0.0.1'  # Remplacer par l'adresse IP du serveur NS3
        self.NS3_SERVER_PORT : int= 12345  # Remplacer par le port du serveur NS3

        self.setup()

    def send_map_size(self):
        map = Map()
        map_width, map_height = map.get_map_size()
        map.show_map_size()
        self.sock.sendto(f"MAP|{map_width}|{map_height}|".encode(), (self.NS3_SERVER_IP, self.NS3_SERVER_PORT))

    def send_vehicle_number(self):
        vehicle_list = VehicleList()
        self.num_vehicles = vehicle_list.main()
        self.sock.sendto(f"VEHICLE|{self.num_vehicles}|".encode(), (self.NS3_SERVER_IP, self.NS3_SERVER_PORT))

    def odo_to_point(self, odo_msg):
        point_msg = Waypoint()
        point_msg.position.x = odo_msg.pose.pose.position.x
        point_msg.position.y = odo_msg.pose.pose.position.y
        point_msg.position.z = odo_msg.pose.pose.position.z
        point_msg.velocity.x = odo_msg.twist.twist.linear.x
        point_msg.velocity.y = odo_msg.twist.twist.linear.y
        point_msg.velocity.z = odo_msg.twist.twist.linear.z
        point_msg.acceleration.x = odo_msg.twist.twist.angular.x
        point_msg.acceleration.y = odo_msg.twist.twist.angular.y
        point_msg.acceleration.z = odo_msg.twist.twist.angular.z
        return point_msg

    def publisher_odo(self, odo_data, vehicle_index):
        # Envoyer les données à mon topic ROS
        self.pub.publish(odo_data)
        # Envoyer les données au topic du serveur ROSNS3
        pub = rospy.Publisher(f'/robot_{vehicle_index+1}/current_state', Waypoint, queue_size=10)
        point_msg = self.odo_to_point(odo_data)
        pub.publish(point_msg)
        self.rate.sleep()
        # Envoyer les données au serveur NS3
        self.sock.sendto(f"COORDINATES|{vehicle_index}|{odo_data.pose.pose.position.x}|{odo_data.pose.pose.position.y}|{odo_data.pose.pose.position.z}|".encode(), (self.NS3_SERVER_IP, self.NS3_SERVER_PORT))

    def callback(self, data, vehicle_index):         
        print("=============================\nReceived Odometry data:")
        for attr in ['x', 'y', 'z']:
            rospy.loginfo("Position: %f", getattr(data.pose.pose.position, attr))
        # for attr in ['x', 'y', 'z']:
        #     rospy.loginfo("Velocity: %f", getattr(data.twist.twist.linear, attr))
        # for attr in ['x', 'y', 'z']:
        #     rospy.loginfo("Acceleration: %f", getattr(data.twist.twist.angular, attr))

        self.publisher_odo(data, vehicle_index)

    def listener(self, pub, rate):     # Definition de notre subscriber
        rospy.init_node('odo_publisher', anonymous=True)
        for i in range(self.num_vehicles):
            rospy.Subscriber(f"/carla/vehicle_{i+1}/odometry", Odometry, lambda data, vehicle_index=i: self.callback(data, vehicle_index))
        rospy.spin()   # Permet au programme d'être en route continuellement
    
    def initisalisation(self):
        pub = rospy.Publisher('odo_data', Odometry, queue_size=10)
        rate = rospy.Rate(10)
        return pub, rate
        
    def setup(self):
        self.send_vehicle_number()
        self.send_map_size()
        self.pub, self.rate = self.initisalisation()
        self.listener(self.pub, self.rate)

if __name__ == '__main__':
    rospy.init_node('odo_publisher', anonymous=True)
    gnss = GNSS()