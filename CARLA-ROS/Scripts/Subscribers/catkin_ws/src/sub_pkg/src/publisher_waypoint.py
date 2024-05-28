#!/usr/bin/env python3
import rospy, socket, time
from rosns3_client.msg import Waypoint
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from get_map_size import Map

class GNSS():
    def __init__(self, *args, **kwargs):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.NS3_SERVER_IP = '127.0.0.1'  # Remplacer par l'adresse IP du serveur NS3
        self.NS3_SERVER_PORT = 12345  # Remplacer par le port du serveur NS3

        self.send_map_size()
        
        self.pub, self.rate = self.initisalisation()
        self.listener(self.pub, self.rate)

    def send_map_size(self):
        map = Map()
        map_width, map_height = map.get_map_size()
        map.show_map_size()
        self.sock.sendto(str((map_width, map_height, 0.0)).encode(), (self.NS3_SERVER_IP, self.NS3_SERVER_PORT))
        time.sleep(3)

    def initisalisation(self):
        rospy.init_node('odo_publisher', anonymous=True)
        pub = rospy.Publisher('odo_data', Odometry, queue_size=10)
        rate = rospy.Rate(10)
        return pub, rate

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

    def publisher_odo(self, odo_data):
        print("=============================")
        self.pub.publish(odo_data)

        pub = rospy.Publisher('/robot_1/current_state', Waypoint, queue_size=10)
        point_msg = self.odo_to_point(odo_data)
        pub.publish(point_msg)
        self.rate.sleep()
        # Envoyer les données au serveur NS3
        self.sock.sendto(str((odo_data.pose.pose.position.x, odo_data.pose.pose.position.y, odo_data.pose.pose.position.z)).encode(), (self.NS3_SERVER_IP, self.NS3_SERVER_PORT))
        # time.sleep(3)

    def callback(self, data):         
        print("Received Odometry data:")
        for attr in ['x', 'y', 'z']:
            rospy.loginfo("Position: %f", getattr(data.pose.pose.position, attr))
        for attr in ['x', 'y', 'z']:
            rospy.loginfo("Velocity: %f", getattr(data.twist.twist.linear, attr))
        for attr in ['x', 'y', 'z']:
            rospy.loginfo("Acceleration: %f", getattr(data.twist.twist.angular, attr))

        self.publisher_odo(data)

    def listener(self, pub, rate):     # Definition de notre subscriber
        rospy.Subscriber("/carla/ego_vehicle/odometry", Odometry, self.callback)  # Nous indiquons à quelle publishers nous nous souscrivons 
        rospy.spin()    # Permet au programme d'être en route continuellement
    
if __name__ == '__main__':
    gnss = GNSS()