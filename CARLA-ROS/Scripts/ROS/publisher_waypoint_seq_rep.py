#!/usr/bin/env python3
import rospy, socket, time, threading, os
from math import sqrt
from rosns3_client.msg import Waypoint
from nav_msgs.msg import Odometry
from get_map_size import Map
from get_vehicle_list import VehicleList
from displayArrowFromXML import DisplayArrow

class GNSS():
    def __init__(self):
        self.sock : socket.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.NS3_SERVER_IP : str = '127.0.0.1'  # Remplacer par l'adresse IP du serveur NS3
        self.NS3_SERVER_PORT : int = 12345  # Remplacer par le port du serveur NS3

        self.NS3_RLT_SERVER_IP : str = '127.0.0.1'
        self.NS3_RLT_SERVER_PORT : int = 23456

        self.manage_response = ManageResponse(self.sock)
        self.manage_response.start()

        self.start_time = time.time()

        self.setup()

    def send_map_size(self):
        map = Map()
        map_x, map_y, map_z = map.get_map_size()
        map.show_map_size()
        self.sock.sendto(f"MAP|{map_x}|{map_y}|{map_z}|".encode(), (self.NS3_SERVER_IP, self.NS3_SERVER_PORT))

    def send_vehicle_number(self):
        vehicle_list = VehicleList()
        self.num_vehicles = vehicle_list.main()
        self.sock.sendto(f"VEHICLE|{self.num_vehicles}|".encode(), (self.NS3_SERVER_IP, self.NS3_SERVER_PORT))
        self.sock.sendto(f"VEHICLE|{self.num_vehicles}|".encode(), (self.NS3_RLT_SERVER_IP, self.NS3_RLT_SERVER_PORT))

    def odo_to_point(self, odo_msg):
        point_msg = Waypoint()
        #Position
        point_msg.position.x = odo_msg.pose.pose.position.x
        point_msg.position.y = odo_msg.pose.pose.position.y
        point_msg.position.z = odo_msg.pose.pose.position.z
        #Orientation
        point_msg.velocity.x = odo_msg.twist.twist.linear.x
        point_msg.velocity.y = odo_msg.twist.twist.linear.y
        point_msg.velocity.z = odo_msg.twist.twist.linear.z
        #Acceleration
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
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        print(f"Elapsed time: {elapsed_time}")
        print(f"Vehicle index: {vehicle_index}")
        message = f"COORDINATES|{vehicle_index}|{odo_data.pose.pose.position.x}|{odo_data.pose.pose.position.y}|{odo_data.pose.pose.position.z}|{odo_data.twist.twist.linear.x}|{odo_data.twist.twist.linear.y}|{odo_data.twist.twist.linear.z}|{elapsed_time}|".encode()
        print(message)
        self.sock.sendto(message, (self.NS3_SERVER_IP, self.NS3_SERVER_PORT))
        self.sock.sendto(message, (self.NS3_RLT_SERVER_IP, self.NS3_RLT_SERVER_PORT))

    def callback(self, data):         
        # Calculate vehicle index based on sequence number
        vehicle_index = data.header.seq % self.num_vehicles
        # print(f"Vehicle index: {vehicle_index}")
        self.publisher_odo(data, vehicle_index)

    def listener(self, pub, rate):     # Definition de notre subscriber
        rospy.init_node('odo_publisher', anonymous=True)
        rospy.Subscriber(f"/carla/ego_vehicle/odometry", Odometry, self.callback)
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

class ManageResponse(threading.Thread):
    def __init__(self, sock):
        threading.Thread.__init__(self)
        self.sock = sock
        self.running = True

    def run(self):
        while self.running:
            data, addr = self.sock.recvfrom(1024)
            print(f"Received response: {data.decode()}")
            self.manage_response(data)
    
    def manage_response(self, data):
        data = data.decode().split("|")
        if data[0] == "COORDINATES":
            self.car_index = int(data[1])
            self.car_position_x = data[2]
            self.car_position_y = data[3]
            self.car_position_z = data[4]
            
            # Nettoyer les chaînes et convertir en float
            self.car_velocity_x = round(float(data[5].replace('\x00', '')), 15)
            self.car_velocity_y = round(float(data[6].replace('\x00', '')), 15)
            self.car_velocity_z = round(float(data[7].replace('\x00', '')), 15)

            self.speed = sqrt(self.car_velocity_x**2 + self.car_velocity_y**2 + self.car_velocity_z**2)
            print(f"Car {self.car_index} Speed: {self.speed}")

        elif data[0] == 'DRAW':
            display_arrow = DisplayArrow()
            display_arrow.main()

        elif data[0] == "STOP":
            print("Stopping the program...")
            time.sleep(1.5)
            os._exit(0)
            

def stop():
    input("Press enter to stop the program\n")
    print("Stopping the program...")
    os._exit(0)

if __name__ == '__main__':
    rospy.init_node('odo_publisher', anonymous=True)
    threading.Thread(target=stop).start()
    gnss = GNSS()