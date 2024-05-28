#!/usr/bin/env python3
import rospy, socket, time
from sensor_msgs.msg import NavSatFix

class GNSS():
    def __init__(self, *args, **kwargs):
        self.pub, self.rate = self.initisalisation()
        self.listener(self.pub, self.rate)

    def initisalisation(self):
        rospy.init_node('gnss_publisher', anonymous=True)
        pub = rospy.Publisher('gnss_data', NavSatFix, queue_size=10)
        rate = rospy.Rate(10)
        return pub, rate

    def listener(self, pub, rate):     # Definition de notre subscriber
        rospy.Subscriber("/carla/ego_vehicle/gnss", NavSatFix, self.callback)  # Nous indiquons à quelle publishers nous nous souscrivons 
        rospy.spin()    # Permet au programme d'être en route continuellement
    
    def publisher_gnss(self, gnss_data):
        # Initialiser le nœud ROS
        print("=============================")
        # Publier les données GNSS sur le topic
        self.pub.publish(gnss_data)
        self.rate.sleep()

        # Envoyer les données au serveur NS3
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        NS3_SERVER_IP = '127.0.0.1'  # Remplacer par l'adresse IP du serveur NS3
        NS3_SERVER_PORT = 12345  # Remplacer par le port du serveur NS3
        sock.sendto(str((gnss_data.latitude, gnss_data.longitude, gnss_data.altitude)).encode(), (NS3_SERVER_IP, NS3_SERVER_PORT))
        # time.sleep(2)

    def callback(self, data):         # Affiche en direct au travers de terminaux ROS les information dont nous avons besoin
        print("Received NavSatFix data:")
        rospy.loginfo("Latitude: %f", data.latitude)
        rospy.loginfo("Longitude: %f", data.longitude)
        rospy.loginfo("Altitude: %f", data.altitude)
        gnss_data = NavSatFix()
        gnss_data.latitude = data.latitude
        gnss_data.longitude = data.longitude
        gnss_data.altitude = data.altitude

        self.publisher_gnss(gnss_data)

if __name__ == '__main__':
    gnss = GNSS()