#!/usr/bin/env python3
import rospy, time
from sensor_msgs.msg import NavSatFix  # Import du type de message pour NavSatFix (GPS)

def callback(data):         # Affiche en direct au travers de terminaux ROS les information dont nous avons besoin
    rospy.loginfo("Received NavSatFix data:")
    rospy.loginfo("Latitude: %f", data.latitude)
    rospy.loginfo("Longitude: %f", data.longitude)
    rospy.loginfo("Altitude: %f", data.altitude)
    time.sleep(1)

def listener():     # Definition de notre subscriber
    rospy.init_node('subscriber_navsatfix', anonymous=True)     # Nom de notre noeud
    rospy.Subscriber("/carla/ego_vehicle/gnss", NavSatFix, callback)  # Nous indiquons à quelle publishers nous nous souscrivons 
    rospy.spin()    # Permet au programme d'être en route continuellement

if __name__ == '__main__':
    listener()