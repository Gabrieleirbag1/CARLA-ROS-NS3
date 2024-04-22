#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16MultiArray  # Import du type de message pour Int16MultiArray

def callback(data):             # Affiche en direct au travers de terminaux ROS les information dont nous avons besoin
    rospy.loginfo("Received data:")
    rospy.loginfo("Data: %s", str(data.data))

def listener():     # Definition de notre subscriber
    rospy.init_node('subscriber_int16_multi_array', anonymous=True)       # Nom de notre noeud
    rospy.Subscriber("/routing_table", Int16MultiArray, callback)    # Nous indiquons à quelle publishers nous nous souscrivons 
    rospy.spin()        # Permet au programme d'être en route continuellement

if __name__ == '__main__':
    listener()

