#!/usr/bin/env python3
import rospy, time
from std_msgs.msg import Float32  # Import du type de message pour Float32

def callback(data):             # Affiche en direct au travers de terminaux ROS les information dont nous avons besoin
    rospy.loginfo("Received data: %f", data.data)
    time.sleep(1)

def listener():     # Definition de notre subscriber
    rospy.init_node('subscriber_speedometer', anonymous=True)       # Nom de notre noeud
    rospy.Subscriber("/carla/ego_vehicle/speedometer", Float32, callback)    # Nous indiquons à quelle publishers nous nous souscrivons 
    rospy.spin()        # Permet au programme d'être en route continuellement

if __name__ == '__main__':
    listener()