#!/usr/bin/env python3
import rospy, time
from sensor_msgs.msg import Imu  # Import du type de message pour Imu 

def callback(data):             # Affiche en direct au travers de terminaux ROS les information dont nous avons besoin
    rospy.loginfo("Received Imu data:")
    rospy.loginfo("Linear Acceleration: x=%f, y=%f, z=%f", data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z)
    rospy.loginfo("Angular Velocity: x=%f, y=%f, z=%f", data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z)
    rospy.loginfo("Orientation: x=%f, y=%f, z=%f, w=%f", data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
    time.sleep(1)

def listener():     # Definition de notre subscriber
    rospy.init_node('subscriber_imu', anonymous=True)       # Nom de notre noeud
    rospy.Subscriber("/carla/ego_vehicle/imu", Imu, callback)   # Nous indiquons à quelle publishers nous nous souscrivons 
    rospy.spin()        # Permet au programme d'être en route continuellement

if __name__ == '__main__':
    listener()