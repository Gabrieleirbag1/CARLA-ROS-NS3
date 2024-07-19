#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import socket, time

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

        # Envoyer les données au serveur NS3
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        NS3_SERVER_IP = '127.0.0.1'  # Remplacer par l'adresse IP du serveur NS3
        NS3_SERVER_PORT = 12345  # Remplacer par le port du serveur NS3
        sock.sendto(hello_str.encode(), (NS3_SERVER_IP, NS3_SERVER_PORT))
        time.sleep(2)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass