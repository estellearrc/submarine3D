#!/usr/bin/env python
# -*- coding: utf-8 -*-

# /!\ Ligne au dessus très importante
# Nécessaire pour faire tourner ros sous python
import rospy
from std_msgs.msg import Bool

class Camera:
    ### TO COMPLETE ###
    pass

def node(f = 10):
    # Init
    rospy.init_node('camera', anonymous=True)
    pub = rospy.Publisher('CAM_received', Bool, queue_size=10)
    
    msg = Bool()

    rate = rospy.Rate(f)
    # Possibilité faire un fichier externe à lire pour avoir les fréquences

    # Boucle ROS
    while not rospy.is_shutdown():

        # Update msg
        msg.data = False

        # Publish
        
        # rospy.loginfo("Sent : X = " + str(xtilde) + ", Y = " + str(ytilde))
        pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    camera = Camera()
    try:
        node()
    except rospy.ROSInterruptException:
        pass