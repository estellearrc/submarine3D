#!/usr/bin/env python
# -*- coding: utf-8 -*-

# /!\ Ligne au dessus très importante
# Nécessaire pour faire tourner ros sous python
import rospy
from std_msgs.msg import String

class SIM:
    ### TO COMPLETE ###
    def __init__(self):
        self.msg = ""

def get_msg(msg):
    ### A MODIFIER ###
    sim.msg = msg.data

def node(f = 10):
    # Init
    rospy.init_node('sim', anonymous=True)
    pub = rospy.Publisher('SIM_received', String, queue_size=10)
    rospy.Subscriber('SIM_send', String, get_msg)
    
    msg = String()

    rate = rospy.Rate(f)
    # Possibilité faire un fichier externe à lire pour avoir les fréquences

    # Boucle ROS
    while not rospy.is_shutdown():
        # Update msg
        msg = ""

        # Publish
        
        # rospy.loginfo("Sent : X = " + str(xtilde) + ", Y = " + str(ytilde))
        pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    sim = SIM()
    try:
        node()
    except rospy.ROSInterruptException:
        pass
