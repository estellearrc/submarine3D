#!/usr/bin/env python
# -*- coding: utf-8 -*-

# /!\ Ligne au dessus très importante
# Nécessaire pour faire tourner ros sous python
import rospy
from std_msgs.msg import Bool

class LED:
    ### TO COMPLETE ###
    def __init__(self):
        self.value = False

def get_msg(msg):
    ### A MODIFIER ###
    led.value = msg.data

def node(f = 10):
    # Init
    rospy.init_node('led', anonymous=True)
    rospy.Subscriber('CAM_send', Bool, get_msg)
    
    msg = Bool()

    rate = rospy.Rate(f)
    # Possibilité faire un fichier externe à lire pour avoir les fréquences

    # Boucle ROS
    while not rospy.is_shutdown():

        
        rate.sleep()

if __name__ == "__main__":
    led = LED()
    try:
        node()
    except rospy.ROSInterruptException:
        pass