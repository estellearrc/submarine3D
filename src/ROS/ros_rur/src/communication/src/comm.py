#!/usr/bin/env python
# -*- coding: utf-8 -*-

# /!\ Ligne au dessus très importante
# Nécessaire pour faire tourner ros sous python
import rospy
from std_msgs.msg import Bool, String
from numpy import cos, sin, pi, array

class Communication:
    ### TO COMPLETE ###
    def __init__(self):
        self.sim = ""
        self.cam = False

def get_sim(msg):
    com.sim = msg.data

def get_camera(msg):
    ### A MODIFIER ###
    com.cam = msg.data

def node(f = 10):
    # Init
    rospy.init_node('controle', anonymous=True)
    pubCam = rospy.Publisher('CAM_send', Bool, queue_size=10)
    pubSim = rospy.Publisher('SIM_send', String, queue_size=10)
    rospy.Subscriber('SIM_received', String, get_sim)
    rospy.Subscriber('CAM_received', Bool, get_camera)
    
    cam_msg = Bool()
    sim_msg = String()

    rate = rospy.Rate(f)
    # Possibilité faire un fichier externe à lire pour avoir les fréquences

    # Boucle ROS
    while not rospy.is_shutdown():
        # Update msg
        cam_msg.data = com.cam
        sim_msg.data = com.sim

        # Publish
        
        # rospy.loginfo("Sent : X = " + str(xtilde) + ", Y = " + str(ytilde))
        pubCam.publish(cam_msg)
        pubSim.publish(sim_msg)
        rate.sleep()

if __name__ == "__main__":
    com = Communication()
    try:
        node()
    except rospy.ROSInterruptException:
        pass