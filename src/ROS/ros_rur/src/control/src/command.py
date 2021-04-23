#!/usr/bin/env python
# -*- coding: utf-8 -*-

# /!\ Ligne au dessus très importante
# Nécessaire pour faire tourner ros sous python
import rospy
from geometry_msgs.msg import PoseStamped, AccelStamped
from std_msgs.msg import Float64, Float64MultiArray
from numpy import cos, sin, pi

class Command:
    def __init__(self):
        self.u_speed = [0, 0, 0]
        self.u_angle = [0, 0, 0]
    ### TO COMPLETE ###
    def transform(self, robot_cmd):
        self.u_speed, self.u_angle = [0, 0, 0], [0, 0, 0]

def get_cmd(msg):
    ### A MODIFIER ###
    command.transform(msg.data)

def node(f = 10):
    # Init
    rospy.init_node('commande', anonymous=True)
    u_angle_pub = rospy.Publisher('u_angle', Float64MultiArray, queue_size=10)
    u_speed_pub = rospy.Publisher('u_speed', Float64MultiArray, queue_size=10)
    rospy.Subscriber('robot_cmd', Float64MultiArray, get_cmd)
    
    u_angle_msg = Float64MultiArray()
    u_speed_msg = Float64MultiArray()

    rate = rospy.Rate(f)
    # Possibilité faire un fichier externe à lire pour avoir les fréquences

    # Boucle ROS
    while not rospy.is_shutdown():
        # Update msg
        u_speed_msg.data = command.u_speed
        u_angle_msg.data = command.u_angle

        # Publish
        
        # rospy.loginfo("Sent : X = " + str(xtilde) + ", Y = " + str(ytilde))
        u_angle_pub.publish(u_angle_msg)
        u_speed_pub.publish(u_speed_msg)
        rate.sleep()

if __name__ == "__main__":
    command = Command()
    try:
        node()
    except rospy.ROSInterruptException:
        pass