#!/usr/bin/env python
# -*- coding: utf-8 -*-

# /!\ Ligne au dessus très importante
# Nécessaire pour faire tourner ros sous python
import rospy
from geometry_msgs.msg import PoseStamped, AccelStamped
from std_msgs.msg import Float64, Float64MultiArray
from numpy import cos, sin, pi, array

class Control:
    ### TO COMPLETE ###
    def f(self, x, u):
        return x ### A CODER ###

    def euler(self, x, u, dt):
        return x + dt*self.f(x,u)

    def command(self, x, w):
        # Return robot_cmd
        return [0, 0, 0, 0, 0, 0]


def get_pressure(msg):
    control.pressure = msg.data

def get_gps(msg):
    ### A MODIFIER ###
    control.gps = msg.pose

def get_imu(msg):
    ### A MODIFIER ###
    control.imu = msg.accel

def node(f = 10):
    # Init
    rospy.init_node('controle', anonymous=True)
    pub = rospy.Publisher('robot_cmd', Float64MultiArray, queue_size=10)
    rospy.Subscriber('pressure_data', Float64, get_pressure)
    rospy.Subscriber('GPS_data', PoseStamped, get_gps)
    rospy.Subscriber('IMU_data', AccelStamped, get_imu)
    
    robot_cmd_msg = Float64MultiArray()

    rate = rospy.Rate(f)
    # Possibilité faire un fichier externe à lire pour avoir les fréquences

    # Init vars
    x = array([[0, 0, 0]]).T ## Vecteur d'état à déterminer
    w = array([[0, 0]]).T ## Vecteur consigne à déterminer
    dt = 0.1

    # Boucle ROS
    while not rospy.is_shutdown():
        u = control.command(x, w)
        control.f(x, u)
        control.euler(x, u, dt)

        # Update msg
        robot_cmd_msg.data = u

        # Publish
        
        # rospy.loginfo("Sent : X = " + str(xtilde) + ", Y = " + str(ytilde))
        pub.publish(robot_cmd_msg)
        rate.sleep()

if __name__ == "__main__":
    control = Control()
    try:
        node()
    except rospy.ROSInterruptException:
        pass