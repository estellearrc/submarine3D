#!/usr/bin/env python
# -*- coding: utf-8 -*-

# /!\ Ligne au dessus très importante
# Nécessaire pour faire tourner ros sous python
import rospy
from std_msgs.msg import String, Float64
import time

class pressure_sensor:
    #### TO COMPLETE ####
    def get_pressure(self):
        return 0

def node(f = 20):
    # Init
    rospy.init_node('pression', anonymous=True)
    pub = rospy.Publisher('pressure_data', Float64, queue_size=10)
    msg = Float64()

    rate = rospy.Rate(f)
    # Possibilité faire un fichier externe à lire pour avoir les fréquences

    # Boucle ROS
    while not rospy.is_shutdown():
        
        # Lecture capteur de pression
        p = sensor.get_pressure()
        
        # Update msg
        msg.data = p
        
        # Publish

        # rospy.loginfo("Sent : pressure = " + str(p))
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    sensor = pressure_sensor()
    try:
        node()
    except rospy.ROSInterruptException:
        pass