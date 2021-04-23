#!/usr/bin/env python
# -*- coding: utf-8 -*-

# /!\ Ligne au dessus très importante
# Nécessaire pour faire tourner ros sous python
import rospy
from std_msgs.msg import String, Float64MultiArray
import time

class ESC:
    #### TO COMPLETE ####
    def send_cmd(self, u1, u2, u3):
        pass


def close():
    ## A CHANGER POUR COUPER LES MOTEURS A LA FERMETURE ##
    esc.send_cmd(0, 0, 0)
    rospy.loginfo("PROPELLERS SHUTTED DOWN SUCCESSFULLY")


def get_command(msg):
    u1, u2, u3 = msg.data

    esc.send_cmd(u1, u2, u3)

def node(f = 40):
    # Init
    rospy.init_node('ESC', anonymous=True)
    rospy.Subscriber('u_speed', Float64MultiArray, get_command)

    rate = rospy.Rate(f)
    # Possibilité faire un fichier externe à lire pour avoir les fréquences

    # Boucle ROS
    while not rospy.is_shutdown():
        
        rate.sleep()

    # Pour fermer correctement le noeud : coupure des moteurs
    time.sleep(0.1)
    rospy.on_shutdown(close)

if __name__ == '__main__':
    esc = ESC()
    try:
        node()
    except rospy.ROSInterruptException:
        pass