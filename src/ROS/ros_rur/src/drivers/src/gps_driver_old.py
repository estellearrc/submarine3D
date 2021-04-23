#!/usr/bin/env python
# -*- coding: utf-8 -*-

# /!\ Ligne au dessus très importante
# Nécessaire pour faire tourner ros sous python
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from numpy import cos, sin, pi

class GPS:
    ### TO COMPLETE ###
    def __init__(self):
        self.is_shutdown = False

    def read_gll(self):
        if not self.is_shutdown:
            return 0, 0, 0, 0
        else:
            # Valeurs par défaut possibles de changer
            return -9999, -9999, -9999, -9999


    # Les deux méthodes suivantes sont nécessaires afin d'éviter de
    # surcharger le bus de communication en commandes vers le GPS
    def shutdown(self):
        if self.is_shutdown == False:
            ### ETEINDRE LE GPS // ARRETER LA RECHERCHE DE SIGNAL ###
            pass

    def startup(self):
        if self.is_shutdown == True:
            ### ALLUMER LE GPS // REPRENDRE LA RECHERCHE DE SIGNAL ###
            pass


def get_pressure(msg):
    p = msg.data
    if p <= 3: # (Valeur en mètres / A modifier si besoin)
        gps.shutdown()
    else:
        gps.startup()

def node(f = 10):
    # Init
    rospy.init_node('GPS', anonymous=True)
    pub = rospy.Publisher('GPS_data', PoseStamped, queue_size=10)
    rospy.Subscriber('pressure_data', Float64, get_pressure)
    pos = PoseStamped()

    lx0 = -3.01473333 * (pi/180) # longitude ref (ssi dans un plan)
    ly0 = 48.19906500 * (pi/180) # latitude ref (ssi dans un plan)
    ro = 6371000 # Rayon de la terre (mètres)

    rate = rospy.Rate(f)
    # Possibilité faire un fichier externe à lire pour avoir les fréquences

    # Boucle ROS
    while not rospy.is_shutdown():

        # Lecture GPS
        val = gps.read_gll()
        lx = val[2] * (pi/180) # Valeurs supposées en °
        ly = val[0] * (pi/180) # Valeurs supposées en °

        # Valide SSI dans un espace considéré plan
        xtilde = ro*cos(ly)*(lx-lx0) # Conversion lattitude/longitude -> x
        ytilde = ro * (ly - ly0) # Conversion lattitude -> y

        # Update msg
        pos.pose.position.x = xtilde
        pos.pose.position.y = ytilde
        
        # Publish
        
        # rospy.loginfo("Sent : X = " + str(xtilde) + ", Y = " + str(ytilde))
        pub.publish(pos)
        rate.sleep()

if __name__ == "__main__":
    gps = GPS()
    try:
        node()
    except rospy.ROSInterruptException:
        pass