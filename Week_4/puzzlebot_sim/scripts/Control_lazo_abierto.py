#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import numpy as np
import rospy
from geometry_msgs.msg import Twist
import time

def send_velocity():
    rospy.init_node('send_velocity_node', anonymous=True)
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    # Crear el mensaje Twist
    twist_msg = Twist()
    twist_msg.linear.x = 0.1  # Velocidad lineal deseada en m/s
    twist_msg.angular.z = 0.1

    start_time = time.time()

    while not rospy.is_shutdown() and time.time() - start_time < 40.0:
        vel_pub.publish(twist_msg)
        rate.sleep()

    # Detener el robot al finalizar los 4 segundos
    twist_msg.linear.x = 0.0
    twist_msg.angular.z = 0.0

    vel_pub.publish(twist_msg)

if __name__ == '__main__':
    try:
        send_velocity()
    except rospy.ROSInterruptException:
        pass