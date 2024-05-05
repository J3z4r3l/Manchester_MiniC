#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import numpy as np

class SquareTrajectory:
    def __init__(self):
        rospy.init_node('square_trajectory_node')
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz

    def move(self, linear_speed, angular_speed, distance):
        # Calcular el tiempo necesario para moverse a la distancia especificada
        if linear_speed != 0:
            time_to_move = distance / linear_speed

        else:
            time_to_move = 0  
        

        twist = Twist()
        twist.linear.x = linear_speed

        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < time_to_move:
            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()

        # Detener el movimiento
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        self.rate.sleep()

        angular_distance = np.pi / 2
        
        if angular_speed != 0:
            time_to_rotate = angular_distance / angular_speed
        else:
            time_to_rotate = 0  
        # Crear mensaje Twist para rotar
        twist.angular.z = angular_speed

        # Publicar el mensaje para rotar
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < time_to_rotate:
            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()

        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.rate.sleep()

    def move_ang(self, angular_speed):
        twist = Twist()
        angular_distance = np.pi / 4
        
        time_to_rotate = angular_distance / angular_speed

        # Crear mensaje Twist para rotar
        twist.angular.z = angular_speed

        # Publicar el mensaje para rotar
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < time_to_rotate:
            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()

        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.rate.sleep()


    def run(self):
        # Mover al punto (1,0)
        self.move(linear_speed=0.1, angular_speed=0.1, distance=1)
        self.move(linear_speed=0.1, angular_speed=0.1, distance=1)
        self.move(linear_speed=0.1, angular_speed=0.1, distance=1)
        self.move(linear_speed=0.1, angular_speed=0.1, distance=1)
        



        

if __name__ == '__main__':
    try:
        square_trajectory = SquareTrajectory()
        square_trajectory.run()
    except rospy.ROSInterruptException:
        pass
