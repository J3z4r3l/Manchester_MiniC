#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion
from tf import TransformBroadcaster
from std_msgs.msg import Float32
import numpy as np
from tf.transformations import quaternion_from_euler



class TrianglePublisher:
    def __init__(self):
        rospy.init_node('pose_publisher', anonymous=True)
        rospy.Subscriber("/wr", Float32, self.wr_real_callback)
        rospy.Subscriber("/wl", Float32, self.wl_real_callback)
        self.rate = rospy.Rate(100)  # 1 Hz
        self.half_dist = 0.0#15  # equals y
        self.x_base_pose = 0.0#3.39
        self.rotation = 0.0#0.026
        self.wr = 0.0
        self.wl = 0.0
        self.first=True
        self.previous_time=0.0
        self.radius=0.05
        self.wheelbase=0.19
        self.theta=0.0
        self.x=0.0
        self.y=0.0

    def wr_real_callback(self, msg):
        self.wr = msg.data

    def wl_real_callback(self, msg):
        self.wl = msg.data
    
    def wrap_to_Pi(self,theta):
        result = np.fmod((theta + np.pi),(2 * np.pi))
        if(result < 0):
                result += 2 * np.pi
        return result - np.pi
   
    def get_position(self, dt):
        self.vel = self.radius * (self.wr + self.wl) / 2
        self.w = self.radius * (self.wr - self.wl) / self.wheelbase
        self.theta += self.w * dt  # 
          #
        self.x = self.vel * np.cos(self.theta) * dt
        self.y = self.vel * np.sin(self.theta) * dt

        return self.x, self.y, self.theta
    
    def publish_triangle(self):
        dx=0.0 
        dy=0.0   
        dtheta=0.0
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()  # Get current time
            if self.first:
                self.previous_time = current_time
                self.first = False
            else:
                dt = (current_time - self.previous_time).to_sec()  # get dt
                self.previous_time = current_time
                dx, dy, dtheta = self.get_position(dt)
                self.x_base_pose += dx 
                self.half_dist += dy   
                self.rotation = self.wrap_to_Pi(dtheta)
                print("x")
                print(self.x_base_pose)
                print("y")
                print(self.half_dist)
                print("theta")
                rotation_quaternion = quaternion_from_euler(0, 0, self.rotation)
                print(rotation_quaternion[2])
                self.rate.sleep()

if __name__ == '__main__':
    try:
        triangle_publisher = TrianglePublisher()
        triangle_publisher.publish_triangle()
    except rospy.ROSInterruptException:
        pass
