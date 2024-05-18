#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import quaternion_from_euler

class simulation:
    def __init__(self):
          #Initialize
          rospy.init_node("puzzlebot_wheels")
          self.loop_rate = rospy.Rate(rospy.get_param("~node_rate",100))
          #Parameters
          self.radius=0.05
          self.wheelbase=0.19
          self.v_=0.0
          self.w_=0.0
          #Publishers and Suscribers
          self.wr=rospy.Publisher("/wr_1",Float32,queue_size=10)
          self.wl=rospy.Publisher("/wl_1",Float32,queue_size=10)
          rospy.Subscriber('/cmd_vel',Twist,self.twist_callback)

    def twist_callback(self,msg):
          self.v_ = msg.linear.x
          self.w_ = msg.angular.z
         
    
    def simulate_wheels(self):
         while not rospy.is_shutdown():
            #get w's velocities and pub
            wr_1 = (2.0*self.v_ + self.wheelbase *self.w_)/(2.0*self.radius)
            wl_1= (2.0*self.v_ - self.wheelbase * self.w_)/(2.0*self.radius)
            self.wr.publish(wr_1)
            self.wl.publish(wl_1)
            self.loop_rate.sleep()

if __name__=='__main__':
    pendulum=simulation()
    try:
         pendulum.simulate_wheels()  
    except rospy.ROSInterruptException:
        pass 
    