#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped, Quaternion
from tf.transformations import quaternion_from_euler

class simulation:
     def __init__(self):
          #Initialize
          rospy.init_node("puzz_sim")
          self.loop_rate = rospy.Rate(rospy.get_param("~node_rate",100))
          #Parameters
          self.first=True
          self.radius=0.05
          self.wheelbase=0.19
          self.v_=0.0
          self.w_=0.0
          self.x_dot=0.0
          self.y_dot=0.0
          self.theta_dot=0.0
          #Publishers and Suscribers
          self.pub_pose = rospy.Publisher('/pose', PoseStamped, queue_size=10)         
          rospy.Subscriber('/cmd_vel',Twist,self.twist_callback)

     def twist_callback(self,msg):
          self.v_ = msg.linear.x
          self.w_ = msg.angular.z
  
     def wrap_to_Pi(self,theta):
        result = np.fmod((theta + np.pi),(2 * np.pi))
        if(result < 0):
                result += 2 * np.pi
        return result - np.pi

     def pose_stamped(self, x,y,theta_yaw):
          pose_robot=PoseStamped()
          pose_robot.header.stamp = rospy.Time.now()
          pose_robot.header.frame_id = "odom" #ask ????????
          pose_robot.pose.position.x = x
          pose_robot.pose.position.y = y
          quaternion=Quaternion(*quaternion_from_euler(0,0,theta_yaw))
          pose_robot.pose.orientation = quaternion
          return pose_robot
     
     def vel_xyz(self,theta):
          #No linealizado
          x_dot=self.v_*np.cos(theta) 
          y_dot=self.v_*np.sin(theta) 
          theta_dot=self.w_
          #linealizado
          #x_dot=-self.v_*np.sin(theta) #vel
          #y_dot=self.v_*np.cos(theta) #vel
          #theta_dot=self.w_      
          return x_dot,y_dot,theta_dot 
     
     def simulate(self):
          theta=0.0
          x=0.0
          y=0.0

          while not rospy.is_shutdown():
              
              current_time = rospy.Time.now().to_sec()  # Get current time
              if self.first:
                  self.previous_time = current_time
                  self.first = False
              else:
                  dt = current_time - self.previous_time  #get dt
                  self.previous_time = current_time
                  
                  #PoseStamped
                  self.x_dot, self.y_dot, self.theta_dot = self.vel_xyz(theta)
                  x+= self.x_dot*dt  
                  y+= self.y_dot*dt
                  theta+= self.theta_dot*dt 
                  print(self.wrap_to_Pi(theta))
                  pose_puzzlebot=self.pose_stamped(x,y,theta)
                  self.pub_pose.publish(pose_puzzlebot)
                  #end
                  self.loop_rate.sleep()

if __name__=='__main__':
    pendulum=simulation()
    try:
         pendulum.simulate()  
    except rospy.ROSInterruptException:
        pass 
    