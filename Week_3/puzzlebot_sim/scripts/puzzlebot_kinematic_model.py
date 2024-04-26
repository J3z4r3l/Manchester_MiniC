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
          rospy.init_node("puzz_sim")
          self.loop_rate = rospy.Rate(rospy.get_param("~node_rate",100))
          
          #Parameters
          self.first=True
          self.radius=0.05
          self.wheelbase=0.19
          self.pose_theta_wl=0.0
          self.pose_theta_wr=0.0
          self.v_=0.0
          self.w_=0.0
          self.x_dot=0.0
          self.y_dot=0.0
          self.theta_dot=0.0

          #Publishers and Suscribers
          self.pub_js = rospy.Publisher('/joint_states', JointState, queue_size=10)                   
          self.pub_pose = rospy.Publisher('/pose', PoseStamped, queue_size=10)         
          self.wr=rospy.Publisher("/wr",Float32,queue_size=10)
          self.wl=rospy.Publisher("/wl",Float32,queue_size=10)
          rospy.Subscriber('/cmd_vel',Twist,self.twist_callback)

     def twist_callback(self,msg):
          self.v_ = msg.linear.x
          self.w_ = msg.angular.z

     def init_joints(self):
          self.msg=JointState()
          self.msg.header.frame_id= "base_link"
          self.msg.header.stamp= rospy.Time.now()
          self.msg.name.extend(["wheel_joint1", "wheel_joint2"])
          self.msg.position.extend([0.0, 0.0])
          self.msg.velocity.extend([0.0, 0.0])
          self.msg.effort.extend([0.0, 0.0])
     
     def position_theta(self,wr,wl,dt):
          wr_pos=wr*dt
          wl_pos=wl*dt
          return wr_pos,wl_pos
          
     def pose_stamped(self, x,y,theta_yaw):
          pose_robot=PoseStamped()
          pose_robot.header.stamp = rospy.Time.now()
          pose_robot.header.frame_id = "odom" #ask ????????
          pose_robot.pose.position.x = x
          pose_robot.pose.position.y = y
          quaternion=quaternion_from_euler(0,0,theta_yaw)
          pose_robot.pose.orientation.x = quaternion[0]
          pose_robot.pose.orientation.y = quaternion[1]
          pose_robot.pose.orientation.z = quaternion[2]
          pose_robot.pose.orientation.w = quaternion[3]
          #update the position
          return pose_robot
     
     def vel_xyz(self,theta):
          x_dot=self.v_*np.cos(theta) #vel
          y_dot=self.v_*np.sin(theta) #vel
          theta_dot=self.w_
                  
          return x_dot,y_dot,theta_dot 
     
     def simulate(self):
          self.init_joints()
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
                  
                  #get w's velocities and pub
                  wr_1 = (2.0*self.v_ + self.wheelbase *self.w_)/(2.0*self.radius)
                  wl_1= (2.0*self.v_ - self.wheelbase * self.w_)/(2.0*self.radius)
                  self.wr.publish(wr_1)
                  self.wl.publish(wl_1)
                  
                  #Joints_transforms for wheels posses
                  self.pose_theta_wr,self.pose_theta_wl=self.position_theta(wr_1,wl_1,dt,)
                  self.msg.header.stamp = rospy.Time.now()
                  self.msg.position[0] +=  self.pose_theta_wr
                  self.msg.position[1] += self.pose_theta_wl
                  self.pub_js.publish(self.msg)
                  
                  #PoseStamped
                  self.x_dot, self.y_dot, self.theta_dot = self.vel_xyz(theta)
                  x+= self.x_dot*dt  
                  y+= self.y_dot*dt
                  theta+= self.theta_dot*dt 
                  pose_puzzlebot=self.pose_stamped(x,y,theta)
                  #self.pub_pose.publish(pose_puzzlebot)
                  #end
                  self.loop_rate.sleep()

if __name__=='__main__':
    pendulum=simulation()
    try:
         pendulum.simulate()  
    except rospy.ROSInterruptException:
        pass 
    