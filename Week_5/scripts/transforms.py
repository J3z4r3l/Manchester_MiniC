#!/usr/bin/env python
import rospy 
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Quaternion
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Float64MultiArray


class transform:
    def __init__(self):
          #Initialize
          rospy.init_node("puzzlebot_transform")
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
          self.wr_speed=0.0
          self.wl_speed=0.0
          self.x_odom=0.0
          self.y_odom=0.0
          self.x_pose=0.0
          self.y_pose=0.0
          self.wl_r_speed = 0.0
          self.wl_r_speed = 0.0
          #Messages 
          self.snt_tnf=TransformBroadcaster()
          self.tnf=TransformStamped()
          self.quaternion=Quaternion(*quaternion_from_euler(0,0,0))
          self.quaternion2=Quaternion(*quaternion_from_euler(0,0,0))
          self.msg=JointState()
          #Publishers and Suscribers
          self.pub_js = rospy.Publisher('/joint_states', JointState, queue_size=10)                   
          rospy.Subscriber('/odom', Odometry, self.odom_callback)         
          rospy.Subscriber("/wr_1",Float32,self.wr_callback)
          rospy.Subscriber("/wl_1",Float32,self.wl_callback)
          
 
    def wr_callback(self, v_r):
        self.wr_speed = v_r.data
    def wl_callback(self, v_l):
        self.wl_speed = v_l.data

    def odom_callback(self, odom):
        self.x_odom = odom.pose.pose.position.x
        self.y_odom = odom.pose.pose.position.y
        self.quaternion = odom.pose.pose.orientation    
     
    def init_joints(self):
          self.msg.header.frame_id= "base_link"
          self.msg.header.stamp= rospy.Time.now()
          self.msg.name.extend(["wheel_joint1", "wheel_joint2"])
          self.msg.position.extend([0.0, 0.0])
          self.msg.velocity.extend([0.0, 0.0])
          self.msg.effort.extend([0.0, 0.0])
      
     
    def transform(self):
        tnf=TransformStamped()
        tnf.header.stamp=rospy.Time.now()
        tnf.header.frame_id= "odom"
        tnf.child_frame_id="base_link"
        tnf.transform.translation.x = self.x_odom
        tnf.transform.translation.y = self.y_odom
        tnf.transform.translation.z = 0.0
        #rotation
        tnf.transform.rotation = self.quaternion
        self.snt_tnf.sendTransformMessage(tnf)
    
    def run(self):
        self.init_joints()
            
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()  # Get current time

            if self.first:
                self.previous_time = current_time
                self.first = False
            else:
                dt = (current_time - self.previous_time).to_sec()  # get dt
                self.previous_time = current_time
                #wheel joints
                self.msg.header.stamp = rospy.Time.now()
                self.msg.position[0] +=  self.wr_speed*dt
                self.msg.position[1] += self.wl_speed*dt
                self.pub_js.publish(self.msg)
                # Publish transform
                self.transform()

if __name__=='__main__':
    trans=transform()
    try:
         trans.run()  
    except rospy.ROSInterruptException:
        pass 
    
