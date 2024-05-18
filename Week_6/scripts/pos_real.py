#!/usr/bin/env python
import rospy 
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler



class LocalizationNode:
    def __init__(self):
        # Initialize the robot's pose
        rospy.init_node("localisation")
        self.rate = rospy.Rate(100) 
        self.previous_time = rospy.Time.now()

        # Define the wheelbase and radius of the robot
        self.first = True
        self.wheelbase = 0.19 
        self.radius = 0.05 
        self.wr_speed = 0.0
        self.wl_speed = 0.0
        self.vel=0.0
        self.w=0.0
        self.theta = 0.0
        self.x=0.0
        self.y=0.0
        self.rotation=0.0
        #variables to tune
        self.kr=1.7
        self.kl=1.8
        self.wr_r_speed=0.0
        self.wl_r_speed=0.0
        self.odom_msg = Odometry()
        self.snt_tnf=TransformBroadcaster()
        self.pose_robot=PoseStamped()
        self.H_matrix=np.zeros((3,3),dtype=float)
        self.covariance_matrix=np.zeros((3,3),dtype=float)
        self.Qk_matrix=np.zeros((3,3),dtype=float)
        self.cov_delta_q=np.array([[self.kr * abs(self.wr_speed), 0], [0, self.kl * abs(self.wl_speed)]],dtype=float)
        self.quaternion=0.0
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        rospy.Subscriber("/puzzlebot_1/wr",Float32,self.wr_real_callback)
        rospy.Subscriber("/puzzlebot_1/wl",Float32,self.wl_real_callback)
        rospy.Subscriber('/puzzlebot_1/base_controller/cmd_vel', Twist, self.callback_twist)
        self.vel_x_t=0.0
        self.vel_z_t=0.0

    def callback_twist(self,msg):
        self.vel_x_t=msg.linear.x
        self.vel_z_t=msg.angular.z
        
    
    def wr_real_callback(self, v_r):
        self.wr_r_speed = v_r.data
    def wl_real_callback(self, v_l):
        self.wl_r_speed = v_l.data
        
    def get_positon(self,wr,wl,dt):
        self.vel=self.radius*(wr+wl)/2
        self.w=self.radius*(wr-wl)/self.wheelbase
        self.theta += self.w*dt
        self.x += self.vel*np.cos(self.theta)*dt
        self.y += self.vel*np.sin(self.theta)*dt
        return  self.theta 
      
    def get_odometry(self,current_time,x,y,theta,covarianza):
        self.odom_msg.header.stamp = current_time
        self.odom_msg.header.frame_id = "odom" #or odom
        self.odom_msg.child_frame_id = "base_link"
        self.odom_msg.pose.pose.position.x = x
        self.odom_msg.pose.pose.position.y = y
        self.quaternion=Quaternion(*quaternion_from_euler(0,0,theta))
        
        self.odom_msg.pose.pose.orientation= self.quaternion
        #
        co = np.zeros((6,6),dtype=float)
        co[:2,:2] = covarianza[:2,:2]
        co[-1,:2] = covarianza[-1,:2]
        co[:2,-1] = covarianza[:2,-1]
        co[-1,-1] = covarianza[-1,-1]
        self.odom_msg.pose.covariance = co.reshape(36).tolist()
        #self.odom_msg.twist.twist.linear.x=self.vel
        #self.odom_msg.twist.twist.angular.z= self.w
        return self.odom_msg

    def wrap_to_Pi(self,theta):
        result = np.fmod((theta + np.pi),(2 * np.pi))
        if(result < 0):
                result += 2 * np.pi
        return result - np.pi
    
    
    def calculate_odometry(self):
        dtheta=0.0
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()  # Get current time
            if self.first: 
                self.previous_time = current_time
                if self.vel_x_t!=0 or self.vel_z_t!=0:
                    self.first = False
                odom_msg = self.get_odometry(current_time,0.0,0.0,0,self.covariance_matrix)
                # Publish Odometry message
                self.odom_pub.publish(odom_msg)
                print("in")
                
            else:
                print("out")             
                dt = (current_time - self.previous_time).to_sec()  # get dt
                self.previous_time = current_time
                #get the pose of the robot
                dtheta=self.get_positon(self.wr_r_speed,self.wl_r_speed,dt)
                # Create Odometry message
                self.rotation=self.wrap_to_Pi(dtheta)
                odom_msg = self.get_odometry(current_time,self.x,self.y,self.rotation    ,self.covariance_matrix)
                print(self.quaternion.z)
                # Publish Odometry message
                self.odom_pub.publish(odom_msg)
                self.rate.sleep()
                



if __name__ == "__main__":
    try:
        node = LocalizationNode()
        node.calculate_odometry()
    except rospy.ROSInterruptException:
        pass
