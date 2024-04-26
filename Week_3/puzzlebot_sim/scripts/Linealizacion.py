#!/usr/bin/env python
import rospy 
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler
from tf import TransformBroadcaster

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
        self.theta = 0
        self.x=0
        self.y=0
        self.snt_tnf=TransformBroadcaster()
        self.pose_robot=PoseStamped()

        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        rospy.Subscriber("/wr", Float32, self.wr_callback)
        rospy.Subscriber("/wl", Float32, self.wl_callback)
        
         
    def wr_callback(self, v_r):
        self.wr_speed = v_r.data

    def wl_callback(self, v_l):
        self.wl_speed = v_l.data
    
    def get_pose_stamped(self,x,y,theta):
    # Update robot's pose
        self.pose_robot.pose.position.x += x       
        self.pose_robot.pose.position.y += y       
        quaternion = quaternion_from_euler(0, 0, theta)
        self.pose_robot.pose.orientation.x = quaternion[0]
        self.pose_robot.pose.orientation.y = quaternion[1]
        self.pose_robot.pose.orientation.z = quaternion[2]
        self.pose_robot.pose.orientation.w = quaternion[3]
    
    def get_odometry(self,current_time):
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "odom" #or odom
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = self.pose_robot.pose.position.x
        odom_msg.pose.pose.position.y = self.pose_robot.pose.position.y
        odom_msg.pose.pose.orientation.x = self.pose_robot.pose.orientation.x
        odom_msg.pose.pose.orientation.y = self.pose_robot.pose.orientation.y
        odom_msg.pose.pose.orientation.z = self.pose_robot.pose.orientation.z
        odom_msg.pose.pose.orientation.w = self.pose_robot.pose.orientation.w
        return odom_msg

    def transform(self,odom):
        tnf=TransformStamped()
        tnf.header.stamp=rospy.Time.now()
        tnf.header.frame_id= "odom"
        tnf.transform.translation.x = odom.pose.pose.position.x
        tnf.transform.translation.y = odom.pose.pose.position.y
        tnf.transform.translation.z = 0.0
        #rotation
        tnf.transform.rotation.x = odom.pose.pose.orientation.x
        tnf.transform.rotation.y = odom.pose.pose.orientation.y
        tnf.transform.rotation.z = odom.pose.pose.orientation.z
        tnf.transform.rotation.w = odom.pose.pose.orientation.w
        self.snt_tnf.sendTransform(
        (tnf.transform.translation.x, tnf.transform.translation.y, tnf.transform.translation.z),
        (tnf.transform.rotation.x, tnf.transform.rotation.y, tnf.transform.rotation.z, tnf.transform.rotation.w),
        rospy.Time.now(),
        "base_link",
        tnf.header.frame_id
    )
    
    def wrap_to_Pi(self,theta):
        result = np.fmod((theta + np.pi),(2 * np.pi))
        if(result < 0):
                result += 2 * np.pi
        return result - np.pi
    
    def get_velocity(self):
        v = self.radius * (self.wr_speed + self.wl_speed) / 2.0
        w = self.radius * (self.wr_speed - self.wl_speed) / self.wheelbase
        return v,w


    def get_position(self,x,y,theta,dt):
        v,w=self.get_velocity()
        x = x-(v * np.sin(self.theta) * dt*theta)
        y = y+v * np.cos(self.theta) * dt*theta
        theta = self.wrap_to_Pi(w)  # Actualiza theta correctamente
        return x,y,theta


    
    def calculate_odometry(self):
        current_time = rospy.Time.now()  # Get current time
    
        if self.first:
            self.previous_time = current_time
            self.first = False
        else:
            dt = (current_time - self.previous_time).to_sec()  # get dt
            self.previous_time = current_time
            
            #get pose
            self.x,self.y,self.theta=self.get_position(self.x,self.y,self.theta,dt)
            self.get_pose_stamped(self.x,self.y,self.theta)

    
            # Create Odometry message
            odom_msg = self.get_odometry(current_time)
            
            # Publish Odometry message
            self.odom_pub.publish(odom_msg)
    
            # Publish transform
            self.transform(odom_msg)
    
    def run(self):
           # Run the node
           while not rospy.is_shutdown():
               self.calculate_odometry()
               self.rate.sleep()

if __name__ == "__main__":
    try:
        node = LocalizationNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
