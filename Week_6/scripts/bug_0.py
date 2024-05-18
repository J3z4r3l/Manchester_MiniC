#! /usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool
from tf.transformations import euler_from_quaternion

##New basic controller
class ControlPID:
    def __init__(self, dt, kp = 1, ki = 0, kd = 0):
        self.dt = dt
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.error_actual = 0
        self.pred = 0
        ##Inicialiace points
        self.x_list=0.0
        self.y_list=0.0
        #errosself.
        self.error_ang=0.0

    #get the points 
    def points_callback(self,x,y):
        self.x_list=x
        self.y_list=y
    #get the velocity for the distance we need 
    def get_pid(self, current, goal):
        error_p = goal - current
        self.integral = self.integral + self.ki*error_p*self.dt
        ed = (error_p - self.error_actual)/self.dt
        self.error_actual = error_p
        self.pred = current + (error_p*self.kp) + self.integral + (ed*self.kd)
        return self.pred
    
    def get_pid_d(self,goal):
        error_p = goal 
        self.integral = self.integral + self.ki*error_p*self.dt
        ed = (error_p - self.error_actual)/self.dt
        self.error_actual = error_p
        self.pred = (error_p*self.kp) + self.integral + (ed*self.kd)
        return self.pred
    
    #get angular error
    def e_ang (self,z_pose,x,y):
        self.error_ang = np.arctan2(self.y_list - y, self.x_list - x)
        a = self.error_ang - z_pose
        #find best possible angle 
        #a -= 2*np.pi if a > np.pi else -2*np.pi if a < -np.pi else 0
        return a
    #get distance error
    def e_dis(self,x,y):
        error_dist = np.sqrt((self.x_list - x) ** 2 + (self.y_list -y) ** 2)
        return error_dist

class Bugs:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('bug_node')
        #start sus and publishers
        rospy.Subscriber('/odom',Odometry,self.odom_callback)
        rospy.Subscriber('/puzzlebot_1/scan',LaserScan,self.callbackScan)
        self.vel_pub=rospy.Publisher('/puzzlebot_1/base_controller/cmd_vel', Twist, queue_size=10)
       
        #start values
        self.vel = Twist()
        self.x_odom=0.0 
        self.y_odom=0.0 
        self.quaternion=0.0
        self.pos_z=0.0
        self.rate = rospy.Rate(100)
        self.first = True
        self.goal_x=[1,1,0,0]
        self.goal_y=[-1.5,2,1,1,0]
        self.turnPID = ControlPID(0.09,kp = 0.8)
        self.fowaPID = ControlPID(0.09,kp = 0.2,kd = 0.005)
        self.regions=0.0
        self.obstacle=2
        self.avoid_count = 0
        self.rotate_start_time =0.0
        
        
                
        
    def odom_callback(self, odom):
        self.x_odom = odom.pose.pose.position.x
        self.y_odom = odom.pose.pose.position.y
        self.quaternion = odom.pose.pose.orientation
        (_,_,self.pos_z) = euler_from_quaternion([self.quaternion.x, self.quaternion.y, self.quaternion.z, self.quaternion.w])
    
    def callbackScan(self, msg):
        # Import scan and calculate angles
        scan = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(scan))
        # Filter out NaNs and infinities
        valid_indices = np.isfinite(scan)
        scan = scan[valid_indices]
        angles = angles[valid_indices]
        # Define angular limits for detection
        front_limit = np.pi / 15  # 45 degrees to each side
        left_limit = np.pi / 2  # 90 degrees
        right_limit = -np.pi / 2  # -90 degrees
        # Detect obstacles
        right_obstacle = np.any((scan < 0.35) & (angles < right_limit))
        left_obstacle = np.any((scan < 0.35) & (angles > left_limit))
        front_obstacle = np.any((scan < 0.44) & (np.abs(angles) < front_limit))
        # Update obstacle status  
        if front_obstacle:
            self.obstacle = 2
            print("Frontal")
        elif right_obstacle:
            self.obstacle = 1
            print("Derecha")
        elif left_obstacle:
            self.obstacle = -1
            print("Izquierda")
        else:
            self.obstacle = 0

    def handle_obstacle(self,obstacle):
        if obstacle == 1:
            # Obstacle detected on left or right, rotate away from it
            self.vel.angular.z = 1 * 0.15  # Adjust angular velocity for rotation away from obstacle
            self.vel.linear.x = 0.15
            self.vel_pub.publish(self.vel)
            print("derecha")
        elif self.obstacle == -1:
            self.vel.angular.z = -1 * 0.17  # Adjust angular velocity for rotation away from obstacle
            self.vel.linear.x = 0.15
            self.vel_pub.publish(self.vel)
            print("izquierda")
            
        elif self.obstacle == 2:
            # Rotate 90 degrees to the right
            self.vel.angular.z = -0.7  # -0.5  # Adjust angular velocity for rotation to the right
            self.vel.linear.x = 0.1
            self.vel_pub.publish(self.vel)
            print("front")
            ##here is the time for 1 sec
            rospy.sleep(1)

      
    def run(self):
        error_a=0.0
        error_d=0.0
        index=0
        while not rospy.is_shutdown():
            if self.first:
                self.current_time = rospy.get_time() 
                self.previous_time = rospy.get_time()
                self.first = False  
            else:
                self.current_time = rospy.get_time() 
                dt_1 = (self.current_time - self.previous_time) #get dt
                self.previous_time = self.current_time
                dt=0.09
                ##Instance class
                turnPID = ControlPID(dt,kp = 0.8)
                fowaPID = ControlPID(dt,kp = 0.2,kd = 0.005)
                ##send the goal here we update the goal in every iteration
                turnPID.points_callback(self.goal_x[index],self.goal_y[index])
                fowaPID.points_callback(self.goal_x[index],self.goal_y[index])
                #
                ###get the position
                ##OK
                error_a=turnPID.e_ang(self.pos_z,self.x_odom,self.y_odom)
                error_d=fowaPID.e_dis(self.x_odom,self.y_odom)
                #make a switch case to run foward or turn pid , almost donde

                if self.obstacle==0:
                    print("PID")
                    if abs(error_a)>0.3:
                        self.vel.angular.z=turnPID.get_pid(self.pos_z,error_a)
                        self.vel.linear.x=0.0
                        print("error_a",error_a)
                        self.vel_pub.publish(self.vel)
            
                    elif abs(error_a)<=0.3 and abs(error_d)>=0.1 :
                        self.vel.angular.z=0.0
                        self.vel.linear.x = fowaPID.get_pid_d(error_d)  
                        print("Aqui")
                        self.vel_pub.publish(self.vel)
               
                    elif abs(error_a)<=0.3 and abs(error_d)<=0.1 :
                        self.vel.angular.z= 0.0
                        self.vel.linear.x = 0.0
                        #index+=1
                        print("llegue")
                        self.vel_pub.publish(self.vel)
                
                else:
                    self.handle_obstacle(self.obstacle)
                    
                #get the pid_vel until the error is close to 0
                self.rate.sleep()

if __name__ == "__main__":
    try:
        node = Bugs()
        node.run()
    except rospy.ROSInterruptException:
        pass

        

    