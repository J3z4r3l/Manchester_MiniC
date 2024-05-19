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
        a -= 2*np.pi if a > np.pi else -2*np.pi if a < -np.pi else 0
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
        self.goal_x=[]
        self.goal_y=[]
        self.turnPID = ControlPID(0.09,kp = 0.8)
        self.fowaPID = ControlPID(0.09,kp = 0.2,kd = 0.005)
        self.obstacle=2
        self.avoid_count = 0
        self.rotate_start_time =0.0
        
        
                
        
    def odom_callback(self, odom):
        self.x_odom = odom.pose.pose.position.x
        self.y_odom = odom.pose.pose.position.y
        self.quaternion = odom.pose.pose.orientation
        (_,_,self.pos_z) = euler_from_quaternion([self.quaternion.x, self.quaternion.y, self.quaternion.z, self.quaternion.w])

    def calcular_pendiente_y_generar_arreglos(self,punto_a, punto_b):
        x1, y1 = punto_a
        x2, y2 = punto_b
        pendiente = (y2 - y1) / (x2 - x1)

        # Generar arreglo de valores x
        Ax = min(x1, x2)
        Bx = max(x1, x2)
        arreglo_x = []
        x = Ax
        while x < Bx:
            if x>0:
                arreglo_x.append(x)
            x += 1
        arreglo_x.append(Bx)

        # Calcular arreglo de valores y
        arreglo_y = [pendiente * x for x in arreglo_x]

        return arreglo_x, arreglo_y    
   
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
        right_obstacle_up = np.any((scan >= 0.35) & (scan < 0.5) & (angles < right_limit))
        left_obstacle = np.any((scan < 0.35) & (angles > left_limit))
        lef_obstacle_up = np.any((scan >= 0.35) & (scan < 0.5) & (angles > left_limit))
        front_obstacle = np.any((scan < 0.5) & (np.abs(angles) < front_limit))
        # Update obstacle status  
        if front_obstacle:
            self.obstacle = 2
            print("Frontal")
        elif right_obstacle:
            self.obstacle = 1
            print("Derecha")
        elif right_obstacle_up:
            self.obstacle = 3
            print("Der_lim")
        
        elif left_obstacle:
            self.obstacle = -1
            print("Izquierda")
        elif lef_obstacle_up:
            self.obstacle = 4
            print("Izq_lim")
        
        else:
            self.obstacle = 0
        
    def rotate_and_move(self):
        self.vel.angular.z = -0.5  # Ajust the angular velocity for a right turn
        self.vel.linear.x = 0.05    # Adjust the linear velocity for forward movement
        self.vel_pub.publish(self.vel)
        rospy.sleep(0.5)  # Move for 0.5 seconds
        self.vel.angular.z = 0.0
        self.vel.linear.x = 0.0
        self.vel_pub.publish(self.vel)


    def handle_obstacle(self, obstacle, goal_x):
        distance_to_goal = abs(goal_x)
        if (obstacle == 4 or obstacle == -1) and distance_to_goal <= 0.5:
            # Rotate and move forward for 0.5 seconds
            self.rotate_and_move()
        else:
            if obstacle == 1:
                self.vel.angular.z = 0.15
                self.vel.linear.x = 0.15
                self.vel_pub.publish(self.vel)
                print("Seguir pared a la derecha")
            elif obstacle == 3:
                self.vel.angular.z = -0.1
                self.vel.linear.x = 0.1
                self.vel_pub.publish(self.vel)
                print("Seguir limite derecho")
            elif obstacle == -1:
                self.vel.angular.z = -1*0.15
                self.vel.linear.x = 0.15
                self.vel_pub.publish(self.vel)
                print("Seguir pared a la izquierda")
            elif obstacle == 4:
                self.vel.angular.z = 0.25
                self.vel.linear.x = 0.1
                self.vel_pub.publish(self.vel)
                print("Seguir limite izquierdo")
            elif obstacle == 2:
                self.vel.angular.z = -0.7
                self.vel.linear.x = 0.05
                self.vel_pub.publish(self.vel)
                print("Obstaculo frontal")


      
    def run(self):
        error_a=0.0
        error_d=0.0
        index=0
        punto_a=[0,0]
        punto_b=[4.5,-2]
        self.goal_x,self.goal_y= self.calcular_pendiente_y_generar_arreglos(punto_a,punto_b)
        print(self.goal_x,self.goal_y)
        
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
                turnPID = ControlPID(dt,kp = 0.8,  kd=0.01)
                fowaPID = ControlPID(dt,kp = 0.2,kd = 0.005)
                ##send the goal here we update the goal in every iteration
                if index<len(self.goal_x):
                    if self.x_odom > self.goal_x[index]:
                        index += 1
                        print("Increment index due to position in x")
                    ###get the position
                    turnPID.points_callback(self.goal_x[index],self.goal_y[index])
                    fowaPID.points_callback(self.goal_x[index],self.goal_y[index])
                    error_a=turnPID.e_ang(self.pos_z,self.x_odom,self.y_odom)
                    error_d=fowaPID.e_dis(self.x_odom,self.y_odom)
                    print(error_d)
                    print(index,self.goal_x[index],self.goal_y[index])

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
                            index+=1
                            print("llegue")
                            self.vel_pub.publish(self.vel)

                    else:
                        self.handle_obstacle(self.obstacle,error_d)

                    #get the pid_vel until the error is close to 0
                else: 
                    print("end")
                self.rate.sleep()

if __name__ == "__main__":
    try:
        node = Bugs()
        node.run()
    except rospy.ROSInterruptException:
        pass

        

    
