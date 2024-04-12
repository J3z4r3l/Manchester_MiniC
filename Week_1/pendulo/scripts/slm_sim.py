#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState





class PendulumSimulator:
     def __init__(self):
          #Initialize
          rospy.init_node("pendulum_sim")
          
          #Parameters
          self.k = 0.01  # Damping coefficient
          self.m = 0.75  # Mass
          self.l=0.36
          self.g = 9.8  # Gravity
          self.tao=0.0
          self.a = self.l/2
          self.j = (4/3) *self.m *self.a**2
          self.x1=0.0
          self.x2=0.0
          self.loop_rate = rospy.Rate(rospy.get_param("~node_rate",100))
          self.first=True
          ##Publishers 
          self.pub_custom_joint_state = rospy.Publisher('/joint_states', JointState, queue_size=10)         
    
    #wrap to pi function
     def wrap_to_Pi(self,theta):
           result = np.fmod((theta + np.pi),(2 * np.pi))
           if(result < 0):
                result += 2 * np.pi
           return result - np.pi
     
     ##Ponemos la posicion de nuestros nodos 
     def init_joints(self):
          self.msg=JointState()
          self.msg.header.frame_id= "link1"
          self.msg.header.stamp= rospy.Time.now()
          self.msg.name.extend(["joint2"])
          self.msg.position.extend([0.0])
          self.msg.velocity.extend([0.0])
          self.msg.effort.extend([0.0])
          

     
     def simulate(self):
          self.init_joints()

          while not rospy.is_shutdown():
              current_time = rospy.Time.now().to_sec()  # Get current time

              if self.first:
                  self.previous_time = current_time
                  self.first = False
              else:
                  dt = current_time - self.previous_time  # Calculate time difference
                  self.previous_time = current_time
                  self.x1 += self.x2 * dt
                  self.x2_dot = (1 / self.j) * (self.tao - self.m * self.g * self.a * np.cos(self.x1) - self.k * self.x2)
                  self.x2 += self.x2_dot*dt

                  # Update message
                  self.msg.header.stamp = rospy.Time.now()
                  self.msg.position[0] = self.wrap_to_Pi(self.x1)
                  self.msg.velocity[0] = self.x2
                  self.pub_custom_joint_state.publish(self.msg)
                  self.loop_rate.sleep()

if __name__=='__main__':
    pendulum=PendulumSimulator()
    try:
         pendulum.simulate()
                    
    except rospy.ROSInterruptException:
        pass #Initialise and Setup node