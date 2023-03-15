import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Twist, Quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

import sys

import numpy as np
import math
import time

class MinimalSubscriber(Node):

    def __init__(self):
        self.Init = True
        self.Init_pos = Point()
        self.Init_pos.x = 0.0
        self.Init_pos.y = 0.0
        self.Init_ang = 0.0
        self.globalAng = 0.0
        self.globalPos = Point()
        self.l = 0.01 #meters
        self.i = 0
        self.K_goal = 0.1
        self.goal = np.array([[1.5,1.5,0.0], [0.0,1.4,1.4]]) # waypoints
        self.epsilon = 0.05 + (self.i*0.01)
        self.case = 0
        self.follow_cw_vector = np.array([-1.0,0.0])
        self.follow_ccw_vector = np.array([1.0,0.0])
        self.avoidvec = np.array([1.0 + self.globalPos.x,1.0 + self.globalPos.y])
        self.tau = 0.0
        self.error_goal = self.K_goal * np.array([(self.goal[0,self.i] - (self.globalPos.x + self.l*np.cos(self.globalAng))),(self.goal[1,self.i] - (self.globalPos.y + self.l*np.cos(self.globalAng)))]) # go to goal  vector
        self.error_goalscalar = np.sqrt(np.sum(np.square(self.error_goal))) # go to goal magnitude
        self.error_obs = self.K_goal * np.array([(self.avoidvec[0] - (self.globalPos.x + self.l*np.cos(self.globalAng))),(self.avoidvec[1] - (self.globalPos.y + self.l*np.cos(self.globalAng)))]) # avoid obs vector
        self.x_obs = self.globalPos.x - self.avoidvec[0]
        self.d_obs = 0.1 # desired distance from obstacle
        self.new_theta = 0.0
        self.avoid = False

        # Creates the node.
        super().__init__('go_to_goal')

        lidar_qos_profile = QoSProfile(
            reliability = QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history = QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            durability = QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            depth = 10
        )
        # Subscribe to scan node
        print('herefirst')
        self.currentpos = self.create_subscription(LaserScan, '/scan', self.obstacle, lidar_qos_profile)
        self.currentpos

        # Subscribe to Odom node
        self.odom = self.create_subscription(Odometry, '/odom', self.update_Odometry, 1)
        self.odom

        #Declare publisher for velocity
        self._vel_publish = self.create_publisher(Twist,'/cmd_vel', 10)

    def obstacle(self,msg):
        self.avoid = True
        ranges = msg.ranges
        ranges_arr = (np.asarray(ranges)).reshape(len(ranges))
        print(len(ranges))
        ranges_arr[60:120] = 0.0
        angles_arr = np.linspace(msg.angle_min, msg.angle_max,num = len(ranges))
        new_ranges_idx = (np.where((ranges_arr < 0.2) & (ranges_arr > 0.0)))[0]
        new_ranges = ranges_arr[np.where((ranges_arr < 0.3) & (ranges_arr > 0.0))]
        average_ranges = np.nanmean(new_ranges)
        average_angle = np.mean(angles_arr[new_ranges_idx])
        print(average_angle)
        average_angle = np.arctan2(np.sin(average_angle),np.cos(average_angle))
        print(average_angle)

        # In global frame
        self.avoidvec = np.array([(average_ranges*np.cos(average_angle))+self.globalPos.x,(average_ranges*np.sin(average_angle))+self.globalPos.y])
        #self.avoidvec = np.array([(average_ranges*np.cos(average_angle+self.globalAng))+self.globalPos.x,(average_ranges*np.sin(average_angle+self.globalAng))+self.globalPos.y])
        if np.isnan(self.avoidvec[0]) or np.isnan(self.avoidvec[1]) or (np.any(new_ranges) == False):
            self.avoid = False
            self.avoidvec = np.array([1.0 + self.globalPos.x,1.0 + self.globalPos.y]) # no obstacle, so set to far away value
        print('avoid',self.avoidvec)

    def goToGoal(self,i):
        if self.avoid == False or self.case == 0:
            self.x_obs = self.globalPos.x - self.avoidvec[0]
            self.error_goal = self.K_goal * np.array([(self.goal[0,self.i] - (self.globalPos.x + self.l*np.cos(self.globalAng))),(self.goal[1,self.i] - (self.globalPos.y + self.l*np.cos(self.globalAng)))]) # go to goal  vector
            # Determine which waypoint to use
            self.i = i

            # Check for obstacle
            angle1 = np.arccos((np.dot(self.error_goal,self.follow_cw_vector)) / ((np.linalg.norm(self.error_goal)) * (np.linalg.norm(self.follow_cw_vector))))
            angle1 = np.arctan2(np.sin(angle1),np.cos(angle1))
            angle2 = np.arccos((np.dot(self.error_goal,self.follow_ccw_vector)) / ((np.linalg.norm(self.error_goal)) * (np.linalg.norm(self.follow_ccw_vector))))
            angle2 = np.arctan2(np.sin(angle2),np.cos(angle2))
            if (self.avoid == True) and (angle1 > 0.0):
                move = Twist()
                move.linear.x = 0.0
                move.angular.z = 0.0
                self._vel_publish.publish(move)
                time.sleep(2)
                print('checkobs',self.x_obs,angle1)
                self.tau = self.globalPos.x #save current x position of robot
                # Go to follow clockwise
                self.case = 1
                return angle1
            elif (self.avoid == True) and (angle2 > 0.0):
                move = Twist()
                move.linear.x = 0.0
                move.angular.z = 0.0
                self._vel_publish.publish(move)
                time.sleep(2)
                print('ccw',self.x_obs,angle2)
                self.tau = self.globalPos.x #save current position of robot
                # Go to follow counterclockwise
                self.case = 2
                return angle2
            
            # Calculate velocity and apply control
            velocities = np.zeros([2])
            theta = -self.globalAng
            rotation_matrix = np.array([[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]])
            constant = np.array([[1, 0],[0, (1/self.l)]])
            velocities = np.matmul(np.matmul(constant,rotation_matrix),self.error_goal)
            print('gtg',velocities)

            # Move robot
            move = Twist()
            while abs(velocities[0]) < 0.03:
                velocities[0] = velocities[0] * 5

            if velocities[1] <= -1.5:
                velocities[1] = -1.4
            elif velocities[1] >= 1.5:
                velocities[1] = 1.4
            #velocities[1] = np.arctan2(np.cos(velocities[1]), np.sin(velocities[1]))
            print('gtg',velocities)
            move.linear.x = float(velocities[0])
            move.angular.z = float(velocities[1])
            self._vel_publish.publish(move)
            return

    def follow_cw(self,angle,i):
        angle = angle
        self.i = i
        self.x_obs = self.globalPos.x - self.avoidvec[0]
        self.error_goal = self.K_goal * np.array([(self.goal[0,self.i] - (self.globalPos.x + self.l*np.cos(self.globalAng))),(self.goal[1,self.i] - (self.globalPos.y + self.l*np.cos(self.globalAng)))]) # go to goal  vector
        self.error_obs = self.K_goal * np.array([(self.avoidvec[0] - (self.globalPos.x + self.l*np.cos(self.globalAng))),(self.avoidvec[1] - (self.globalPos.y + self.l*np.cos(self.globalAng)))]) # avoid obs vector
        if self.case == 1:
            # Move robot 45 degrees clockwise
            move = Twist()
            move.linear.x = 0.0
            move.angular.z = -1.2
            self._vel_publish.publish(move)

            # Move robot another 45 degrees clockwise
            move = Twist()
            move.linear.x = 0.0
            move.angular.z = -1.2
            self._vel_publish.publish(move)

            angle3 = np.arccos((np.dot(self.error_goal,self.error_obs)) / ((np.linalg.norm(self.error_goal)) * (np.linalg.norm(self.error_obs))))
            angle3 = np.arctan2(np.sin(angle3),np.cos(angle3))
            # Follow wall clockwise (go left)

            if ((angle3 > 0.0) and (abs(self.globalPos.x - self.goal[0,self.i]) < abs(self.tau - self.goal[0,self.i]))) or (self.x_obs >= (self.d_obs + self.epsilon)):
                # Go to goal
                self.case = 0
                return
            elif (self.x_obs < (self.d_obs - self.epsilon)):
                # Go to avoid obstacle
                self.case = 3
                return
            return

    def follow_ccw(self,angle,i):
        self.i = i
        angle = angle
        self.x_obs = self.globalPos.x - self.avoidvec[0]
        self.error_goal = self.K_goal * np.array([(self.goal[0,self.i] - (self.globalPos.x + self.l*np.cos(self.globalAng))),(self.goal[1,self.i] - (self.globalPos.y + self.l*np.cos(self.globalAng)))]) # go to goal  vector
        self.error_obs = self.K_goal * np.array([(self.avoidvec[0] - (self.globalPos.x + self.l*np.cos(self.globalAng))),(self.avoidvec[1] - (self.globalPos.y + self.l*np.cos(self.globalAng)))]) # avoid obs vector
        if self.case == 2:
            # Move robot 45 degrees cclockwise
            move = Twist()
            move.linear.x = 0.0
            move.angular.z = 1.2
            self._vel_publish.publish(move)

            # Move robot another 45 degrees cclockwise
            move = Twist()
            move.linear.x = 0.0
            move.angular.z = 1.2
            self._vel_publish.publish(move)

            angle4 = np.arccos((np.dot(self.error_goal,self.error_obs)) / ((np.linalg.norm(self.error_goal)) * (np.linalg.norm(self.error_obs))))
            angle4 = np.arctan2(np.sin(angle4),np.cos(angle4))
            # Follow wall counterclockwise
            if ((angle4 > 0.0) and (abs(self.globalPos.x - self.goal[0,self.i]) < abs(self.tau - self.goal[0,self.i]))) or (self.x_obs >= (self.d_obs + self.epsilon)):
                # Go to goal
                self.case = 0
                return
            elif (self.x_obs < (self.d_obs - self.epsilon)):
                # Go to avoid obstacle
                self.case = 3
                return
            return

    def avoidObstacle(self,i):
        self.i = i
        self.x_obs = self.globalPos.x - self.avoidvec[0]
        self.error_goal = self.K_goal * np.array([(self.goal[0,self.i] - (self.globalPos.x + self.l*np.cos(self.globalAng))),(self.goal[1,self.i] - (self.globalPos.y + self.l*np.cos(self.globalAng)))]) # go to goal  vector
        if self.case == 3:
            # Calculate velocity and apply control
            velocities = np.zeros([2])
            theta = -self.globalAng
            rotation_matrix = np.array([[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]])
            constant = np.array([[1, 0],[0, (1/self.l)]])
            velocities = np.matmul(np.matmul(constant,rotation_matrix), self.error_goal)
            print('avoidobs',velocities)

            # Move robot
            move = Twist()
            move.linear.x = 0.05
            move.angular.z = 0.0
            self._vel_publish.publish(move)

            # Check for obstacle
            angle5 = np.arccos((np.dot(self.error_goal,self.follow_cw_vector)) / ((np.linalg.norm(self.error_goal)) * (np.linalg.norm(self.follow_cw_vector))))
            angle5 = np.arctan2(np.sin(angle5),np.cos(angle5))
            angle6 = np.arccos((np.dot(self.error_goal,self.follow_ccw_vector)) / ((np.linalg.norm(self.error_goal)) * (np.linalg.norm(self.follow_ccw_vector))))
            angle6 = np.arctan2(np.sin(angle6),np.cos(angle6))
            if (abs(self.x_obs) > self.d_obs) and (angle5 > 0.0):
                self.tau = self.globalPos.x #save current x position of robot
                # Go to follow clockwise
                self.case = 1
                return angle5
            elif (abs(self.x_obs) > self.d_obs) and (angle6 > 0.0):
                self.tau = self.globalPos.x #save current position of robot
                # Go to follow counterclockwise
                self.case = 2
                return angle6
            return

    def update_Odometry(self,msg):
        print('odom',self.i,self.case,self.avoid,self.globalPos.x,self.globalPos.y)
        position = msg.pose.pose.position
        
        #Orientation uses the quaternion aprametrization.
        #To get the angular position along the z-axis, the following equation is required.
        q = msg.pose.pose.orientation
        orientation = np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

        if self.Init:
            #The initial data is stored to by subtracted to all the other values as we want to start at position (0,0) and orientation 0
            self.Init = False
            self.Init_ang = orientation
            self.globalAng = self.Init_ang
            Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        
            self.Init_pos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y
            self.Init_pos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y
            self.Init_pos.z = position.z
        Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        

        #We subtract the initial values
        self.globalPos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y - self.Init_pos.x
        self.globalPos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y - self.Init_pos.y
        self.globalAng = orientation - self.Init_ang

        self.globalAng = np.arctan2(np.sin(self.globalAng), np.cos(self.globalAng))
            
        if (abs(self.globalPos.x - self.goal[0,self.i]) < self.epsilon) and (abs(self.globalPos.y - self.goal[1,self.i]) < self.epsilon) and (self.i < 2):
            move = Twist()
            move.linear.x = 0.0
            move.angular.z = 0.0
            self._vel_publish.publish(move)
            print('stop')
            time.sleep(5)
            self.i = self.i + 1
            print(self.i)
        
        elif (abs(self.globalPos.x - self.goal[0,self.i]) < self.epsilon) and (abs(self.globalPos.y - self.goal[1,self.i]) < self.epsilon) and (self.i == 2):
            print(self.i)
            move = Twist()
            move.linear.x = 0.0
            move.angular.z = 0.0
            self._vel_publish.publish(move)
            sys.exit()

        if self.case == 0 or self.avoid == False:
            # Go to goal
            print('here')
            self.new_theta = self.goToGoal(self.i)
        elif self.case == 1:
            print('here2')
            # Follow wall clockwise
            self.follow_cw(self.new_theta,self.i)
        elif self.case == 2:
            print('here3')
            # Follow wall counterclockwise
            self.follow_ccw(self.new_theta,self.i)
        elif self.case == 3 and self.avoid == True:
            # Avoid obstacle
            #print('here3')
            self.new_theta = self.avoidObstacle(self.i)

def main():
    rclpy.init()
    print('Hello')
    subscriber = MinimalSubscriber() # Creates class object to be used
    rclpy.spin(subscriber)
    # while rclpy.ok():
    #     print('hey')
    #     for i in range (3):
    #         print('hey2')
    #         while (subscriber.globalPos.x < subscriber.goal[0,i]) and (subscriber.globalPos.y < subscriber.goal[1,i]):
    #             print('hey3')
    #             #rclpy.spin_once(subscriber) # Trigger callback processing
    #             if subscriber.case == 0:
    #                 # Go to goal
    #                 print('here')
    #                 new_theta = subscriber.goToGoal(i)
    #             elif subscriber.case == 1:
    #                 print('here2')
    #                 # Follow wall clockwise
    #                 subscriber.follow_cw(new_theta,i)
    #             elif subscriber.case == 2:
    #                 print('here3')
    #                 # Follow wall counterclockwise
    #                 subscriber.follow_ccw(new_theta,i)
    #             elif subscriber.case == 3:
    #                 # Avoid obstacle
    #                 #print('here3')
    #                 new_theta = subscriber.avoidObstacle(i)
    #         time.sleep(10)
    
    # Clean up and shutdown
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()