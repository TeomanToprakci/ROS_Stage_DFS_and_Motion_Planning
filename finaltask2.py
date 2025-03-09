#!/usr/bin/env python3

import rospy
import yaml
import os
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        
        package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        config_path = os.path.join(package_path, 'task2', 'config', 'rParam.yaml')
        
        try:
            with open(config_path, 'r') as file:
                config = yaml.safe_load(file)
                self.SIDE_DISTANCE = config.get('R', 3.0)
        except Exception as e:
            rospy.logwarn(f"Failed to load rParam.yaml: {e}")
            self.SIDE_DISTANCE = 3.0
        rospy.loginfo(f"Using R value: {self.SIDE_DISTANCE}")
        
        self.velocity_publisher = rospy.Publisher('/robot_0/cmd_vel', Twist, queue_size=10)
        self.laser_subscriber = rospy.Subscriber('/robot_0/base_scan', LaserScan, self.laser_callback)
        self.odom_subscriber = rospy.Subscriber('/robot_0/odom', Odometry, self.odom_callback)
        
        self.current_x = -7.5
        self.current_y = -7.5
        self.current_yaw = 0.0
        self.obstacle_detected = False
        self.turn_direction = "right"
        self.state = "moving_up"
        self.rate = rospy.Rate(10)
        
        self.LINEAR_SPEED = 1.0
        self.ANGULAR_SPEED = 0.5
        self.OBSTACLE_DISTANCE = 1.5
        
        self.target_x = 7.3  
        self.target_y = 2
        
    def odom_callback(self, data):
        position = data.pose.pose.position
        self.current_x = position.x
        self.current_y = position.y
        
        orientation = data.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, self.current_yaw = euler_from_quaternion(orientation_list)
        
    def laser_callback(self, data):
        front_distance = min(data.ranges[85:95])
        if front_distance < self.OBSTACLE_DISTANCE:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False
            
    def move(self, linear_speed=0.0, angular_speed=0.0):
        vel_msg = Twist()
        vel_msg.linear.x = linear_speed
        vel_msg.angular.z = angular_speed
        self.velocity_publisher.publish(vel_msg)
        
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def turn_to_angle(self, target_angle):
        target_angle = self.normalize_angle(target_angle)
        rospy.sleep(0.5)
        
        while not rospy.is_shutdown():
            current_angle = self.normalize_angle(self.current_yaw)
            error = self.normalize_angle(target_angle - current_angle)
            
            if abs(error) < 0.02:
                break
                
            if abs(error) > 0.5:
                angular_speed = 0.4 if error > 0 else -0.4
            else:
                angular_speed = 0.2 if error > 0 else -0.2
                
            self.move(0.0, angular_speed)
            self.rate.sleep()
            
        self.move(0.0, 0.0)
        rospy.sleep(1.0)
        
    def check_target_reached(self):
        distance_to_target = math.sqrt((self.current_x - self.target_x)**2 + 
                                     (self.current_y - self.target_y)**2)
        if distance_to_target < 0.3:
            self.move(0.0, 0.0)
            rospy.loginfo("Reached final target position!")
            rospy.sleep(0.5)
            rospy.signal_shutdown("Target reached")
            return True
        return False

    def move_distance(self, distance):
        start_x = self.current_x
        start_y = self.current_y
        while not rospy.is_shutdown():
            if self.check_target_reached():
                return True
                
            current_distance = math.sqrt((self.current_x - start_x)**2 + 
                                       (self.current_y - start_y)**2)
            if current_distance >= distance:
                break
            self.move(self.LINEAR_SPEED, 0.0)
            self.rate.sleep()
        self.move(0.0, 0.0)
        rospy.sleep(0.5)
        return False
        
    def run(self):
        while not rospy.is_shutdown():
            if self.check_target_reached():
                return
                
            if self.state == "moving_up":
                if self.obstacle_detected:
                    self.move(0.0, 0.0)
                    rospy.sleep(0.5)
                    
                    if self.turn_direction == "right":
                        self.turn_to_angle(self.current_yaw - math.pi/2)
                        if self.move_distance(self.SIDE_DISTANCE):
                            return
                        self.turn_to_angle(self.current_yaw - math.pi/2)
                        self.turn_direction = "left"
                    else:
                        self.turn_to_angle(self.current_yaw + math.pi/2)
                        if self.move_distance(self.SIDE_DISTANCE):
                            return
                        self.turn_to_angle(self.current_yaw + math.pi/2)
                        self.turn_direction = "right"
                        
                    rospy.sleep(0.5)
                else:
                    self.move(self.LINEAR_SPEED, 0.0)
                    
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = RobotController()
        controller.run()
    except rospy.ROSInterruptException:
        pass

