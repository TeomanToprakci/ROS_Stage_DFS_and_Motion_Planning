#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import tf
from tf.transformations import euler_from_quaternion
import random

class GridExplorer:
    def __init__(self):
        rospy.init_node('grid_explorer', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        self.cell_size = 5.0
        self.grid_size = 4
        self.origin_x = -10.0
        self.origin_y = -10.0
        
        self.current_x = -7.5
        self.current_y = -7.5
        self.current_theta = 0
        
        self.obstacles = {4, 7, 16}
        
        self.rewards = self.place_rewards()
        self.collected_rewards = 0
        rospy.loginfo(f"Rewards placed at cells: {sorted(list(self.rewards))}")
        
        self.visited_cells = set()
        self.current_cell = 1
        self.rate = rospy.Rate(10)
        
        self.linear_speed = 0.5
        self.angular_speed = 0.5
        self.position_tolerance = 0.1
        self.angle_tolerance = 0.05

    def place_rewards(self):
        # Get all possible cells (1-16) excluding obstacles
        available_cells = set(range(1, 17)) - self.obstacles
        reward_cells = set(random.sample(list(available_cells), 3))
        return reward_cells

    def odom_callback(self, msg):
        # Get position
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Get orientation
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, self.current_theta = euler_from_quaternion(orientation_list)

    def get_cell_center(self, cell_number):
        # Convert cell number to row and column (for simple grid pattern)
        row = (cell_number - 1) // self.grid_size  # Integer division for row (0-3)
        col = (cell_number - 1) % self.grid_size   # Remainder for column (0-3)
        
        center_x = self.origin_x + (col * self.cell_size) + (self.cell_size / 2)
        center_y = self.origin_y + (row * self.cell_size) + (self.cell_size / 2)
        
        return center_x, center_y

    def get_current_cell(self):
        col = int((self.current_x - self.origin_x) // self.cell_size)
        row = int((self.current_y - self.origin_y) // self.cell_size)
        
        # Calculate cell number using simple grid pattern
        return row * self.grid_size + col + 1

    def move_to_cell(self, target_cell):
        rospy.loginfo(f"Attempting to move to cell {target_cell}")
        
        # Single obstacle check
        if target_cell in self.obstacles:
            rospy.loginfo(f"Cannot move to cell {target_cell} - obstacle")
            return False
            
        target_x, target_y = self.get_cell_center(target_cell)
        current_x, current_y = self.get_cell_center(self.current_cell)
        
        dx = target_x - current_x
        dy = target_y - current_y
        
        if abs(dy) > abs(dx):
            desired_angle = math.pi/2 if dy > 0 else -math.pi/2
        else:
            desired_angle = 0 if dx > 0 else math.pi
            
        self.rotate_to_angle(desired_angle)
        
        while not rospy.is_shutdown():
            distance = math.sqrt((target_x - self.current_x)**2 + (target_y - self.current_y)**2)
            
            if distance < 0.5:
                self.stop_robot()
                self.current_theta = desired_angle
                self.current_cell = target_cell
                rospy.loginfo(f"Reached target cell {target_cell}")
                return True
            
            vel_msg = Twist()
            vel_msg.linear.x = self.linear_speed
            self.cmd_vel_pub.publish(vel_msg)
            self.rate.sleep()

    def rotate_to_angle(self, target_angle):
        while not rospy.is_shutdown():
            angle_diff = target_angle - self.current_theta
            
            while angle_diff > math.pi: angle_diff -= 2 * math.pi
            while angle_diff < -math.pi: angle_diff += 2 * math.pi
            
            if abs(angle_diff) < self.angle_tolerance:
                self.stop_robot()
                self.current_theta = target_angle
                return
                
            vel_msg = Twist()
            vel_msg.angular.z = min(self.angular_speed, abs(angle_diff)) * (1 if angle_diff > 0 else -1)
            self.cmd_vel_pub.publish(vel_msg)
            self.rate.sleep()

    def stop_robot(self):
        vel_msg = Twist()
        self.cmd_vel_pub.publish(vel_msg)

    def can_move_up(self, current_cell):
        target_cell = current_cell + self.grid_size
        return (target_cell <= 16 and  # Check if within grid
                target_cell not in self.obstacles and  # Check for obstacles
                current_cell + self.grid_size <= 16 and  # Check if not in top row
                target_cell not in self.visited_cells)  # Don't revisit cells

    def can_move_right(self, current_cell):
        target_cell = current_cell + 1
        rospy.loginfo(f"Checking right movement from {current_cell} to {target_cell}")
        
        # Check if target is an obstacle or already visited
        if target_cell in self.obstacles or target_cell in self.visited_cells:
            rospy.loginfo(f"Cannot move right - Cell {target_cell} is obstacle or visited")
            return False
            
        # Check if at right edge
        if current_cell % self.grid_size == 0:
            rospy.loginfo("Cannot move right - At rightmost column")
            return False
            
        # Check if within grid
        if target_cell > 16:
            rospy.loginfo("Cannot move right - Would exceed grid bounds")
            return False
            
        rospy.loginfo(f"Can move right to cell {target_cell}")
        return True

    def can_move_down(self, current_cell):
        target_cell = current_cell - self.grid_size
        return (target_cell > 0 and  
                target_cell not in self.obstacles and  
                current_cell > self.grid_size and  
                target_cell not in self.visited_cells)  

    def can_move_left(self, current_cell):
        target_cell = current_cell - 1
        return (target_cell > 0 and
                target_cell not in self.obstacles and
                current_cell % self.grid_size != 1 and
                target_cell not in self.visited_cells)

    def explore(self):
        current_cell = 1
        self.visited_cells.add(current_cell)
        path = [current_cell]
        
        # Check if starting cell has a reward
        if current_cell in self.rewards:
            rospy.loginfo(f"Collected reward at cell {current_cell}")
            self.rewards.remove(current_cell)
            self.collected_rewards += 1
            if self.collected_rewards >= 2:
                rospy.loginfo("Collected 2 rewards! Exploration complete!")
                return
        
        while not rospy.is_shutdown():
            moved = False
            rospy.loginfo(f"Current position: Cell {current_cell}")
            
            # Try to move to unvisited cells in priority order
            if self.can_move_up(current_cell):
                next_cell = current_cell + self.grid_size
                rospy.loginfo(f"Trying to move up from cell {current_cell} to cell {next_cell}")
                if self.move_to_cell(next_cell):
                    current_cell = next_cell
                    self.visited_cells.add(current_cell)
                    path.append(current_cell)
                    moved = True
                    
                    # Check if new cell has a reward
                    if current_cell in self.rewards:
                        rospy.loginfo(f"Collected reward at cell {current_cell}")
                        self.rewards.remove(current_cell)
                        self.collected_rewards += 1
                        if self.collected_rewards >= 2:
                            rospy.loginfo("Collected 2 rewards! Exploration complete!")
                            return
                            
                    rospy.loginfo(f"Successfully moved to cell {current_cell}")
                    continue
            
            if not moved and self.can_move_right(current_cell):
                next_cell = current_cell + 1
                rospy.loginfo(f"Trying to move right from cell {current_cell} to cell {next_cell}")
                if self.move_to_cell(next_cell):
                    current_cell = next_cell
                    self.visited_cells.add(current_cell)
                    path.append(current_cell)
                    moved = True
                    
                    if current_cell in self.rewards:
                        rospy.loginfo(f"Collected reward at cell {current_cell}")
                        self.rewards.remove(current_cell)
                        self.collected_rewards += 1
                        if self.collected_rewards >= 2:
                            rospy.loginfo("Collected 2 rewards! Exploration complete!")
                            return
                            
                    rospy.loginfo(f"Successfully moved to cell {current_cell}")
                    continue
            
            if not moved and self.can_move_down(current_cell):
                next_cell = current_cell - self.grid_size
                rospy.loginfo(f"Trying to move down from cell {current_cell} to cell {next_cell}")
                if self.move_to_cell(next_cell):
                    current_cell = next_cell
                    self.visited_cells.add(current_cell)
                    path.append(current_cell)
                    moved = True
                    
                    if current_cell in self.rewards:
                        rospy.loginfo(f"Collected reward at cell {current_cell}")
                        self.rewards.remove(current_cell)
                        self.collected_rewards += 1
                        if self.collected_rewards >= 2:
                            rospy.loginfo("Collected 2 rewards! Exploration complete!")
                            return
                            
                    rospy.loginfo(f"Successfully moved to cell {current_cell}")
                    continue
            
            if not moved and self.can_move_left(current_cell):
                next_cell = current_cell - 1
                rospy.loginfo(f"Trying to move left from cell {current_cell} to cell {next_cell}")
                if self.move_to_cell(next_cell):
                    current_cell = next_cell
                    self.visited_cells.add(current_cell)
                    path.append(current_cell)
                    moved = True
                    
                    # Check if new cell has a reward
                    if current_cell in self.rewards:
                        rospy.loginfo(f"Collected reward at cell {current_cell}")
                        self.rewards.remove(current_cell)
                        self.collected_rewards += 1
                        if self.collected_rewards >= 2:
                            rospy.loginfo("Collected 2 rewards! Exploration complete!")
                            return
                            
                    rospy.loginfo(f"Successfully moved to cell {current_cell}")
                    continue
            
            # If no unvisited cells available, backtrack
            if not moved:
                # Check if there are any unvisited cells left
                all_cells = set(range(1, 17)) - self.obstacles
                unvisited = all_cells - self.visited_cells
                
                if unvisited and len(path) > 1:  # If there are still unvisited cells and we can backtrack
                    # Remove current cell from path and go back one step
                    path.pop()  # Remove current cell
                    next_cell = path[-1]  # Get previous cell
                    
                    rospy.loginfo(f"Backtracking from cell {current_cell} to cell {next_cell}")
                    if self.move_to_cell(next_cell):
                        current_cell = next_cell
                        moved = True
                        continue
                else:
                    rospy.loginfo("Exploration complete!")
                    rospy.loginfo(f"Visited cells: {sorted(list(self.visited_cells))}")
                    break

if __name__ == '__main__':
    try:
        explorer = GridExplorer()
        explorer.explore()
    except rospy.ROSInterruptException:
        pass

