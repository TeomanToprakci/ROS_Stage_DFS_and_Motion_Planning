#!/usr/bin/env python3

#chmod u+x ~/catkin_ws/src/beginner_tutorials/final_project/task2/finaltask2stepB.py

import rospy
from std_srvs.srv import Empty

class ResetRobotPosition:
    def __init__(self):
        rospy.init_node('reset_robot_position', anonymous=True)
        self.reset_service_name = '/reset_positions'
        
        rospy.loginfo("Waiting for the /reset_positions service...")
        rospy.wait_for_service(self.reset_service_name)
        self.reset_service = rospy.ServiceProxy(self.reset_service_name, Empty)
        rospy.loginfo("/reset_positions service is ready.")
    
    def reset(self):
        try:
            rospy.loginfo("Calling /reset_positions service to reset robot position...")
            self.reset_service()
            rospy.loginfo("Robot position successfully reset.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    try:
        resetter = ResetRobotPosition()
        resetter.reset()
    except rospy.ROSInterruptException:
        pass

