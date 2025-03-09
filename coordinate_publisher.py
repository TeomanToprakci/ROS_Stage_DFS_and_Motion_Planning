#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelStates
from beginner_tutorials.msg import coordinates

def callback(data):
    try:
        index = data.name.index("tb3_2")
        x = data.pose[index].position.x
        y = data.pose[index].position.y

        msg = coordinates()
        msg.x = x
        msg.y = y
        pub.publish(msg)
        
    except ValueError as e:
        rospy.logwarn_throttle(5, "Robot tb3_2 not found in model states")
    except Exception as e:
        rospy.logerr(f"Error in callback: {str(e)}")

if __name__ == "__main__":
    rospy.init_node("tb3_2_coordinates_publisher")
    pub = rospy.Publisher("/tb3_2/coordinates", coordinates, queue_size=10)
    rospy.Subscriber("/gazebo/model_states", ModelStates, callback)
    rospy.spin()
