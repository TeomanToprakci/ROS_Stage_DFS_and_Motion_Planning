#!/usr/bin/env python3

# calculate_distance.py
import rospy
from beginner_tutorials.srv import euclidean_distance, euclidean_distanceResponse
from math import sqrt

def handle_distance_request(req):
    # Euclidean mesafe hesaplaması
    distance = sqrt((req.x2 - req.x1)**2 + (req.y2 - req.y1)**2)
    return euclidean_distanceResponse(distance)

def calculate_distance_server():
    rospy.init_node('calculate_distance_server')
    rospy.Service('/calculate_distance', euclidean_distance, handle_distance_request)
    rospy.loginfo("Ready to calculate Euclidean distance.")
    rospy.spin()

if __name__ == "__main__":
    calculate_distance_server()
