#!/usr/bin/env python3

import rospy 
from std_msgs.msg import Float64
from turtlesim.msg import Pose
import math

class DistanceReader:
    def __init__(self):
        
        rospy.init_node('turtlesim_distance_node', anonymous=True)

        rospy.Subscriber("/turtle1/pose", Pose, self.callback)

        self.distance_publisher = rospy.Publisher('/turtle_dist', Float64, queue_size=10)

        rospy.loginfo("Initialized node!")

        # 🔹 Variables to store previous position
        self.prev_x = None
        self.prev_y = None

        # 🔹 Total distance
        self.total_distance = 0.0

        rospy.spin()

    def callback(self, msg):
        rospy.loginfo("Turtle Position: %f %f", msg.x, msg.y)

        # 🔹 First time: just store position
        if self.prev_x is None:
            self.prev_x = msg.x
            self.prev_y = msg.y
            return

        # 🔹 Calculate distance using formula
        dx = msg.x - self.prev_x
        dy = msg.y - self.prev_y

        distance = math.sqrt(dx**2 + dy**2)

        # 🔹 Add to total distance
        self.total_distance += distance

        # 🔹 Update previous position
        self.prev_x = msg.x
        self.prev_y = msg.y

        # 🔹 Publish distance
        dist_msg = Float64()
        dist_msg.data = self.total_distance

        self.distance_publisher.publish(dist_msg)

        rospy.loginfo("Total Distance: %f", self.total_distance)


if __name__ == '__main__': 
    try: 
        DistanceReader()
    except rospy.ROSInterruptException: 
        pass