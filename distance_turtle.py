#!/usr/bin/env python3

# Import required ROS libraries and message types
import rospy
from turtlesim.msg import Pose      # For receiving turtle position
from std_msgs.msg import Float64    # For publishing distance

# Function to initialize and run the node
def distance_node():
    global pub  # Declare publisher as global so it can be used in callback

    # Initialize ROS node
    rospy.init_node('turtle_distance_node', anonymous=True)

    # Create publisher to send total distance on /turtle_dist topic
    pub = rospy.Publisher('/turtle_dist', Float64, queue_size=10)

    # Create subscriber to receive turtle position from /turtle1/pose
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)

    # Log message to indicate node has started
    rospy.loginfo("Distance node started...")

    # Keep node running and listening for incoming messages
    rospy.spin()


# Callback function that runs every time a new pose message is received
def pose_callback(msg):
    global prev_x, prev_y, total_distance

    # If this is the first message, just store initial position
    if prev_x is None and prev_y is None:
        prev_x = msg.x
        prev_y = msg.y
        return

    # Calculate change in position
    dx = msg.x - prev_x
    dy = msg.y - prev_y

    # Compute distance between two points using Euclidean formula
    distance = (dx**2 + dy**2) ** 0.5

    # Add this distance to total distance
    total_distance += distance

    # Update previous position for next calculation
    prev_x = msg.x
    prev_y = msg.y

    # Create and publish the total distance message
    dist_msg = Float64()
    dist_msg.data = total_distance
    pub.publish(dist_msg)


# Initialize global variables
prev_x = None        # Previous x position
prev_y = None        # Previous y position
total_distance = 0.0 # Total distance traveled


# Main function check
if __name__ == '__main__':
    try:
        distance_node()  # Start the node
    except rospy.ROSInterruptException:
        pass
