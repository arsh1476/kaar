#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float64
from turtlesim.msg import Pose
import math

class TurtlesimController:

    def __init__(self):

        # -------- State Variables --------

        # Current pose of the turtle (updated from /turtle1/pose)
        self.pose = Pose()

        # -------- Distance Control --------
        self.goal_distance = 0          # Target distance to travel
        self.start_pose = Pose()        # Starting position when goal received
        self.dist_goal_active = False   # Flag to activate distance control
        self.forward = True             # Direction of movement

        # -------- Angle Control --------
        self.goal_angle = 0             # Target rotation angle
        self.angle_goal_active = False  # Flag to activate angle control

        # -------- Position Control --------
        self.goal_position = Point()        # Target (x, y) position
        self.position_goal_active = False   # Flag to activate position control

        # -------- Initialize ROS Node --------
        rospy.init_node('turtlesim_controller_node', anonymous=True)

        # -------- Subscribers --------

        # Subscribe to turtle pose (position + orientation)
        rospy.Subscriber("/turtle1/pose", Pose, self.pose_callback)

        # Subscribe to distance goal
        rospy.Subscriber("/goal_distance", Float64, self.distance_callback)

        # Subscribe to angle goal
        rospy.Subscriber("/goal_angle", Float64, self.angle_callback)

        # Subscribe to position goal (x, y)
        rospy.Subscriber("/goal_position", Point, self.position_callback)

        # -------- Publisher --------

        # Publish velocity commands to move the turtle
        self.vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # -------- Control Loop --------

        # Timer runs control loop every 0.01 seconds
        rospy.Timer(rospy.Duration(0.01), self.control_loop)

        rospy.loginfo("Turtlesim Controller Node Started")
        rospy.spin()

    # -------- Callback Functions --------

    def pose_callback(self, msg):
        # Update current turtle pose
        self.pose = msg

    def distance_callback(self, msg):
        # Set distance goal
        self.goal_distance = abs(msg.data)

        # Determine direction (forward or backward)
        self.forward = msg.data > 0

        # Save starting position
        self.start_pose = self.pose

        # Activate distance control and disable others
        self.dist_goal_active = True
        self.angle_goal_active = False
        self.position_goal_active = False

    def angle_callback(self, msg):
        # Set angle goal
        self.goal_angle = msg.data

        # Activate angle control and disable others
        self.angle_goal_active = True
        self.dist_goal_active = False
        self.position_goal_active = False

    def position_callback(self, msg):
        # Set position goal (x, y)
        self.goal_position = msg

        # Activate position control and disable others
        self.position_goal_active = True
        self.dist_goal_active = False
        self.angle_goal_active = False

    # -------- Main Control Logic --------

    def control_loop(self, event):

        cmd = Twist()  # Velocity command

        # -------- POSITION CONTROL --------
        if self.position_goal_active:

            # Compute difference between current and target position
            dx = self.goal_position.x - self.pose.x
            dy = self.goal_position.y - self.pose.y

            # Distance to goal
            distance = math.sqrt(dx**2 + dy**2)

            # Angle to goal
            target_angle = math.atan2(dy, dx)

            # Difference between current and desired angle
            angle_diff = self.normalize_angle(target_angle - self.pose.theta)

            # Step 1: Rotate towards goal
            if abs(angle_diff) > 0.05:
                cmd.angular.z = 1.5 if angle_diff > 0 else -1.5

            # Step 2: Move forward
            elif distance > 0.1:
                cmd.linear.x = 1.5

            # Step 3: Stop when goal reached
            else:
                self.position_goal_active = False

        # -------- DISTANCE CONTROL --------
        elif self.dist_goal_active:

            # Calculate distance travelled from starting point
            dx = self.pose.x - self.start_pose.x
            dy = self.pose.y - self.start_pose.y
            travelled = math.sqrt(dx**2 + dy**2)

            # Stop if goal reached
            if travelled >= self.goal_distance:
                self.dist_goal_active = False
            else:
                cmd.linear.x = 1.5 if self.forward else -1.5

        # -------- ANGLE CONTROL --------
        elif self.angle_goal_active:

            # Compute shortest angular difference
            angle_diff = self.normalize_angle(self.goal_angle - self.pose.theta)

            # Stop if rotation is complete
            if abs(angle_diff) < 0.05:
                self.angle_goal_active = False
            else:
                cmd.angular.z = 1.5 if angle_diff > 0 else -1.5

        # Publish velocity command
        self.vel_pub.publish(cmd)

    # -------- Helper Function --------

    def normalize_angle(self, angle):
        """
        Normalize angle to range [-pi, pi]
        This avoids wraparound issues in rotation
        """
        return math.atan2(math.sin(angle), math.cos(angle))


# -------- Main Execution --------

if __name__ == '__main__':
    try:
        TurtlesimController()
    except rospy.ROSInterruptException:
        pass