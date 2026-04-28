#!/usr/bin/env python3

# Import Dependencies
import rospy
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float64
from turtlesim.msg import Pose
import math


class TurtlesimStraightsAndTurns:
    def __init__(self):

        # Distance goal variables
        self.last_distance = 0.0
        self.start_distance = 0.0
        self.goal_distance = 0.0
        self.dist_goal_active = False
        self.forward_movement = True

        # Angle goal variables
        self.goal_angle = 0.0
        self.angle_goal_active = False
        self.current_theta = 0.0
        self.start_angle = 0.0
        self.rotate_anticlockwise = True

        # Position variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.position_goal_active = False
        self.position_stage = 0   # 0 = rotate, 1 = move straight

        # Initialize the node
        rospy.init_node('turtlesim_straights_and_turns_node', anonymous=True)

        # Subscribers
        rospy.Subscriber("/turtle_dist", Float64, self.distance_callback)
        rospy.Subscriber("/goal_angle", Float64, self.goal_angle_callback)
        rospy.Subscriber("/goal_distance", Float64, self.goal_distance_callback)
        rospy.Subscriber("/position_goal", Point, self.position_goal_callback)
        rospy.Subscriber("/turtle1/pose", Pose, self.pose_callback)

        # Publisher
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # Timer callback
        timer_period = 0.01
        rospy.Timer(rospy.Duration(timer_period), self.timer_callback)

        rospy.loginfo("Initialized node!")
        rospy.spin()

    def pose_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_theta = msg.theta

    def distance_callback(self, msg):
        self.last_distance = msg.data

    def goal_angle_callback(self, msg):
        self.goal_angle = abs(msg.data)
        self.start_angle = self.current_theta
        self.angle_goal_active = True

        self.rotate_anticlockwise = (msg.data >= 0)

        # only one goal at a time
        self.dist_goal_active = False
        self.position_goal_active = False

        rospy.loginfo("Received angle goal: %.3f radians", msg.data)

    def goal_distance_callback(self, msg):
        self.start_distance = self.last_distance
        self.goal_distance = abs(msg.data)
        self.dist_goal_active = True
        self.forward_movement = (msg.data >= 0)

        # only one goal at a time
        self.angle_goal_active = False
        self.position_goal_active = False

        rospy.loginfo("Received distance goal: %.3f", msg.data)

    def position_goal_callback(self, msg):
        self.goal_x = msg.x
        self.goal_y = msg.y
        self.position_goal_active = True
        self.position_stage = 0

        # only one goal at a time
        self.dist_goal_active = False
        self.angle_goal_active = False

        rospy.loginfo("Received position goal: x=%.3f, y=%.3f", msg.x, msg.y)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def distance_to_goal(self):
        return math.sqrt((self.goal_x - self.current_x) ** 2 + (self.goal_y - self.current_y) ** 2)

    def timer_callback(self, msg):
        vel_msg = Twist()

        # DISTANCE GOAL
        if self.dist_goal_active:
            travelled = abs(self.last_distance - self.start_distance)

            if travelled >= self.goal_distance - 0.01:
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = 0.0
                self.velocity_publisher.publish(vel_msg)

                self.dist_goal_active = False
                self.goal_distance = 0.0
                rospy.loginfo("Distance goal reached.")
            else:
                vel_msg.linear.x = 1.0 if self.forward_movement else -1.0
                vel_msg.angular.z = 0.0
                self.velocity_publisher.publish(vel_msg)

        # ANGLE GOAL
        elif self.angle_goal_active:
            turned_angle = abs(self.normalize_angle(self.current_theta - self.start_angle))

            if turned_angle >= self.goal_angle - 0.01:
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = 0.0
                self.velocity_publisher.publish(vel_msg)

                self.angle_goal_active = False
                self.goal_angle = 0.0
                rospy.loginfo("Angle goal reached.")
            else:
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = 1.0 if self.rotate_anticlockwise else -1.0
                self.velocity_publisher.publish(vel_msg)

        # POSITION GOAL
        elif self.position_goal_active:
            dx = self.goal_x - self.current_x
            dy = self.goal_y - self.current_y
            target_angle = math.atan2(dy, dx)
            angle_error = self.normalize_angle(target_angle - self.current_theta)
            dist_error = self.distance_to_goal()

            # Stage 0: rotate toward target
            if self.position_stage == 0:
                if abs(angle_error) < 0.03:
                    vel_msg.linear.x = 0.0
                    vel_msg.angular.z = 0.0
                    self.velocity_publisher.publish(vel_msg)
                    self.position_stage = 1
                    rospy.loginfo("Rotation toward position goal complete. Moving straight.")
                else:
                    vel_msg.linear.x = 0.0
                    vel_msg.angular.z = 1.0 if angle_error > 0 else -1.0
                    self.velocity_publisher.publish(vel_msg)

            # Stage 1: move straight
            elif self.position_stage == 1:
                if dist_error < 0.05:
                    vel_msg.linear.x = 0.0
                    vel_msg.angular.z = 0.0
                    self.velocity_publisher.publish(vel_msg)

                    self.position_goal_active = False
                    rospy.loginfo("Position goal reached.")
                else:
                    # if turtle drifts too much, rotate again
                    if abs(angle_error) > 0.1:
                        self.position_stage = 0
                    else:
                        vel_msg.linear.x = 1.0
                        vel_msg.angular.z = 0.0
                        self.velocity_publisher.publish(vel_msg)

        # NO ACTIVE GOAL
        else:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            self.velocity_publisher.publish(vel_msg)


if __name__ == '__main__':
    try:
        turtlesim_straights_and_turns_class_instance = TurtlesimStraightsAndTurns()
    except rospy.ROSInterruptException:
        pass