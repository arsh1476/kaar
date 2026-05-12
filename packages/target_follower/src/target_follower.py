#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, AprilTagDetectionArray


class Target_Follower:
    def __init__(self):

        # Initialize ROS node
        rospy.init_node('target_follower_node', anonymous=True)

        # Ensure robot stops safely on shutdown
        rospy.on_shutdown(self.clean_shutdown)

        # Name of the Duckiebot
        self.robot_name = "mybota002410"

        # ===== SPEED VALUES =====
        self.search_speed = 0.7          # Speed used while searching for tag

        self.max_turn_speed = 0.6        # Maximum turning speed
        self.min_turn_speed = 0.25       # (Not used, but reserved for smoother control)

        self.forward_speed = 0.35        # Forward speed (tuned for rough surface)
        self.backward_speed = -0.22      # Reverse speed when too close

        # ===== DISTANCE CONTROL =====
        self.goal_distance = 0.35        # Desired distance from the target (meters)
        self.distance_threshold = 0.06   # Allowed error range to avoid jitter

        # ===== CENTER CONTROL =====
        self.center_threshold = 0.04     # Allowed horizontal error (for alignment)

        # Time tracking for lost tag handling
        self.last_seen_time = rospy.Time.now()
        self.lost_timeout = rospy.Duration(0.5)  # Time before switching to search mode

        # Publisher: sends velocity commands to robot
        self.cmd_vel_pub = rospy.Publisher(
            f'/{self.robot_name}/car_cmd_switch_node/cmd',
            Twist2DStamped,
            queue_size=1
        )

        # Subscriber: receives AprilTag detections
        rospy.Subscriber(
            f'/{self.robot_name}/apriltag_detector_node/detections',
            AprilTagDetectionArray,
            self.tag_callback,
            queue_size=1
        )

        rospy.loginfo("Target follower with forward/backward control started")

        # Keep node running
        rospy.spin()

    def tag_callback(self, msg):
        # Called every time new AprilTag detection data is received
        self.move_robot(msg.detections)

    def move_robot(self, detections):

        # Create a velocity command message
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()

        # Default: no movement
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0

        # ===== CASE 1: NO TAG DETECTED =====
        if len(detections) == 0:
            time_since_seen = rospy.Time.now() - self.last_seen_time

            # If tag was just seen recently → stop (avoid sudden movement)
            if time_since_seen < self.lost_timeout:
                rospy.loginfo("Tag briefly lost - stop")
                cmd_msg.v = 0.0
                cmd_msg.omega = 0.0
            else:
                # If tag lost for longer → rotate to search
                rospy.loginfo("No tag - searching")
                cmd_msg.v = 0.0
                cmd_msg.omega = self.search_speed

            # Publish command and exit
            self.cmd_vel_pub.publish(cmd_msg)
            return

        # ===== CASE 2: TAG DETECTED =====

        # Update last seen time
        self.last_seen_time = rospy.Time.now()

        # Take first detected tag (main target)
        tag = detections[0]

        # Extract position of tag relative to robot
        x = tag.transform.translation.x
        y = tag.transform.translation.y   # left/right offset
        z = tag.transform.translation.z   # distance from robot

        rospy.loginfo("Tag position | x: %.3f y: %.3f z: %.3f", x, y, z)

        # Calculate errors
        horizontal_error = y                  # alignment error
        distance_error = z - self.goal_distance  # distance from desired position

        # ===== ANGULAR CONTROL (LEFT/RIGHT ALIGNMENT) =====
        if horizontal_error > self.center_threshold:
            rospy.loginfo("Tag left - turning left")
            cmd_msg.omega = self.max_turn_speed

        elif horizontal_error < -self.center_threshold:
            rospy.loginfo("Tag right - turning right")
            cmd_msg.omega = -self.max_turn_speed

        else:
            # Tag is centered → no rotation needed
            rospy.loginfo("Tag centered")
            cmd_msg.omega = 0.0

        # ===== LINEAR CONTROL (FORWARD/BACKWARD DISTANCE) =====
        if distance_error > self.distance_threshold:
            rospy.loginfo("Tag far - moving forward")
            cmd_msg.v = self.forward_speed

        elif distance_error < -self.distance_threshold:
            rospy.loginfo("Tag close - moving backward")
            cmd_msg.v = self.backward_speed

        else:
            # Within acceptable distance → stop moving forward/backward
            rospy.loginfo("Correct distance - no forward/backward")
            cmd_msg.v = 0.0

        # Send movement command to robot
        self.cmd_vel_pub.publish(cmd_msg)

    def clean_shutdown(self):
        # Called when ROS node shuts down
        rospy.loginfo("Shutdown - stopping robot")
        self.stop_robot()

    def stop_robot(self):
        # Publishes zero velocity to safely stop the robot
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)


if __name__ == '__main__':
    try:
        Target_Follower()
    except rospy.ROSInterruptException:
        pass
