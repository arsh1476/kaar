#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import AprilTagDetectionArray


class Target_Follower:
    def __init__(self):

        rospy.init_node('target_follower_node', anonymous=True)
        rospy.on_shutdown(self.clean_shutdown)

        # Robot name
        self.robot_name = "mybota002410"

        # Publisher for robot movement
        self.cmd_vel_pub = rospy.Publisher(
            "/" + self.robot_name + "/car_cmd_switch_node/cmd",
            Twist2DStamped,
            queue_size=1
        )

        # Subscriber for AprilTag detections
        rospy.Subscriber(
            "/" + self.robot_name + "/apriltag_detector_node/detections",
            AprilTagDetectionArray,
            self.tag_callback,
            queue_size=1
        )

        # Control parameters
        self.seek_speed = 1.5          # speed used when searching for object
        self.kp = 4.0                  # proportional control gain
        self.dead_zone = 0.03          # if object is almost centered, stop rotating
        self.min_omega = 0.4           # minimum rotation to overcome friction
        self.max_omega = 2.0           # maximum safe rotation speed

        rospy.loginfo("Target follower node started for robot: %s", self.robot_name)
        rospy.spin()

    def tag_callback(self, msg):
        self.move_robot(msg.detections)

    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

    def stop_robot(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

    def publish_velocity(self, v, omega):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = v
        cmd_msg.omega = omega
        self.cmd_vel_pub.publish(cmd_msg)

    def move_robot(self, detections):

        # Feature 1: Seek object
        # If no AprilTag is detected, rotate in-place slowly.
        if len(detections) == 0:
            rospy.loginfo("No object detected. Seeking object...")
            self.publish_velocity(0.0, self.seek_speed)
            return

        # Use the first detected AprilTag
        tag = detections[0]

        x = tag.transform.translation.x
        y = tag.transform.translation.y
        z = tag.transform.translation.z
        tag_id = tag.tag_id

        rospy.loginfo("Detected tag ID: %d | x: %.3f y: %.3f z: %.3f", tag_id, x, y, z)

        # Feature 2: Look at object
        # We use x as the left/right error.
        # The goal is to make x close to 0.
        error = x

        # If the tag is centered, stop rotating.
        if abs(error) < self.dead_zone:
            rospy.loginfo("Object is centered. Stopping rotation.")
            self.publish_velocity(0.0, 0.0)
            return

        # Proportional controller
        omega = -self.kp * error

        # Apply minimum omega so robot can overcome friction
        if omega > 0:
            omega = max(omega, self.min_omega)
        else:
            omega = min(omega, -self.min_omega)

        # Limit maximum omega for safety
        if omega > self.max_omega:
            omega = self.max_omega
        elif omega < -self.max_omega:
            omega = -self.max_omega

        rospy.loginfo("Looking at object. Error: %.3f | Omega: %.3f", error, omega)

        # No forward/backward movement for this task
        self.publish_velocity(0.0, omega)


if __name__ == '__main__':
    try:
        target_follower = Target_Follower()
    except rospy.ROSInterruptException:
        pass
