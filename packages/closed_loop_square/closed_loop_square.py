#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState, WheelEncoderStamped


class ClosedLoopSquare:
    def __init__(self):
        rospy.init_node("closed_loop_square_node", anonymous=True)

        self.robot_name = "mybota002410"

        self.cmd_topic = "/" + self.robot_name + "/car_cmd_switch_node/cmd"
        self.left_encoder_topic = "/" + self.robot_name + "/left_wheel_encoder_node/tick"
        self.right_encoder_topic = "/" + self.robot_name + "/right_wheel_encoder_node/tick"
        self.fsm_topic = "/" + self.robot_name + "/fsm_node/mode"

        self.cmd_pub = rospy.Publisher(
            self.cmd_topic,
            Twist2DStamped,
            queue_size=1
        )

        rospy.Subscriber(self.left_encoder_topic, WheelEncoderStamped, self.left_encoder_callback)
        rospy.Subscriber(self.right_encoder_topic, WheelEncoderStamped, self.right_encoder_callback)
        rospy.Subscriber(self.fsm_topic, FSMState, self.fsm_callback)

        self.left_ticks = 0
        self.right_ticks = 0

        self.started = False
        self.running = False

        # IMPORTANT:
        # Change these two values after your testing.
        # Measure encoder ticks for 1 metre and 90 degrees on your robot.
        self.TICKS_PER_METER = 850
        self.TICKS_PER_90_DEGREE = 300

        rospy.on_shutdown(self.stop_robot)

    def left_encoder_callback(self, msg):
        self.left_ticks = msg.data

    def right_encoder_callback(self, msg):
        self.right_ticks = msg.data

    def fsm_callback(self, msg):
        if msg.state == "LANE_FOLLOWING" and not self.started:
            self.started = True
            self.running = True
            rospy.loginfo("Starting closed loop square...")
            self.draw_square()
            self.running = False
            rospy.loginfo("Closed loop square completed.")

    def publish_cmd(self, linear_speed, angular_speed):
        cmd = Twist2DStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.v = linear_speed
        cmd.omega = angular_speed
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        cmd = Twist2DStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.v = 0.0
        cmd.omega = 0.0

        for i in range(10):
            self.cmd_pub.publish(cmd)
            rospy.sleep(0.05)

    def move_straight(self, distance_meter, speed):
        rospy.loginfo("Moving straight: distance = %.2f m, speed = %.2f", distance_meter, speed)

        start_left = self.left_ticks
        start_right = self.right_ticks

        target_ticks = abs(distance_meter) * self.TICKS_PER_METER

        if distance_meter >= 0:
            move_speed = abs(speed)
        else:
            move_speed = -abs(speed)

        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            left_change = abs(self.left_ticks - start_left)
            right_change = abs(self.right_ticks - start_right)

            average_ticks = (left_change + right_change) / 2.0

            if average_ticks >= target_ticks:
                break

            self.publish_cmd(move_speed, 0.0)
            rate.sleep()

        self.stop_robot()
        rospy.loginfo("Straight movement completed.")

    def rotate_in_place(self, angle_degree, angular_speed):
        rospy.loginfo("Rotating: angle = %.2f degrees, angular speed = %.2f", angle_degree, angular_speed)

        start_left = self.left_ticks
        start_right = self.right_ticks

        target_ticks = abs(angle_degree) / 90.0 * self.TICKS_PER_90_DEGREE

        if angle_degree >= 0:
            turn_speed = abs(angular_speed)
        else:
            turn_speed = -abs(angular_speed)

        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            left_change = abs(self.left_ticks - start_left)
            right_change = abs(self.right_ticks - start_right)

            average_ticks = (left_change + right_change) / 2.0

            if average_ticks >= target_ticks:
                break

            self.publish_cmd(0.0, turn_speed)
            rate.sleep()

        self.stop_robot()
        rospy.loginfo("Rotation completed.")

    def draw_square(self):
        rospy.loginfo("Drawing 1 metre closed loop square...")

        for side in range(4):
            rospy.loginfo("Square side %d", side + 1)

            self.move_straight(1.0, 0.25)
            self.rotate_in_place(90.0, 2.5)

        self.stop_robot()
        rospy.loginfo("Finished drawing square.")


if __name__ == "__main__":
    try:
        node = ClosedLoopSquare()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
