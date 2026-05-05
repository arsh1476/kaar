#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState

class Drive_Square:
    def __init__(self):

        self.cmd_msg = Twist2DStamped()

        rospy.init_node('drive_square_node', anonymous=True)

        # 🔴 CHANGE 'akandb' TO YOUR ROBOT NAME
        self.pub = rospy.Publisher('/mybota0024/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/akandb/fsm_node/mode', FSMState, self.fsm_callback, queue_size=1)

        self.running = False   # prevent multiple triggers

    def fsm_callback(self, msg):
        rospy.loginfo("State: %s", msg.state)

        if msg.state == "NORMAL_JOYSTICK_CONTROL":
            self.stop_robot()
            self.running = False

        elif msg.state == "LANE_FOLLOWING" and not self.running:
            self.running = True
            rospy.sleep(1)
            self.move_robot()

    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)

    def run(self):
        rospy.spin()

    def move_robot(self):

        # 🔧 TUNE THESE VALUES
        forward_speed = 0.3
        turn_speed = 4.0

        forward_time = 3.0   # adjust for ~1 meter
        turn_time = 1.0      # adjust for ~90 degrees

        for i in range(4):

            rospy.loginfo(f"Side {i+1}: Moving Forward")

            # Move forward
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = forward_speed
            self.cmd_msg.omega = 0.0
            self.pub.publish(self.cmd_msg)

            rospy.sleep(forward_time)

            # Stop
            self.stop_robot()
            rospy.sleep(0.5)

            rospy.loginfo(f"Side {i+1}: Turning")

            # Turn 90 degrees
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = 0.0
            self.cmd_msg.omega = turn_speed
            self.pub.publish(self.cmd_msg)

            rospy.sleep(turn_time)

            # Stop again
            self.stop_robot()
            rospy.sleep(0.5)

        rospy.loginfo("Square complete!")
        self.stop_robot()


if __name__ == '__main__':
    try:
        node = Drive_Square()
        node.run()
    except rospy.ROSInterruptException:
        pass