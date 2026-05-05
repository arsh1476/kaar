#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState


class Drive_Square:
    def __init__(self):
        # Initialize message
        self.cmd_msg = Twist2DStamped()

        # Initialize ROS node
        rospy.init_node('drive_square_node', anonymous=True)

        # IMPORTANT: use your robot name here
        self.vehicle_name = "mybota002410"

        # Publisher
        self.pub = rospy.Publisher(
            f'/{self.vehicle_name}/car_cmd_switch_node/cmd',
            Twist2DStamped,
            queue_size=1
        )

        # Subscriber
        rospy.Subscriber(
            f'/{self.vehicle_name}/fsm_node/mode',
            FSMState,
            self.fsm_callback,
            queue_size=1
        )

        self.is_running = False

    # FSM callback
    def fsm_callback(self, msg):
        rospy.loginfo(f"FSM State: {msg.state}")

        if msg.state == "NORMAL_JOYSTICK_CONTROL":
            self.is_running = False
            self.stop_robot()

        elif msg.state == "LANE_FOLLOWING" and not self.is_running:
            self.is_running = True
            rospy.sleep(1)
            self.move_robot()

    # Stop robot
    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Robot Stopped")

    # Move in square
    def move_robot(self):

        for i in range(4):
            rospy.loginfo(f"Side {i+1}")

            # Move forward
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = 0.3      # forward speed
            self.cmd_msg.omega = 0.0
            self.pub.publish(self.cmd_msg)
            rospy.loginfo("Moving forward")
            rospy.sleep(2.8)          # adjust for ~1 meter

            # Stop briefly
            self.stop_robot()
            rospy.sleep(0.5)

            # Turn 90 degrees
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = 0.0
            self.cmd_msg.omega = 2.2  # turning speed
            self.pub.publish(self.cmd_msg)
            rospy.loginfo("Turning 90 degrees")
            rospy.sleep(1.2)          # adjust angle

            # Stop again
            self.stop_robot()
            rospy.sleep(0.5)

        # Final stop
        self.stop_robot()
        rospy.loginfo("Finished square path")

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = Drive_Square()
        node.run()
    except rospy.ROSInterruptException:
        pass