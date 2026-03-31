#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def move_turtle_square():
    rospy.init_node('turtlesim_square_node', anonymous=True)

    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    rospy.loginfo("Drawing squares continuously...")
    rospy.sleep(2)

    while not rospy.is_shutdown():

        rate = rospy.Rate(1)

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 2.0
        pub.publish(cmd_vel_msg)

        rate.sleep()

        cmd_vel_msg = Twist()
        cmd_vel_msg.angular.z = 2.0
        pub.publish(cmd_vel_msg)

        rate.sleep()

        rospy.loginfo("One square done, continuing...")

    # never reaches here unless stopped
    vel.linear.x = 0.0
    vel.angular.z = 0.0
    pub.publish(vel)


if __name__ == '__main__':
    try:
        move_turtle_square()
    except rospy.ROSInterruptException:
        pass
