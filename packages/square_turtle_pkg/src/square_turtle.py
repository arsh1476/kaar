#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def move_turtle_square():
    rospy.init_node('turtlesim_square_node', anonymous=True)

    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    vel = Twist()

    rospy.loginfo("Drawing square...")

    rospy.sleep(2)  # allow connection

    while not rospy.is_shutdown():

        for i in range(4):

            # 🔹 Move forward
            vel.linear.x = 2.0
            vel.angular.z = 0.0

            start = rospy.Time.now().to_sec()
            while rospy.Time.now().to_sec() - start < 2:
                pub.publish(vel)
                rate.sleep()

            # 🔹 Stop
            vel.linear.x = 0.0
            pub.publish(vel)
            rospy.sleep(1)

            # 🔹 Turn
            vel.angular.z = 1.57

            start = rospy.Time.now().to_sec()
            while rospy.Time.now().to_sec() - start < 1:
                pub.publish(vel)
                rate.sleep()

            # 🔹 Stop
            vel.angular.z = 0.0
            pub.publish(vel)
            rospy.sleep(1)

        rospy.loginfo("Square complete!")
        break   # remove this if you want continuous squares

    rospy.spin()


if __name__ == '__main__':
    try:
        move_turtle_square()
    except rospy.ROSInterruptException:
        pass
