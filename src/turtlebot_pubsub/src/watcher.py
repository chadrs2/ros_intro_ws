#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose

class Watcher():
    def __init__(self):
        rospy.init_node('watcher')
        self.pose_sub = rospy.Subscriber('/turtle1/pose', Pose, callback=self.pose_callback)

    def pose_callback(self, msg):
        print(
            "\n\nX:", msg.x,
            "\nY:", msg.y,
            "\ntheta:", msg.theta,
            "\nv:", msg.linear_velocity,
            "\nomega:", msg.angular_velocity,
        )
        return

if __name__ == '__main__':
    try:
        watcher_subscriber = Watcher()
        rospy.spin()  # Keep the program from exiting
    except rospy.ROSInterruptException:
        pass