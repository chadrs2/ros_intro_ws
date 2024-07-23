#!/usr/bin/env python3

import math
import rospy
from geometry_msgs.msg import Twist, Vector3


class Mover():
    def __init__(self):
        rospy.init_node('mover')
        self.vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    def move_forward(self):
        vel_msg = Twist(
            Vector3(1,0,0),
            Vector3(0,0,0)
        )
        self.vel_pub.publish(vel_msg)

    def move_turn(self,deg=90.):
        vel_msg = Twist(
                Vector3(0,0,0),
                Vector3(0,0,deg*math.pi/180.)
            )
        self.vel_pub.publish(vel_msg)

if __name__ == '__main__':
    mover_publisher = Mover()
    rate = rospy.Rate(1)
    frwd_cntr = 0
    while not rospy.is_shutdown():
        if frwd_cntr < 5:
            mover_publisher.move_forward()
        else:
            mover_publisher.move_turn()
            frwd_cntr = 0
        frwd_cntr += 1
        rate.sleep()
    # try:
    #     mover_publisher = Mover()
    #     rospy.spin()  # Keep the program from exiting
    # except rospy.ROSInterruptException:
    #     pass