#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import Point, Pose, Twist
from tf.transformations import euler_from_quaternion

MAX_TIME_SECONDS=60
SPEED=0.3
RUN_LOOP_HZ=10

# BasicMover
class Move:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.twist = Twist()
        
    def run(self):
        rate = rospy.Rate(RUN_LOOP_HZ)

        while not rospy.is_shutdown():
            if True:
                self.twist.linear.x=SPEED/2.0
            elif False:
                pass
            else:
                return
            self.cmd_vel_pub.publish(self.twist)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('basic_mover')
    Move().run()
