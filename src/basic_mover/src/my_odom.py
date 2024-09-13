#!/usr/bin/env python3

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point 
from tf.transformations import euler_from_quaternion

class MyOdom:
    def __init__(self):
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_cb)
        self.my_odom_pub = rospy.Publisher('my_odom', Point, queue_size=1)
        self.old_pose = None 
        self.dist = 0.0
        self.yaw = 0.0
                
    def odom_cb(self, msg):
        """Callback function for `odom_sub`."""
        cur_pose = msg.pose.pose
        self.update_dist(cur_pose) 
        self.update_yaw(cur_pose.orientation)
        self.publish_data()

    def update_dist(self, cur_pose):
        if self.old_pose is not None:
            old_pos = self.old_pose.position
            cur_pos = cur_pose.position

            dx = cur_pos.x - old_pos.x
            dy = cur_pos.y - old_pos.y
            dz = cur_pos.z - old_pos.z
            
            self.dist += math.sqrt(dx**2 + dy**2 + dz**2) # distance formula
        self.old_pose = cur_pose


    def update_yaw(self, cur_orientation):
        """
        Helper to `odom_cb`.
        Updates `self.yaw` to current heading of robot.
        """
        quaternion = (
            cur_orientation.x,
            cur_orientation.y,
            cur_orientation.z,
            cur_orientation.w
        )
        
        euler = euler_from_quaternion(quaternion) # uses the method we were given
        self.yaw = euler[2]

    def publish_data(self):
        """
        Publish `self.dist` and `self.yaw` on the `my_odom` topic.
        """
        # The `Point` object should be used simply as a data container for
        # `self.dist` and `self.yaw` so we can publish it on `my_odom`.
        point_msg = Point()
        point_msg.x = self.dist
        point_msg.y = self.yaw
        point_msg.z = 0
        self.my_odom_pub.publish(point_msg)      

if __name__ == '__main__':
    rospy.init_node('my_odom')
    MyOdom()
    rospy.spin()
