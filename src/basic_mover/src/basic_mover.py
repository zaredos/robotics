#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import Point, Pose, Twist
from tf.transformations import euler_from_quaternion

# BasicMover
class BasicMover:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.my_odom_sub = rospy.Subscriber('my_odom', Point, self.my_odom_cb)
        # Current heading of the robot.
        self.cur_yaw = 0
        self.cur_dist = 0 # had to add this variable for move_forward method

    def my_odom_cb(self, msg):
        """Callback function for `self.my_odom_sub`."""
        if msg is not None:
            self.cur_yaw = msg.y
            self.cur_dist = msg.x

    def turn_to_heading(self, target_yaw):
        """
        Turns the robot to heading `target_yaw`.
        """
        rate = rospy.Rate(10)
        twist = Twist()
        speed = 0.25

        while not rospy.is_shutdown():
            yaw_diff = target_yaw - self.cur_yaw
            yaw_diff = (yaw_diff + math.pi) % (2*math.pi) - math.pi # keep differece in range [-pi,pi]
            print (str(self.cur_yaw) + " " + str(yaw_diff))

            if abs(yaw_diff) <= speed / 2: # if target is 90, speed = 0.25 then it will stop when it is closest (if at 89.8 then next will be 90.05 which is within 0.25/2)
                break

            twist.angular.z = speed * yaw_diff / abs(yaw_diff) # basically is just 0.1 or -0.1 depending on the negativity of the difference (direction)
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        
        twist.angular.z = 0
        self.cmd_vel_pub.publish(twist)
        
    def move_forward(self, target_dist):
        """Moves the robot forward by `target_dist`."""
        rate = rospy.Rate(10)  # Control loop at 10Hz
        twist = Twist()
        start_dist = self.cur_dist
        speed = 0.1

        while not rospy.is_shutdown():
            print(str(start_dist) + " " + str(self.cur_dist))
            if self.cur_dist - start_dist >= target_dist:
                break

            twist.linear.x = speed
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        
        # stop
        twist.linear.x = 0
        self.cmd_vel_pub.publish(twist)

    def out_and_back(self, target_dist):
        """
        This function:
        1. moves the robot forward by `target_dist`;
        2. turns the robot by 180 degrees; and
        3. moves the robot forward by `target_dist`.
        """
        self.move_forward(target_dist)
        self.turn_to_heading((self.cur_yaw + math.pi) % (2*math.pi))
        self.move_forward(target_dist)

    def draw_square(self, side_length):
        """
        This function moves the robot in a square with `side_length` meter sides.
        """ 
        for i in range(4):
            self.move_forward(side_length)
            self.turn_to_heading((self.cur_yaw + math.pi/2) % (2*math.pi))

    def move_in_a_circle(self, r):
        """Moves the robot in a circle with radius `r`"""
        rate = rospy.Rate(10)
        time = 20  # time to complete a circle

        angular_velocity = (2 * math.pi) / time
        linear_velocity = angular_velocity * r

        twist = Twist()
        twist.linear.x = linear_velocity 
        twist.angular.z = angular_velocity  

        while not rospy.is_shutdown():
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        
    def rotate_in_place(self):
        """For debugging."""
        twist = Twist()
        
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            twist.angular.z = 0.1
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('basic_mover')
    #BasicMover().out_and_back(1)
    #BasicMover().draw_square(1)
    BasicMover().move_in_a_circle(1)
    # BasicMover().rotate_in_place()
