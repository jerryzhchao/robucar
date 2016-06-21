#!/usr/bin/env python
import rospy
from math import pi
from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import Joy

class TeleopAckermannJoy():
    def __init__(self):
        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.

        rospy.Subscriber("joy", Joy, self.callback)
        self.pub = rospy.Publisher('cmd_ackermann', AckermannDrive, queue_size=10)
        rate = rospy.Rate(10.0)
        self.linear_speed = 0.0
        self.front_steering = 0.0

        while rospy.is_shutdown() is False:
            ackermann_msg = AckermannDrive()
            ackermann_msg.speed = self.linear_speed
            ackermann_msg.steering_angle = self.front_steering
            self.pub.publish(ackermann_msg)
            rate.sleep()

    def callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.buttons)
        self.linear_speed = data.axes[1]        #left up-down stick
        self.front_steering = data.axes[3]*pi/10      #right left-right stick


if __name__ == '__main__':
    rospy.init_node('teleop_ackermann_joy', anonymous=False)
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        teleop_ackermann_joy = TeleopAckermannJoy()
    except rospy.ROSInterruptException: pass
