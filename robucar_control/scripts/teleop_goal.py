#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy

class TeleopGoal():
    def __init__(self):
        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.

        rospy.Subscriber("/joy_teleop/joy", Joy, self.callback)
        self.pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.pub_view = rospy.Publisher('/move_base_simple/goal_view', PoseStamped, queue_size=10)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.buttons)
        right_left = data.axes[0]
        front_back = data.axes[1]
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "/base_footprint"
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x = front_back *3;
        pose_msg.pose.position.y = right_left *3;
        pose_msg.pose.orientation.w = 1.0;
        self.pub_view.publish(pose_msg)
        if data.buttons[1]:
            self.pub.publish(pose_msg)


if __name__ == '__main__':
    rospy.init_node('teleop_goal', anonymous=True)
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        teleop_goal = TeleopGoal()
    except rospy.ROSInterruptException: pass