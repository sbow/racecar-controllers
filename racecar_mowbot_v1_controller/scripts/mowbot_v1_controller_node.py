#!/usr/bin/python
#

import rospy
from ackermann_msgs.msg import AckermannDriveStamped


class MowbotControllerNode:
    def __init__(self):
        # subscribe to incomming Ackermann drive commands
        rospy.Subscriber("ackermann_cmd_input", AckermannDriveStamped,
                         self.ackermann_cmd_input_callback)

        # publisher for the safe Ackermann drive command
        self.cmd_pub = rospy.Publisher("ackermann_cmd", AckermannDriveStamped, queue_size=10)

        # setup 40 hz timer
        rospy.Timer( rospy.Duration( 1.0/1.0 ), self.timer_callback)

    def ackermann_cmd_input_callback(self, msg):
        # republish the input as output (not exactly "safe")
        self.cmd_pub.publish(msg)

    def timer_callback(self):
        # 40 hz publisher
        rospy.loginfo("Mowbot online!")


if __name__ == "__main__":
    rospy.init_node("mowbot_controller")

    node = MowbotControllerNode()

    rospy.spin()