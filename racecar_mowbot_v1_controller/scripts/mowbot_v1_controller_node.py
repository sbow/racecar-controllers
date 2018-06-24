#!/usr/bin/python
#

import rospy
from ackermann_msgs.msg import AckermannDriveStamped


class MowbotControllerNode:
    mowbot_msg = AckermannDriveStamped()

    def __init__(self):
        # subscribe to incomming Ackermann drive commands
        rospy.Subscriber("ackermann_cmd_input", AckermannDriveStamped,
                         self.ackermann_cmd_input_callback)

        # publisher for the safe Ackermann drive command - Remapped in Launch to Vesc/Ackermann cmd multiplexr
        self.cmd_pub = rospy.Publisher("mowbot_ackermann_cmd", AckermannDriveStamped, queue_size=10)

        # setup 50 hz timer - Main VESC / Motor Servo Command Loop
        rospy.Timer(rospy.Duration(1.0 / 50.0), self.vesc_timer_callback)

        # setup a display timer 1 hz
        rospy.Timer(rospy.Duration(1.0 / 1.0), self.log_timer_callback)

        # read VESC parameters from config/default.yaml:
        # TODO: Doesnt work
        # self.force_scale_x = rospy.get_param("force_scale_x")

    def ackermann_cmd_input_callback(self, msg):
        # republish the input as output (not exactly "safe")
        self.cmd_pub.publish(msg)

    def log_timer_callback(self, msg):
        # 1 hz publisher
        # for messaging
        rospy.loginfo("stayin alive")

    def vesc_timer_callback(self, msg):
        # 50 hz publisher
        # update VESC throttle / servo command
        self.mowbot_commander_simple_demo(self)
        self.cmd_pub.publish(MowbotControllerNode.mowbot_msg)
        rospy.loginfo("Mowbot servo / motor command sent")

    def mowbot_commander_simple_demo(self):
        MowbotControllerNode.mowbot_msg.header.stamp = rospy.Time.now()
        MowbotControllerNode.mowbot_msg.header.frame_id = "mowbot_ackermann_cmd"
        # steering angle - in radians - TODO: rationalize w max value
        MowbotControllerNode.mowbot_msg.drive.steering_angle = 0.2
        # steering angle velocity - in radians per second
        MowbotControllerNode.mowbot_msg.drive.steering_angle_velocity = 0.5
        # speed - drive - m/s
        MowbotControllerNode.mowbot_msg.drive.speed = 0.25
        # acceleration - drive - m/s/s
        MowbotControllerNode.mowbot_msg.drive.acceleration = 0.50
        # jerk - drive - m/s/s/s
        MowbotControllerNode.mowbot_msg.drive.jerk = 2.00


if __name__ == "__main__":
    rospy.init_node("mowbot_controller")

    node = MowbotControllerNode()

    rospy.spin()
