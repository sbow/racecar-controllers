#!/usr/bin/python
#

import rospy

from ackermann_msgs.msg import AckermannDriveStamped
from mowbot_general_control.mowbot_v1_main import MowbotV1Main
from mowbot_laser_handler.mowbot_laser_handler import RplidarLaserHandler
from geometry_msgs.msg import PoseStamped


class MowbotControllerNode:
    mowbot_msg = AckermannDriveStamped()
    mb_gnrl = MowbotV1Main()
    rplidar_obj = []
    pose_obj = []


    def __init__(self):
        # subscribe to incomming Ackermann drive commands
        rospy.Subscriber("ackermann_cmd_input", AckermannDriveStamped,
                         self.ackermann_cmd_input_callback)

        # publisher for the safe Ackermann drive command - Remapped in Launch to Vesc/Ackermann cmd multiplexr
        self.cmd_pub = rospy.Publisher("mowbot_ackermann_cmd", AckermannDriveStamped, queue_size=10)

        # setup RPLidar handler - mowbot_laser_handler RplidarLaserHandler obj
        # RplidarLaserHandler( RawScanTopic, ROI_Ang_min_rad, ROI_Ang_max_rad, TF_Publish_Frame, OutputScanTopic )
        self.rplidar_obj = RplidarLaserHandler('scan', -360/360.0*3.14,
                                               360/360.0*3.14,
                                               'laser', 'mowbot/scan')

        # setup 50 hz timer - Main VESC / Motor Servo Command Loop
        rospy.Timer(rospy.Duration(1.0 / 50.0), self.vesc_timer_callback)

        # setup a display timer 0.25 hz
        rospy.Timer(rospy.Duration(1.0 / 0.25), self.log_timer_callback)

        # setup pose callback - Pose update from hector_slam - rostopic hz /slam_out_pose ~ 13.5 hz Jetson TX1
        rospy.Subscriber("slam_out_pose", PoseStamped, self.hector_pose_callback)

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
        rospy.loginfo(self.pose_obj)

    def vesc_timer_callback(self, msg):
        # 50 hz publisher
        # update VESC throttle / servo command
        # self.mowbot_commander_simple_demo()
        self.update_mb_vesc_cmd()
        self.cmd_pub.publish(MowbotControllerNode.mowbot_msg)
        # rospy.loginfo("Mowbot servo / motor command sent")

    def hector_pose_callback(self, msg):
        # hector_slam /slam_out_pose callback - averages 13.5 Hz
        # TODO: Fill out pose callback, populate self.pose_obj
        self.pose_obj = msg
        MowbotControllerNode.mb_gnrl.pose_slam(self.pose_obj)

    def update_mb_vesc_cmd(self):
        MowbotControllerNode.mb_gnrl.get_command()
        MowbotControllerNode.mowbot_msg.header.stamp = rospy.Time.now()
        MowbotControllerNode.mowbot_msg.header.frame_id = "mowbot_ackermann_cmd"
        MowbotControllerNode.mowbot_msg.drive.steering_angle = MowbotControllerNode.mb_gnrl.cmd_steering_angle
        MowbotControllerNode.mowbot_msg.drive.steering_angle_velocity = MowbotControllerNode.mb_gnrl.cmd_steering_angle_velocity
        MowbotControllerNode.mowbot_msg.drive.speed = MowbotControllerNode.mb_gnrl.cmd_speed
        MowbotControllerNode.mowbot_msg.drive.acceleration = MowbotControllerNode.mb_gnrl.cmd_acceleration
        MowbotControllerNode.mowbot_msg.drive.jerk = MowbotControllerNode.mb_gnrl.cmd_jerk

    def mowbot_commander_simple_demo(self):
        MowbotControllerNode.mowbot_msg.header.stamp = rospy.Time.now()
        MowbotControllerNode.mowbot_msg.header.frame_id = "mowbot_ackermann_cmd"
        # steering angle - in radians - TODO: rationalize w max value
        MowbotControllerNode.mowbot_msg.drive.steering_angle = 0.2
        # steering angle velocity - in radians per second
        MowbotControllerNode.mowbot_msg.drive.steering_angle_velocity = 0.25
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
