#!/usr/bin/python
#

import rospy

from ackermann_msgs.msg import AckermannDriveStamped
from mowbot_general_control.mowbot_v1_main import MowbotV1Main
from mowbot_laser_handler.mowbot_laser_handler import RplidarLaserHandler
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
import mowbot_general_control.lininterp as lininterp

class MowbotControllerNode:
    mowbot_msg = AckermannDriveStamped()
    mb_gnrl = MowbotV1Main()
    rplidar_obj = []
    pose_obj = PoseStamped()
    map_obj = OccupancyGrid()
    cntr_c_prev = [64] # previous cntrl column
    str_cmnd = 0 # steering command from kmeans / bluelines perception
    str_cmnd_prev = 0 # previous steering command - for filtering


    def __init__(self):
        # subscribe to incomming Ackermann drive commands
        rospy.Subscriber("ackermann_cmd_input", AckermannDriveStamped,
                         self.ackermann_cmd_input_callback)

        # subscribe to incoming blue lane data from jetson nano
        rospy.Subscriber("KmeansMsg", String, self.blue_line_callback)

        # publisher for the safe Ackermann drive command - Remapped in Launch to Vesc/Ackermann cmd multiplexr
        self.cmd_pub = rospy.Publisher("mowbot_ackermann_cmd", AckermannDriveStamped, queue_size=10)

        # setup RPLidar handler - mowbot_laser_handler RplidarLaserHandler obj
        # RplidarLaserHandler( RawScanTopic, ROI_Ang_min_rad, ROI_Ang_max_rad, TF_Publish_Frame, OutputScanTopic )
        # TODO: this is actually not currently used for anything... just using SLAM. In the future might be usefull for
        # TODO: near-time avoidance of obstacles.
        self.rplidar_obj = RplidarLaserHandler('scan', -360/360.0*3.14, # class defined in mowbot_laser_handler.py
                                               360/360.0*3.14,
                                               'laser', 'mowbot/scan')

        # setup 50 hz timer - Main VESC / Motor Servo Command Loop
        rospy.Timer(rospy.Duration(1.0 / 50.0), self.vesc_timer_callback)

        # setup a display timer 0.25 hz
        rospy.Timer(rospy.Duration(1.0 / 0.25), self.log_timer_callback)

        # setup pose callback - Pose update from hector_slam - rostopic hz /slam_out_pose ~ 13.5 hz Jetson TX1
        rospy.Subscriber("slam_out_pose", PoseStamped, self.hector_pose_callback)

        # setup map callback - Map update from hector_slam - ros hz map ~ 0.6 hz Jetson TX1
        rospy.Subscriber("map", OccupancyGrid, self.map_callback)

        # read VESC parameters from config/default.yaml:
        # TODO: Doesnt work, need to learn how to use rospy.get_param
        # self.force_scale_x = rospy.get_param("force_scale_x")

    def ackermann_cmd_input_callback(self, msg):
        # republish the input as output (not exactly "safe")
        self.cmd_pub.publish(msg)

    def blue_line_callback(self, msg):
        # from KmeansMsg - blue line msg
        data = msg.data.split(',') # CSV
        #if len(bl_cmin_at_la) > 0:
        #    for col in bl_cmin_at_la:
        #        bl_col_str = bl_col_str + "," + np.str(np.round(col, 2))

         #   bl_string = \
         #       "i_bl," + np.str(i_bl) + ",i_ll," + np.str(i_ll) + ",i_rl," + np.str(i_rl) + \
         #       ",conf_bl," + np.str(conf_bl) + ",conf_ll," + np.str(conf_ll) + ",conf_rl," + \
         #       np.str(conf_rl) + ",n_bl," + np.str(len(bl_cmin_at_la)) + "," + bl_col_str
        print(data)
        cntr_c = [] # this will be the column to control too, input to lookup table for steering command
        if len(data) > 2:
            # at least one blue line was found
            data.pop(0)
            i_bl = int(data[1])
            i_ll = int(data[3])
            i_rl = int(data[5])
            conf_bl = int(data[7])
            conf_ll = int(data[9])
            conf_rl = int(data[11])
            n_bl = int(data[13])
            bl_col_str = data[15:]
            bl_col_str = [float(i) for i in bl_col_str]

            if i_bl > -1:
                test_c = bl_col_str[i_bl]
                cntr_c = test_c
                self.cntr_c_prev = cntr_c
            elif i_ll > -1:
                cntr_c = bl_col_str[i_ll]
                self.cntr_c_prev = cntr_c
            elif i_rl > -1:
                cntr_c = bl_col_str[i_rl]
                self.cntr_c_prev = cntr_c
            else:
                # no blue-line found
                cntr_c = self.cntr_c_prev
        else:
            cntr_c = self.cntr_c_prev
        # now control column should be defined, time to lookup steering angle
        c_lookup = [0,      10,     20,     30,     40,     50,     60,     64,     70,     80,     90,     100,    110,    120,    128]
        str_axis = [1,       1,      1,      1,     .7,    .35,    .15,      0,   -.15,   -.35,    -.7,      -1,     -1,     -1,     -1]
        k_str_filt = .1

        self.str_cmnd_prev = self.str_cmnd # update previous cmnd
        str_cmnd_raw = lininterp.lookup(cntr_c, c_lookup, str_axis)  # linear interpolation of steering calibration table

        #first order filter on steering command:
        str_cmnd_err = str_cmnd_raw - self.str_cmnd_prev # delta to desired command
        str_cmnd_filt = self.str_cmnd_prev + k_str_filt*str_cmnd_err

        #this will be the final steering angle command
        self.str_cmnd  = str_cmnd_filt

    def log_timer_callback(self, msg):
        # 1 hz publisher
        # for messaging
        rospy.loginfo("stayin alive")
        rospy.loginfo(self.mb_gnrl.get_pose())
        rospy.loginfo("Map Pose:")
        rospy.loginfo(self.mb_gnrl.get_map().info.origin)

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
        MowbotControllerNode.mb_gnrl.update_pose(self.pose_obj)

    def map_callback(self, msg):
        # hector_slam /map callback - averages 0.56 Hz
        self.map_obj = msg
        MowbotControllerNode.mb_gnrl.update_map(self.map_obj)

    def update_mb_vesc_cmd(self):
        MowbotControllerNode.mb_gnrl.cmd_steering_angle = self.str_cmnd #set steer command from kmeans / blueline
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
