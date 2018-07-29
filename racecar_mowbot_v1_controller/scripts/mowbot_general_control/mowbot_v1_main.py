#!/usr/bin/python
#

# MOWBOT controller - ROS independant

class MowbotV1Main:
    # main class parameters
    # output should mirror Ackermann Command format:
    # float32 steering_angle  # desired virtual angle (radians)
    # float32 steering_angle_velocity  # desired rate of change (radians/s)
    # float32 speed  # desired forward speed (m/s)
    # float32 acceleration  # desired acceleration (m/s^2)
    # float32 jerk  # desired jerk (m/s^3)

    cmd_steering_angle = 0.0
    cmd_steering_angle_velocity = 0.0
    cmd_speed = 0.0
    cmd_acceleration = 0.0
    cmd_jerk = 0.0

    pose_slam = []
    map = []


    def __init__(self):
        # setup variables
        # create child classes
        pass

    def get_command(self):
        # main "loop"
        self.read_world()
        self.plan_path()
        self.plan_step()
        self.lat_command()
        self.long_command()
        self.rationalize_command()
        pass

    def read_world(self):
        # sense - should be ROS agnostic
        pass

    def plan_path(self):
        # plan path - ros agnostic
        pass

    def plan_step(self):
        # determine next step on path
        pass

    def lat_command(self):
        # determine next lateral (steering) command
        # MowbotV1Main.cmd_steering_angle
        # MowbotV1Main.cmd_steering_angle_velocity

        MowbotV1Main.cmd_steering_angle = 0.2688 # 0.2688 rad should be 2.44 m / 96 inch diam turn circle
        MowbotV1Main.cmd_steering_angle_velocity = 0.25
        pass

    def long_command(self):
        # determine next longnituinal (throttle) command
        # MowbotV1Main.cmd_speed
        # MowbotV1Main.cmd_acceleration
        # MowbotV1Main.cmd_jerk

        MowbotV1Main.cmd_speed = 0.5
        MowbotV1Main.cmd_acceleration = 0.50
        MowbotV1Main.cmd_jerk = 2.00
        pass

    def rationalize_command(self):
        # check for max / min bounds of command
        # check for max step size of command
        # lat and long
        pass

    def get_pose(self):
        return self.pose_slam
        pass

    def update_pose(self, pose):
        # Update pose
        # generic format is slam_out_pose
        # Example Output:
        #
        # header:
        # seq: 2088
        # stamp:
        # secs: 1529885767
        # nsecs: 919487405
        # frame_id: map
        # pose:
        # position:
        # x: 0.0012092590332
        # y: -0.00422286987305
        # z: 0.0
        # orientation:
        # x: 0.0
        # y: 0.0
        # z: 0.00163327538902
        # w: 0.999998666205
        self.pose_slam = pose
        pass

    def get_map(self):
        # return map data
        return self.map

    def update_map(self, map):
        # update map data
        self.map = map
        pass



