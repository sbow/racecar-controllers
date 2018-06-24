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

        MowbotV1Main.cmd_steering_angle = 0.2
        MowbotV1Main.cmd_steering_angle_velocity = 0.25
        pass

    def long_command(self):
        # determine next longnituinal (throttle) command
        # MowbotV1Main.cmd_speed
        # MowbotV1Main.cmd_acceleration
        # MowbotV1Main.cmd_jerk

        MowbotV1Main.cmd_speed = 0.25
        MowbotV1Main.cmd_acceleration = 0.50
        MowbotV1Main.cmd_jerk = 2.00
        pass

    def rationalize_command(self):
        # check for max / min bounds of command
        # check for max step size of command
        # lat and long
        pass

