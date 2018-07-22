#!/usr/bin/python
#

#   mowbot_path_planner.py
#   Shaun Bowman
#   2018/07/26
#
#   Purpose:
#   - Consume MAP data, output desired trajectory
#   - Trajectory to be based on a desired path, which hopefully will have a few options / techniques implemented here.
#   - Intention is for the trajectory to be consumed by mowbot_conntroller & commanded out to the motors / servo

import numpy as np


class MowbotPathPlanner:
    has_init = False

    def __init__(self):
        has_init = True

    def mult_nums(self, x, y):
        return x*y
