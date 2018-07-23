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
from mtrx_pos import MatrixPos
from mowbot_dijkstra import MowbotDijkstra

class MowbotPathPlanner:
    has_init = False
    start_pos = []
    goal = []
    map = []
    x_spacing = [] # meters
    y_spacing = [] # meters
    dijkstra = MowbotDijkstra()

    def __init__(self):
        self.has_init = True
        self.start_pos = MatrixPos()
        self.goal = MatrixPos()
        self.map = []
        self.x_spacing = 1
        self.y_spacing = 1

    def mult_nums(self, x, y):
        return x*y

    def get_path_dijkstra(self, b_DEBUG, b_VISUAL):
        # use dijkstra algorithm on occupancy grid
        # https://realitybytes.blog/2017/07/11/graph-based-path-planning-dijkstras-algorithm/
        self.dijkstra.DEBUG = b_DEBUG
        self.dijkstra.VISUAL = b_VISUAL
        self.dijkstra.do_dijkstras(self.map, self.x_spacing, self.y_spacing, self.start_pos, self.goal)
        pass
