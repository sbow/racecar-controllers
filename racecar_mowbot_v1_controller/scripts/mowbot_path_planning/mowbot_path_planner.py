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
from scipy import ndimage
import math
import  matplotlib.pyplot as plt

class MowbotPathPlanner:
    has_init = False
    start_pos = []
    goal = []
    map = []
    map_navigable = []
    x_spacing = [] # meters
    y_spacing = [] # meters
    dijkstra = MowbotDijkstra()
    turn_radius =   []  # meters, tightest turning radius of robot (for bicycle model)
    veh_wheelbase   =   []  # meters, wheelbase of robot (for bicycle model)
    colormapval = (0, 8)

    def __init__(self):
        self.has_init = True
        self.start_pos = MatrixPos()
        self.goal = MatrixPos()
        self.map = []
        self.x_spacing = 1
        self.y_spacing = 1
        self.turn_radius = 0.75
        self.veh_wheelbase =  0.35

    def mult_nums(self, x, y):
        return x*y

    def get_path_dijkstra(self, b_DEBUG, b_VISUAL, b_VISUAL_RESULT_ONLY):
        # use dijkstra algorithm on occupancy grid
        # https://realitybytes.blog/2017/07/11/graph-based-path-planning-dijkstras-algorithm/
        self.dijkstra.DEBUG = b_DEBUG
        self.dijkstra.VISUAL = b_VISUAL
        self.dijkstra.VISUAL_RESULT_ONLY = b_VISUAL_RESULT_ONLY
        self.dijkstra.do_dijkstras(self.map, self.x_spacing, self.y_spacing, self.start_pos, self.goal)
        pass

    def pad_grid(self, b_VISUAL):
        n_dilations = int( math.ceil( self.turn_radius / self.x_spacing )) # note: this may be simplisitc if x/y spacing differ
        dilation_filter = ndimage.generate_binary_structure(2,1) # [false, true, false],[true, true, true],[false true false]
        map_subset = self.map[1:-1,1:-1] # ignore 1's on boundry of map
        map_subset = ndimage.binary_dilation( map_subset, structure= dilation_filter, iterations=1) # expand regions for turning radius
        #map_subset = ndimage.binary_dilation( map_subset, structure= dilation_filter, iterations=n_dilations) # expand regions for turning radius
        self.map_navigable = np.pad(map_subset, [(1,1),(1,1)], mode='constant', constant_values=1) # add boundry 1's back into map

        if b_VISUAL:
            fig = plt.figure(figsize=(12,12))
            ax = fig.add_subplot(211)
            ax.set_title('Original Grid')
            plt.xticks(visible=False)
            plt.yticks(visible=False)
            plt.imshow(self.map, origin='upper', interpolation='none', clim=self.colormapval)
            ax.set_aspect('equal')
            ax = fig.add_subplot(212)
            ax.set_title('Occupancy Grid')
            plt.xticks(visible=False)
            plt.yticks(visible=False)
            plt.imshow(self.map_navigable, origin='upper', interpolation='none', clim=self.colormapval)
            ax.set_aspect('equal')
            plt.show()
