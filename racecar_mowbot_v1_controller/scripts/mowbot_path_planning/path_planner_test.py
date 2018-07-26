#!/usr/bin/python
#

#   path_planner_test.py
#   Shaun Bowman
#   2018/07/26
#
#   Purpose:
#   - Stand-alone test script for mowbot_path_planner.py
#   - Not to require ROS

from mowbot_path_planner import MowbotPathPlanner
from mtrx_pos import MatrixPos
import numpy as np
import matplotlib.pyplot as plt
from scipy import ndimage

pp = MowbotPathPlanner()

run_tests = {
            'basic': 0,
            'map_20_20' : 0,
            'map_5_5' : 0,
            'map_1000_1000_bmp' : 1,
            'planner_create' : 1,
            'find_navigable_map' :0,
            'run_dijkstra' : 1
            }

start = MatrixPos()
goal = MatrixPos()
map = []
planner = []

# basic test - works
if run_tests['basic'] == 1:
    x = np.arange( 15 )
    y = np.sin( [np.pi/10*i for i in x] )
    result = pp.mult_nums(x, y)

    plt.plot(x, result)
    plt.show()


# Notes on map:
# - Map is an Ocupancy Grid format
# - each pixels value representing the probability of an area being occupied
# - value ranges from 0 to 100, with -1 meaning unknown
if run_tests['map_20_20'] == 1:
    # map_20_20 = np.array([
        # [1, 1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1 ],
        # [1, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1 ],
        # [1, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1 ],
        # [1, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1 ],
        # [1, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1 ],
        # [1, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1 ],
        # [1, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1 ],
        # [1, 0,  0,  0,  0,  0,  1,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1 ],
        # [1, 0,  0,  0,  0,  1,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1 ],
        # [1, 0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1 ],
        # [1, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1 ],
        # [1, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  0,  0,  0,  1 ],
        # [1, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  0,  0,  0,  1 ],
        # [1, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1 ],
        # [1, 0,  0,  1,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1 ],
        # [1, 0,  0,  1,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1 ],
        # [1, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  1 ],
        # [1, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1 ],
        # [1, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1 ],
        # [1, 1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1 ]
        # ])
    map_20_20 = np.array([
        [1, 1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1 ],
        [1, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1 ],
        [1, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  1 ],
        [1, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  1 ],
        [1, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  1,  1,  1,  1,  0,  1 ],
        [1, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  1 ],
        [1, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  1 ],
        [1, 0,  0,  0,  0,  0,  1,  1,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  1 ],
        [1, 0,  0,  0,  0,  1,  1,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  1 ],
        [1, 0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  1 ],
        [1, 0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  1 ],
        [1, 0,  0,  0,  0,  0,  0,  1,  0,  0,  1,  0,  0,  0,  1,  1,  0,  0,  0,  1 ],
        [1, 0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  1,  1,  0,  0,  0,  1 ],
        [1, 0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1 ],
        [1, 0,  0,  1,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1 ],
        [1, 0,  0,  1,  1,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  1 ],
        [1, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  1 ],
        [1, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1 ],
        [1, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1 ],
        [1, 1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1 ]
    ])

    plt.figure()
    plt.imshow( map_20_20 )
    plt.show()
    map = map_20_20

    start_pos = MatrixPos()
    start_pos.i = 17
    start_pos.j = 17

    end_pos = MatrixPos()
    end_pos.i = 3
    end_pos.j = 3

    print( "Start Position i,j: " + str(start_pos.i) + "," + str(start_pos.j)
           + " map value: " + str( map_20_20[start_pos.i, start_pos.j] ) )

    start = start_pos
    goal = end_pos

# Notes on map:
# - Map is an Ocupancy Grid format
# - each pixels value representing the probability of an area being occupied
# - value ranges from 0 to 100, with -1 meaning unknown
if run_tests['map_5_5'] == 1:
# map_20_20 = np.array([
# [1, 1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1 ],
# ])
    map_5_5 = np.array([
        [1, 1,  1,  1,  1],
        [1, 0,  0,  0,  1],
        [1, 0,  1,  0,  1],
        [1, 0,  0,  0,  1],
        [1, 1,  1,  1,  1]
    ])
    plt.figure()
    plt.imshow( map_5_5 )
    plt.show()
    map = map_5_5

    start_pos = MatrixPos()
    start_pos.i = 3
    start_pos.j = 3

    end_pos = MatrixPos()
    end_pos.i   = 1
    end_pos.j   = 1

    start = start_pos
    goal = end_pos

if run_tests['map_1000_1000_bmp'] == 1:
    img_map = ndimage.imread('room_1000_1000.bmp')
    img_map = img_map*-1 + 1 # rotate image.. dumb
    img_map[0,:] = 1
    img_map[-1,:] = 1
    img_map[:, 0] = 1
    img_map[:, -1] = 1

    plt.figure()
    plt.imshow( img_map )
    plt.show()
    map = img_map

    start_pos = MatrixPos()
    start_pos.i = 980
    start_pos.j = 980

    end_pos = MatrixPos()
    end_pos.i   = 20
    end_pos.j   = 20

    start = start_pos
    goal = end_pos

if run_tests['planner_create'] == 1:
    planner = MowbotPathPlanner()
    planner.start_pos = start
    planner.goal = goal
    planner.map = map

if run_tests['run_dijkstra'] == 1:
    # compute path plan using dijkstra's algorithm (shortest path)
    planner.get_path_dijkstra( b_DEBUG=False, b_VISUAL=False, b_VISUAL_RESULT_ONLY=True )

if run_tests['find_navigable_map']:
    # using specified spacing in meteres of occupancy grid & minimum turning radius of robot
    # dialate occupied portions in the grid to prevent robot from hitting things (somewhat)
    planner.pad_grid( b_VISUAL=True )