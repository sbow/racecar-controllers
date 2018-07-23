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

pp = MowbotPathPlanner()

run_tests = {
            'basic': 0,
            'map_20_20' : 1,
            'fixed_start_goal' : 1,
            'planner_create' : 1,
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

if run_tests['fixed_start_goal'] == 1:
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


if run_tests['planner_create'] == 1:
    planner = MowbotPathPlanner()
    planner.start_pos = start
    planner.goal = goal
    planner.map = map

if run_tests['run_dijkstra'] == 1:
    # compute path plan using dijkstra's algorithm (shortest path)
    planner.get_path_dijkstra( b_DEBUG=False, b_VISUAL=True )
