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
import numpy as np
import matplotlib.pyplot as plt

pp = MowbotPathPlanner()

run_tests = {
            'basic': 1,
            'next' : 0,
            }


# basic test - works
if run_tests['basic'] == 1:
    x = np.arange( 15 )
    y = np.sin( [np.pi/10*i for i in x] )
    result = pp.mult_nums(x, y)

    plt.plot(x, result)
    plt.show()

