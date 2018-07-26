#!/usr/bin/python
#

#    mowbot_dijkstra.py
#    Shaun Bowman
#
#
#    Purpose:
#    path-planning algorithm by Dijkstra
#    using an occupancy grid as an initial map (not graph)
#
#    Based on work by Andrew Dahdouh

#    https://realitybytes.blog/2017/07/11/graph-based-path-planning-dijkstras-algorithm/
'''
BSD 2-Clause License

Copyright (c) 2017, Andrew Dahdouh
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

import numpy as np
import math
import matplotlib.pyplot as plt
import pprint

class MowbotDijkstra():

    ANIMATION_PAUSE = 0.001
    DEBUG = False
    VISUAL = False
    VISUAL_RESULT_ONLY = False

    def __init__(self):

        pass

    def do_dijkstras(self, occupancy_map, x_spacing, y_spacing, start, goal):
        """
        Implements Dijkstra's shortest path algorithm
        Input:
        occupancy_map - an N by M numpy array of boolean values (represented
            as integers 0 and 1) that represents the locations of the obstacles
            in the world
        x_spacing - parameter representing spacing between adjacent columns
        y_spacing - parameter representing spacing between adjacent rows
        start - a 3 by 1 numpy array of (x,y,theta) for the starting position
        goal - a 3 by 1 numpy array of (x,y,theta) for the finishing position
        Output:
        path: list of the indices of the nodes on the shortest path found
            starting with "start" and ending with "end" (each node is in
            metric coordinates)
        """
        # self.DEBUG = False
        # self.VISUAL = False
        colormapval = (0, 8)
        goal_found = False

        # Setup Map Visualizations:
        if self.VISUAL == True:
            viz_map=occupancy_map
            fig = plt.figure(figsize=(12,12))
            ax = fig.add_subplot(111)
            ax.set_title('Occupancy Grid')
            plt.xticks(visible=False)
            plt.yticks(visible=False)
            plt.imshow(viz_map, origin='upper', interpolation='none', clim=colormapval)
            ax.set_aspect('equal')
            plt.pause(self.ANIMATION_PAUSE)

        # We will use this delta function to search surrounding nodes.
        delta = [[-1, 0],  # go up
                 [0, -1],  # go left
                 [1, 0],  # go down
                 [0, 1]]  # go right

        # Each node on the map "costs" 1 step to reach.
        cost = 1

        # Convert numpy array of map to list of map, makes it easier to search.
        occ_map = occupancy_map.tolist()
        if self.DEBUG == True:
            print "occ_map: "
            pprint.pprint(occ_map)

        # Converge start and goal positions to map indices.
        x = int(math.ceil((start.j / x_spacing) - 0.5))  # startingx
        y = int(math.ceil((start.i / y_spacing) - 0.5))  # startingy
        goalX = int(math.ceil((goal.j / x_spacing) - 0.5))
        goalY = int(math.ceil((goal.i / y_spacing) - 0.5))
        print "Start Pose: ", x, y
        print "Goal Pose: ", goalX, goalY

        # Make a map to keep track of all the nodes and their cost distance values.
        possible_nodes = [[0 for row in range(len(occ_map[0]))] for col in range(len(occ_map))]
        row = y
        col = x

        # Show the starting node and goal node.
        # 5 looks similar to S and 6 looks similar to G.
        possible_nodes[row][col] = 5

        if self.VISUAL == True:
            viz_map[row][col] = 5
            viz_map[goalY][goalX] = 6
            plt.imshow(viz_map, origin='upper', interpolation='none', clim=colormapval)
            plt.pause(self.ANIMATION_PAUSE)

        if self.DEBUG == True:
            print "Possible Nodes: "
            pprint.pprint(possible_nodes)

        # The g_value will count the number of steps each node is from the start.
        # Since we are at the start node, the total cost is 0.
        g_value = 0
        frontier_nodes = [(g_value, col, row)] # dist, x, y
        searched_nodes = []
        parent_node = {}  # Dictionary that Maps {child node : parent node}
        loopcount = 0

        while len(frontier_nodes) != 0:
            if self.DEBUG == True:
                "\n>>>>>>>>>>>>LOOP COUNT: ", loopcount, "\n"
            frontier_nodes.sort(reverse=True) #sort from shortest distance to farthest
            current_node = frontier_nodes.pop()
            if self.DEBUG == True:
                print "current_node: ", current_node
                print "frontier nodes: ", searched_nodes

            if current_node[1] == goalX and current_node[2] == goalY:
                print "Goal found!"
                goal_found = True
                if self.VISUAL == True:
                    plt.text(2, 10, s="Goal found!", fontsize=18, style='oblique', ha='center', va='top')
                    plt.imshow(viz_map, origin='upper', interpolation='none', clim=colormapval)
                    plt.pause(self.ANIMATION_PAUSE)
                break
            g_value, col, row = current_node

            # Check surrounding neighbors.
            for i in delta:
                possible_expansion_x = col + i[0]
                possible_expansion_y = row + i[1]
                valid_expansion = 0 <= possible_expansion_y < len(occupancy_map[0]) and 0 <= possible_expansion_x < len(occ_map)
                if self.DEBUG == True:
                    print "Current expansion Node: ", possible_expansion_x, possible_expansion_y

                if valid_expansion:
                    try:
                        unsearched_node = possible_nodes[possible_expansion_y][possible_expansion_x] == 0
                        open_node = occ_map[possible_expansion_y][possible_expansion_x] == 0
                        if self.DEBUG == True:
                            print "Check Open or Wall: ", occ_map[possible_expansion_y][possible_expansion_x]
                    except:
                        unsearched_node = False
                        open_node = False
                    if unsearched_node and open_node:
                        # Using  instead of 1 to make it easier to read This node has been searched.
                        # searched_row = possible_expansion_y
                        # searched_col = possible_expansion_x
                        possible_nodes[possible_expansion_y][possible_expansion_x] = 3
                        possible_node = (g_value + cost, possible_expansion_x, possible_expansion_y)
                        frontier_nodes.append(possible_node)
                        if self.DEBUG == True:
                            print "frontier_nodes:", frontier_nodes
                        if self.VISUAL == True:
                            viz_map[possible_expansion_y][possible_expansion_x] = 3
                            plt.imshow(viz_map, origin='upper', interpolation='none', clim=colormapval)
                            plt.pause(self.ANIMATION_PAUSE)


                        # This now builds parent/child relationship
                        parent_node[possible_node] = current_node
                        if self.DEBUG == True:
                            print "Parent Node: \n", parent_node
                            print "While Possible Nodes: "
                            pprint.pprint(possible_nodes)
            loopcount = loopcount+1

        if goal_found == True:

            print "Generating path..."

            route = []
            child_node = current_node
            while parent_node.has_key(child_node):
                route.append(parent_node[child_node])
                child_node = parent_node[child_node]
                route.sort()

            #  route back to metric units:
            if self.DEBUG == True:
                print "Route: ", route
            if self.VISUAL == True:
                for i in range(0, len(route)):
                    viz_map[route[i][2]][route[i][1]] = 7
                    plt.imshow(viz_map, origin='upper', interpolation='none', clim=colormapval)
                    plt.pause(self.ANIMATION_PAUSE)

                viz_map[goalY][goalX] = 7
                plt.imshow(viz_map, origin='upper', interpolation='none', clim=colormapval)
                plt.pause(self.ANIMATION_PAUSE)

            if self.VISUAL_RESULT_ONLY == True:
                viz_map=occupancy_map
                for i in range(0, len(route)):
                    viz_map[route[i][2]][route[i][1]] = 7
                viz_map[goalY][goalX] = 7
                fig = plt.figure(figsize=(12,12))
                ax = fig.add_subplot(111)
                ax.set_title('Occupancy Grid')
                plt.xticks(visible=False)
                plt.yticks(visible=False)
                plt.imshow(viz_map, origin='upper', interpolation='none', clim=colormapval)
                ax.set_aspect('equal')
                plt.show()


            path = []
            position = [start.j, start.i]  # Starting point passed in by function
            path.append(position)  # Add it to the list for the path

            for i in range(0, len(route)):
                position = [round((route[i][1]+0.5)*x_spacing, 3), round((route[i][2]+0.5)*y_spacing, 3)]
                path.append(position)

            # Add the goal state:

            position = [goal.j, goal.i]
            path.append(position)

            print "Path: "
            pprint.pprint(path)

            # Convert to numpy array and return.
            path = np.array(path)
            return path

        else:
            if self.VISUAL == True:
                plt.text(2, 10, s="No path found...", fontsize=18, style='oblique', ha='center', va='top')
                plt.imshow(viz_map, origin='upper', interpolation='none', clim=colormapval)
                plt.pause(self.ANIMATION_PAUSE)

            if self.VISUAL_RESULT_ONLY == True:
                viz_map=occupancy_map
                fig = plt.figure(figsize=(12,12))
                ax = fig.add_subplot(111)
                ax.set_title('Occupancy Grid')
                plt.xticks(visible=False)
                plt.yticks(visible=False)
                plt.imshow(viz_map, origin='upper', interpolation='none', clim=colormapval)
                ax.set_aspect('equal')
                plt.show()

            return False



