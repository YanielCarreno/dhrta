#!/usr/bin/env python
# Author: Yaniel Carreno
# All rights reserved.

from __future__ import print_function
import sys
import numpy as np
import rospy
import cv2
import time
import re
import yaml
from std_msgs.msg import String
from scipy.optimize import linear_sum_assignment
from collections import Counter
from linecache import getline
from std_srvs.srv import Empty, EmptyResponse
from dhrta_msgs.srv import GoalAllocation, GoalAllocationResponse

# Import the library to read csv
import csv

class cluster(object):

    def __init__(self):
        '''
        constructor
        '''
        rospy.init_node('decentralised_goal_allocation')
        rospy.Service('decentralised_goal_allocation', GoalAllocation, self.serviceCall)

        # Has the goal been loaded?
        self.params_loaded = False
        self.allocation_finished = False
        self.constraints_array = []

    def serviceCall(self, req):
        self.mission_constrains_directory = rospy.get_param('~mission_constrains_directory')
        self.ontology_directory = rospy.get_param('~ontology_directory')
        self.allocation_approach = req.allocation_approach
        self.waypoint_file = req.waypoints_file
        self.goals_file = req.goals_file
        self.robot_file = req.robot_file
        self.center_no = req.robots_number
        self.robots_number = req.robots_number
        self.total_iteration = 500

        '''
        Task Allocation strategy considering homogeneous robots which is based on k-means algorithm
        info: we start assuming we know the coordinates of mission goals and number of robots available
        '''
        if(self.allocation_approach.find("DHRTA") != -1):
            self.centroids = self.create_centroids()
            self.load_data()
            self.load_makespan()
            self.load_cap_analyser_results()
            if(self.params_loaded):
                self.k_means()
                self.cluster_allocation()
                self.hrta_strategy()
                print("***************************************************************************************")
                print("* GOAL ALLOCATOR:                                                                     *")
                print("*   - minimise {tasks makespan, distance betweent POIs}                               *")
                print("*                                                                                     *")
                print("***************************************************************************************")
                self.goal_distribution()
                self.save_goals()
                return GoalAllocationResponse(self.allocation_finished, len(self.allocation_results))

        else:
            print('MATP: Fleet characteristics were not properly defined ')
    def hrta_strategy(self):
        self.constraints_array = []
        self.cluster_order = []
        self.goals_in_cluster = []
        self.goals_in_cluster_wp = []
        self.number_solvable_goals = []
        self.allocated_goals = []
        self.robot = []
        self.waypoints = []
        self.set = []
        self.reordering = []
        self.index_for_g = []
        self.clust = []
        self.act_goal = []
        self.solvable_g = []
        self.reordering_goals = []
        self.makespan_accumulated = np.zeros(len(self.robot_points_id))
        self.accumulative_goal = []
        for c in range(0,len(self.goal_order)):
            for count in range(0,len(self.final_allocation)):
                if (self.goal_order[c] == self.final_allocation[count][0]):
                   self.cluster_order.append(self.final_allocation[count][1])
                   break
        for n in range(0, len(self.centroids)):
            self.cluster_goals = []
            self.cluster_goals_wp = []
            for g in range(0, len(self.cluster_order)):
                if (self.cluster_order[g] == n):
                    self.cluster_goals.append(self.goals_index[g])
                    self.cluster_goals_wp.append(self.goal_order[g])

            self.goals_in_cluster.append(self.cluster_goals)
            self.goals_in_cluster_wp.append(self.cluster_goals_wp)

        for cluster in range(0, len(self.new_centroids)):
            for robot in range(0, len(self.tasks_index)):
                total_goals = 0
                self.allocated_goals = []
                for cluster_index in range(0, len(self.goals_in_cluster[cluster])):
                    if self.goals_in_cluster[cluster][cluster_index] in self.tasks_index[robot]:
                       total_goals = total_goals +1
                       self.allocated_goals.append(self.goals_in_cluster[cluster][cluster_index])

                self.solvable_g.append(self.allocated_goals)
                self.number_solvable_goals.append(total_goals)
                self.robot.append(robot)
                self.clust.append(cluster)

        self.max = np.argmax(self.number_solvable_goals)
        self.max_val = np.nanmax(self.number_solvable_goals)

        for h in range(0, len(self.number_solvable_goals)):
            new_value = self.max_val - self.number_solvable_goals[h]
            self.set.append(new_value)

        for i in range(0, len(self.new_centroids)):
            self.reordering_first = []
            self.reordering_goals_first = []
            for u in range(0, len(self.robot)):
                if (self.robot[u] == i):
                    self.reordering_first.append(self.set[u])
                    self.reordering_goals_first.append(self.solvable_g[u])
            self.reordering.append(self.reordering_first)
            self.reordering_goals.append(self.reordering_goals_first)
        cost = np.array(self.reordering)
        row_ind, col_ind = linear_sum_assignment(cost)

        for n in range(0, len(self.new_centroids)):
            self.index_for_g.append(self.reordering_goals[row_ind[n]][col_ind[n]])
            self.waypoints_initial = []
            for r in range(0, len(self.goals_index)):
                if self.goals_index[r] in self.reordering_goals[row_ind[n]][col_ind[n]]:
                    self.waypoints_initial.append(self.goal_order[r])
            self.waypoints.append(self.waypoints_initial)

        self.dist_set_initial = []
        for robot_coord in range(0, len(self.centroids)):
            r_point = np.array(self.robot_data_points[robot_coord])
            self.initial_point = r_point
            self.dist_set_initial = []
            for g_point in range(0, len(self.goals_index)):
                if str(self.goals_index[g_point]) in self.index_for_g[robot_coord]:
                    index = self.goal_order[g_point]
                    index_second = self.points_id.index(index)
                    self.final_point = np.array(self.data_points[index_second])
                    self.dist[robot_coord] = self.compute_euclidean_distance()
                    self.dist_set_initial.append(self.dist[robot_coord])

            if not len(self.dist_set_initial) == 0:
                self.m_dist_index = np.argmin(self.dist_set_initial)
                self.m_dist_value = np.nanmin(self.dist_set_initial)
                new_wp = self.waypoints[robot_coord][self.m_dist_index]
                index_coordinate_new_robot_pos = self.points_id.index(new_wp)
                self.robot_data_points[robot_coord] = self.data_points[index_coordinate_new_robot_pos]
                self.robot_points_id[robot_coord] = new_wp
                self.makespan_accumulated[robot_coord] = self.m_dist_value
                self.accumulative_goal.append(new_wp)

                indx_goal_ordered = self.waypoints[robot_coord].index(new_wp)
                self.goal_to_remove = str(self.index_for_g[robot_coord][indx_goal_ordered])
                index_to_remove = self.goals_index.index(self.goal_to_remove)
                self.goals_index.remove(self.goal_to_remove)
                self.goal_order.pop(index_to_remove)

                for task in range(0,len(self.tasks_index)):
                    if self.goal_to_remove in self.tasks_index[task]:
                       value = self.tasks_index[task]
                       curr = value.replace(str(self.goal_to_remove), "")
                       self.tasks_index[task] = curr
            self.act_goal.append(self.goal_to_remove)
        for constraints in range(0,len(self.new_centroids)):
            if not self.robot_points_id[constraints] in self.initial_robot_wp:
                self.constraints_array.append([self.robot_name[constraints],self.robot_points_id[constraints], self.act_goal[constraints]])

    def goal_distribution(self):
        self.makespan_accumulated = np.zeros(len(self.robot_points_id))
        self.accumulative_goal = []
        self.total_goals_no = self.goals_index
        for self.task_distribution in range(0,len(self.total_goals_no)):
            self.min_costfunc_value = []
            self.min_costfunc_index = []
            self.distance_values = {}
            alpha = 0.45
            for goal_set_indx in range(0, len(self.goals_index)):
                self.costfunc = []
                self.makespan_value = []
                self.v = []
                for robot_indx in range(0, len(self.centroids)):
                    if str(self.goals_index[goal_set_indx]) in self.tasks_index[robot_indx]:
                       robot_coordinates = self.robot_data_points[robot_indx]
                       wp_goal = np.array(self.goal_order[goal_set_indx])
                       index = self.points_id.index(wp_goal)
                       goal_coordinates = np.array(self.data_points[index])
                       self.initial_point = robot_coordinates
                       self.final_point = goal_coordinates
                       self.distance_values[robot_indx] = self.compute_euclidean_distance()
                       dist = self.distance_values[robot_indx]
                       goal_name = str(self.goal_description[goal_set_indx])
                       action_makespan = self.makespan_list['makespan'][goal_name]
                    else:
                       dist = 10**10
                       action_makespan = 1
                    if self.initial_robot_wp[robot_indx] in self.robot_points_id:
                        self.costfunc.append(dist)
                    else:
                        self.costfunc.append(alpha*(action_makespan + self.makespan_accumulated[robot_indx]) + (1 - alpha)*dist)
                self.min_dist_index = np.argmin(self.costfunc)
                self.min_dist_value = np.nanmin(self.costfunc)
                self.min_costfunc_value.append(self.min_dist_value)
                self.min_costfunc_index.append(self.min_dist_index)

            self.min_of_min_indx = np.argmin(self.min_costfunc_value)
            self.min_of_min = np.nanmin(self.min_costfunc_value)
            self.alloc_robot_indx = self.min_costfunc_index[self.min_of_min_indx]
            self.final_robot = self.robot_name[self.alloc_robot_indx]
            self.final_goal = self.goal_order[self.min_of_min_indx]
            self.goal_to_remove = self.goals_index[self.min_of_min_indx]
            self.constraints_array.append([self.final_robot,self.final_goal,self.goal_to_remove])
            self.accumulative_goal.append(self.final_goal)
            self.makespan_accumulated[self.alloc_robot_indx] = self.min_of_min + self.makespan_accumulated[self.alloc_robot_indx]
            self.robot_position_update()

    def robot_position_update(self):
        self.robot_points_id[self.alloc_robot_indx] = self.final_goal
        index_coordinate_new_robot_pos = self.points_id.index(self.final_goal)
        self.robot_data_points[self.alloc_robot_indx] = self.data_points[index_coordinate_new_robot_pos]
        self.goals_index.remove(str(self.goal_to_remove))
        self.goal_order.pop(self.min_of_min_indx)

        for task in range(0,len(self.tasks_index)):
            if self.goal_to_remove in self.tasks_index[task]:
               value = self.tasks_index[task]
               curr = value.replace(str(self.goal_to_remove), "")
               self.tasks_index[task] = curr
    def k_means(self):
        '''
        k_means implement the clustering of the goals based on the number of available robots
        '''
        self.final_allocation = []
        self.clus_index = []
        [self.cluster_label, self.new_centroids] = self.iterate_k_means()
        self.total = len(self.points_id)
        for self.final_iteration in range(0, self.total):
            self.final_allocation.append([self.points_id[self.final_iteration],self.final_cluster[self.final_iteration]])
            if not self.final_cluster[self.final_iteration] in self.clus_index:
                self.clus_index.append(self.final_cluster[self.final_iteration])

    def iterate_k_means(self):
        self.label = []
        self.cluster_label = []
        self.final_cluster = []
        self.total_points = len(self.data_points)
        self.k = len(self.centroids)

        for self.iteration in range(0, self.total_iteration):
            for self.index_point in range(0, self.total_points):
                self.distance = {}
                for self.index_centroid in range(0, self.k):
                    point = self.data_points[self.index_point]
                    centroid = self.centroids[self.index_centroid]
                    self.initial_point = point
                    self.final_point = centroid
                    self.distance[self.index_centroid] = self.compute_euclidean_distance()
                self.label = self.assign_label_cluster()
                self.centroids[self.label[0]] = self.compute_new_centroids()

                if self.iteration == (self.total_iteration - 1):
                    self.cluster_label.append(self.label)
                    self.final_cluster.append(self.index_of_minimum)
        return [self.cluster_label, self.centroids]

    def cluster_allocation(self):
        self.clusters_allocated = []
        self.robots_matched = []
        self.clusters_matched = []
        self.distance_set = []
        self.cluster_index = []
        self.robot_id = []
        self.dist = {}
        for centroids_coordinates in range(0, len(self.new_centroids)):
            centroids_point = self.new_centroids[centroids_coordinates]
            self.final_point = centroids_point
            for robot_coordinates in range(0, len(self.robot_points_id)):
                robot_points = self.robot_data_points[robot_coordinates]
                self.initial_point = robot_points
                self.dist[robot_coordinates] = self.compute_euclidean_distance()
                self.distance_set.append(self.dist[robot_coordinates])
                self.robot_id.append(robot_coordinates)
                self.cluster_index.append(centroids_coordinates)

        for items in range(0, len(self.distance_set)):
            self.r = np.argmin(self.distance_set)
            self.robot = self.robot_id[self.r]
            self.argument = np.nanmin(self.distance_set)
            if not self.robot_name[self.robot] in self.robots_matched and not self.cluster_index[self.r] in self.clusters_matched:
                self.clusters_allocated.append([self.robot_name[self.robot], self.cluster_index[self.r]])
                self.robots_matched.append(self.robot_name[self.robot])
                self.clusters_matched.append(self.cluster_index[self.r])
            self.distance_set.pop(self.r)
            self.robot_id.pop(self.r)
            self.cluster_index.pop(self.r)

        for goal_allocation_constraint in range(0, len(self.final_allocation)):
            goal_id = self.final_allocation[goal_allocation_constraint]
            goal_id = str(goal_id)
            curr = goal_id.find(",")+1
            next = goal_id.find("]")
            cluster_goal_id = goal_id[curr:next]
            curr = goal_id.find("['")+2
            next = goal_id.find(",")-1
            goal_id = goal_id[curr:next]
            for robot_name_constraint in range(0, len(self.clusters_allocated)):
                robot_id = self.clusters_allocated[robot_name_constraint]
                robot_id = str(robot_id)
                curr = robot_id.find(",")+1
                next = robot_id.find("]")
                cluster_robot_id = robot_id[curr:next]
                curr = robot_id.find("[")+2
                next = robot_id.find(",")-1
                robot_id = robot_id[curr:next]
                if (cluster_goal_id == cluster_robot_id):
                    self.constraints_array.append([robot_id, goal_id])

    def create_centroids(self):
        self.centroids = []
        for self.number_robots in range(0, self.center_no):
            self.centroids.append([0.0, 0.0])
        return np.array(self.centroids)

    def load_data(self):
        '''
        load_data loads the data from waypoint, robot and goal files
        input: (1) waypoint_file  (2) goals_file (3) robot_file
        output: (1) goals_id and coordinates (2) robot_id and coordinates
        '''
        self.data_points = []
        self.points_id = []
        self.robot_data_points = []
        self.robot_points_id = []
        self.initial_robot_wp = []
        self.robot_name = []
        self.goals_index = []
        self.goal_order = []
        self.all_lines = []
        self.goals_intermediate_index = []
        self.goal_description = []
        try:
            print('Reading file %s' %self.waypoint_file)
            ifile = open(self.waypoint_file, "rw+")
            lines = 0
            for line in open(self.waypoint_file):
                lines += 1

            for line_no in range(0, lines):
                actual_line = getline(self.waypoint_file, line_no)
                reader = ifile.readline()
                curr = reader.find("[")
                name = reader[:curr]

                test_ifile = open(self.goals_file, "rw+")
                test_lines = 0
                for test_line in open(self.goals_file):
                    test_lines += 1

                for test_line_no in range(0, test_lines):
                    test_actual_line = getline(self.goals_file, test_line_no)
                    test_reader = test_ifile.readline()
                    test_curr = test_reader.find("wp")
                    test_next = test_reader.find(")",test_curr+1)
                    self.value = str(test_reader[test_curr:test_next])+","
                    wp_name_order = str(test_reader[test_curr:test_next])
                    first = test_reader.find('(')+1
                    second = test_reader.find(' ')
                    self.goal_descrip = test_reader[first:second]

                    if not test_line_no in self.all_lines:
                        self.all_lines.append(test_line_no)
                        self.goal_description.append(self.goal_descrip)

                    if not test_line_no in self.goals_intermediate_index:
                        self.goal_order.append(wp_name_order)
                        self.goals_index.append("g"+str(test_line_no)+".-")
                        self.goals_intermediate_index.append(test_line_no)

                    if (self.value.find(name+",") != -1):
                        curr = reader.find("[") + 1
                        next = reader.find(",",curr)
                        self.x_coord = reader[curr:next]

                        curr = next + 1
                        next = reader.find(",",curr)
                        self.y_coord = reader[curr:next]

                        self.data_points.append([float(self.x_coord), float(self.y_coord)])
                        self.points_id.append(name)

                robot_ifile = open(self.robot_file, "rw+")
                robot_lines = 0
                for robot_line in open(self.robot_file):
                    robot_lines += 1

                for robot_line_no in range(0, robot_lines):
                    robot_actual_line = getline(self.robot_file, robot_line_no)
                    robot_reader = robot_ifile.readline()
                    robot_curr = robot_reader.find("[")
                    robot_name = robot_reader[:robot_curr]

                    if robot_name in reader and not robot_name in self.robot_name:
                        curr = reader.find("[") + 1
                        next = reader.find(",",curr)
                        self.x_robot_coord = reader[curr:next]

                        curr = next + 1
                        next = reader.find(",",curr)
                        self.y_robot_coord = reader[curr:next]

                        self.robot_data_points.append([float(self.x_robot_coord), float(self.y_robot_coord)])
                        self.robot_points_id.append(name)
                        self.initial_robot_wp.append(name)
                        self.robot_name.append(robot_name)

            self.params_loaded = True
        except:
            print('Unable to read the indicated file')

    def load_cap_analyser_results(self):
        self.tasks_index = []
        for self.i_line in range(0, self.robots_number):
            self.goal_distribution_data = open(str(self.mission_constrains_directory)+ 'goals_indx_distribution.txt', 'rw')
            self.indx_line = self.goal_distribution_data.readlines()
            self.indx_line = self.indx_line[self.i_line]
            self.goal_distribution_data.close()
            curr = self.indx_line.find("[")+1
            next = self.indx_line.find("]")
            indexes = self.indx_line[curr:next]
            self.tasks_index.append(indexes)

    def load_makespan(self):
        with open(str(self.mission_constrains_directory)+str(self.ontology_directory), 'r') as stream:
            self.makespan_list = yaml.safe_load(stream)
    def compute_euclidean_distance(self):
        return np.sqrt(np.sum((self.initial_point - self.final_point)**2))

    def assign_label_cluster(self):
        self.index_of_minimum = min(self.distance, key=self.distance.get)
        return [self.index_of_minimum, self.data_points[self.index_point], self.centroids[self.index_of_minimum]]

    def compute_new_centroids(self):
        return np.array(self.label[1] + self.centroids[self.label[0]])/2

    def save_goals(self):
        self.allocation_results = []
        for goal_allocation_constraint in range(0, len(self.constraints_array)):
            line = self.constraints_array[goal_allocation_constraint]
            robot_id = str(line)
            curr = robot_id.find("['")
            next = robot_id.find("',")
            robot_name = robot_id[curr+2:next]
            final = robot_id.find("']")
            remain_part = robot_id[next+3:final]
            curr = remain_part.find("'")
            next = remain_part.find("',")
            goal_name = remain_part[curr+1:next]
            remain_part = remain_part[next:final]
            curr = remain_part.find("g")
            next = remain_part.find(".-")
            goal_id_no = remain_part[curr+1:next]
            ifile = open(self.goals_file, "rw+")
            lines = 0
            for line in open(self.goals_file):
                lines += 1
            for line_no in range(0, lines):
                line = getline(self.goals_file, line_no)
                reader = ifile.readline()
                curr = reader.find("(")+1
                next = reader.find(")")
                goal = reader[curr:next]
                if goal_name in reader and int(goal_id_no)==line_no:
                        self.allocation_statement = "("+str(goal)+")"+"["+str(robot_name)+"]"
                        self.allocation_results.append(self.allocation_statement)
        self.allocation_data = open(str(self.mission_constrains_directory)+ 'allocation_solution.txt', 'w+')
        for data_storage in range(0, len(self.allocation_results)):
            self.allocation_data.write(str(self.allocation_results[data_storage])+ '\n')
        self.allocation_data.close()
        self.allocation_finished = True

        self.elements = open(str(self.mission_constrains_directory)+ 'info_distribution.txt', 'w+')
        self.elements.write("("+str(self.allocation_finished)+")"+"["+str(len(self.allocation_results))+"]"+'\n')
        self.elements.close()

if __name__ == '__main__':
    try:
        cluster = cluster()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
