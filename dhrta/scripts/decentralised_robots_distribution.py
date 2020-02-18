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
import csv
from std_msgs.msg import String
from linecache import getline
from std_srvs.srv import Empty, EmptyResponse
from dhrta_msgs.srv import RobotsDistribution, RobotsDistributionResponse


class decentralised_robots_distribution(object):

    def __init__(self):
        #=======================================================================
        # constructor
        #=======================================================================
        rospy.init_node('decentralised_robots_distribution')
        rospy.Service('decentralised_robots_distribution', RobotsDistribution, self.serviceCall)
        #=======================================================================
        # Initialisation
        #=======================================================================
        self.capabilities_recognised = False
        self.goals_indx_allocated = False

    def serviceCall(self, req):
        #=======================================================================
        # System inputs to implement robots distribution
        #=======================================================================
        self.goals_file = req.goals_file
        self.robot_file = req.robot_file
        self.mission_constrains_directory = rospy.get_param('~mission_constrains_directory')
        self.ontology_directory = rospy.get_param('~ontology_directory')
        self.load_goals_cap_ontology()
        self.load_capabilities()
        if (self.capabilities_recognised):
            self.goals_indx_dist = open(str(self.mission_constrains_directory)+ 'goals_indx_distribution.txt', 'w+')
            for capability_robot_indx in range(0, len(self.robot_set)):
                self.final_array = []
                for capability_data_indx in range(0, len(self.data)):
                    for capability_goal_indx in range(0, len(self.goal_set)):
                        if self.robot_set[capability_robot_indx] in self.data[capability_data_indx] and  self.goal_set[capability_goal_indx] in self.data[capability_data_indx]:
                            self.final_array.append("g"+str(capability_goal_indx)+".-")

                self.solution_goal = str(self.robot_set[capability_robot_indx])+str(self.final_array)
                self.goals_indx_dist.write(str(self.solution_goal)+ '\n')
            self.goals_indx_dist.close()
            self.goals_indx_allocated = True
            print("***************************************************************************************")
            print("* ROBOTS DISTRIBUTION:                                                                *")
            print("*      - Capability Analysis                                                          *")
            print("*      - maximise {number of solvable goals}                                          *")
            print("***************************************************************************************")
            return RobotsDistributionResponse(len(self.robot_set),self.goals_indx_allocated)


    def load_capabilities(self):
        #=======================================================================
        # Loads the goal and robot capabilities from the file to find the
        # sets of robots that can do a particular task.
        # input: robot_file and goals_file
        # output: index of the possible tasks to be implemented for the robot
        #=======================================================================
        try:
            print('Reading file %s' %self.goals_file)
            goals_ifile = open(self.goals_file, "rw+")
            self.goal_file_lines = 0
            for goals_line in open(self.goals_file):
                self.goal_file_lines += 1
            self.data = []
            self.robot_set = []
            self.goal_set = []
            for self.goals_line_no in range(0, self.goal_file_lines):
                actual_line = getline(self.goals_file, self.goals_line_no)
                reader = goals_ifile.readline()
                curr = reader.find("(")
                next = reader.find(" ")
                self.goals_name = reader[curr+1:next]
                step = reader.find(")")
                self.goal_coord_id = reader[next+1:step]
                self.goals_capability_name = self.ontology_list['capability'][str(self.goals_name)]
                if not self.goals_line_no in self.goal_set:
                    self.goal_set.append(self.goals_line_no)
                robot_ifile = open(self.robot_file, "rw+")
                self.robot_file_lines = 0
                for robot_line in open(self.robot_file):
                    self.robot_file_lines += 1
                for self.robot_line_no in range(0, self.robot_file_lines):
                    actual_line = getline(self.robot_file, self.robot_line_no)
                    reader = robot_ifile.readline()
                    curr = reader.find("[")
                    next = reader.find("]")
                    self.robot_capability_set = reader[curr:next+1]
                    curr = reader.find("[")
                    self.robot_name = reader[:curr]
                    curr = reader.find("{")
                    next = reader.find("}")
                    self.wp_type = reader[curr+1:next]
                    if not self.robot_name in self.robot_set:
                        self.robot_set.append(self.robot_name)
                    if  self.goals_capability_name in self.robot_capability_set:
                        if self.wp_type in self.goal_coord_id:
                            self.data.append([self.robot_name, self.goals_line_no])

            self.capabilities_recognised = True
        except:
            print('Unable to read the indicated file')
    def load_goals_cap_ontology(self):
        with open(str(self.mission_constrains_directory)+str(self.ontology_directory), 'r') as stream:
            self.ontology_list = yaml.safe_load(stream)

if __name__ == '__main__':
    #===========================================================================
	# Main method
	#===========================================================================
    try:
        decentralised_robots_distribution = decentralised_robots_distribution()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
