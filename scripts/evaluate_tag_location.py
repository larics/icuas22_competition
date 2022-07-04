#!/usr/bin/env python3

import rospy
import rosbag
import math
import numpy as np
import os

class evaluate_tag_location:

	def __init__(self, bag_name, tag_location):
		self.tag_loc = tag_location
		self.bag = rosbag.Bag(bag_name);
		self.points_reconstruct = -1.0;

	def calculate_points(self):
		topic_exist = False
		for topic, msg, t in self.bag.read_messages(topics=["/red/tag_position_reconstructed"]):
			if topic == "/red/tag_position_reconstructed":
				topic_exist = True
				dist = np.linalg.norm(np.asarray([msg.x, msg.y, msg.z])-self.tag_loc)
				self.points_reconstruct = round(math.exp(-2*float(dist))*50.)/2
				print("TAG LOCATION: ", self.tag_loc)
				print("TAG RECONSTRUCT: ", np.asarray([msg.x, msg.y, msg.z]))
				print("DIST: ", dist)
				print("POINTS: ", self.points_reconstruct)
				break

		if not topic_exist:
			print("THERE IS NO TAG_POSITION data")

	def get_points_reconstruct(self):
		return self.points_reconstruct 


if __name__ == '__main__':
    rospy.init_node('evaluator')

    tag_location = np.asarray([2.724, -0.589, 0.917]) #final
    #tag_location = np.asarray([2.67, 0.03, 0.917]) #first

    teams_bag_name = {
    "SANTADRONE": [],
    "SUPAEROS_ION_lab": [],
    "Q-FORGE": [],
    "FIRE-RISC": [],
    "CVAR": []
    }

    take = "final" #final #take1

    team_parrent_folder = {
    "SANTADRONE": "/home/marko/ICUAS/"+take+"/santa/",
    "SUPAEROS_ION_lab": "/home/marko/ICUAS/"+take+"/ionlab/",
    "Q-FORGE": "/home/marko/ICUAS/"+take+"/qforge/",
    "FIRE-RISC": "/home/marko/ICUAS/"+take+"/fire_risc/",
    "CVAR": "/home/marko/ICUAS/"+take+"/cvar/"
    }

    team_score = {
    "SANTADRONE": -1,
    "SUPAEROS_ION_lab": -1,
    "Q-FORGE": -1,
    "FIRE-RISC": -1,
    "CVAR": -1
    }
    for team in team_parrent_folder:
    	print("TEAM NAME: ", team)

    	for path in os.listdir(team_parrent_folder[team]):
    		if os.path.isfile(os.path.join(team_parrent_folder[team], path)):
    			if path.endswith(".bag.active"):
    				teams_bag_name[team].append(path)

    	best_team_score = -1

    	for bag in teams_bag_name[team]:
    		print(bag)
    		team_eval = evaluate_tag_location(team_parrent_folder[team]+bag, tag_location)
    		team_eval.calculate_points()

    		if best_team_score == -1:
    			best_team_score = team_eval.get_points_reconstruct()
    		elif best_team_score < team_eval.get_points_reconstruct():
    			best_team_score = team_eval.get_points_reconstruct()

    		print("")

    	print("BEST TEAM SCORE: ", best_team_score)
    	team_score[team] = best_team_score

    	print("----------------------------------------------------------------------------")
    	print("")

    best_team_score = -1
    best_team_name = ""
    for team in team_parrent_folder:
    	if best_team_score == -1:
    		best_team_score = team_score[team]
    		best_team_name = team
    	elif best_team_score < team_score[team]:
    		best_team_score = team_score[team]
    		best_team_name = team

    print("----------------------------------------------------------------------------")
    print("----------------------------------------------------------------------------")
    print("BEST TEAM: ", best_team_name)
    print("SCORE: ", best_team_score)
    print("----------------------------------------------------------------------------")
    print("----------------------------------------------------------------------------")
    print("")
    print("")
    print("")
    print("")