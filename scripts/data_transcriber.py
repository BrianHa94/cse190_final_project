#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float32, Bool
from cse_190_assi_3.msg import AStarPath, PolicyList
import json
import image_util
from read_config import read_config

class RobotLogger():
    def __init__(self):
        rospy.init_node("robot_logger")
        self.policy_result = rospy.Subscriber(
                "/results/policy_list",
                PolicyList,
                self.handle_mdp_policy_data
        )
        self.simulation_complete_sub = rospy.Subscriber(
                "/map_node/sim_complete",
                Bool,
                self.handle_shutdown
        )
        self.init_files()
        self.config = read_config()
        self.generate_video = self.config["generate_video"] == 1
        rospy.spin()

    def init_files(self):
        self.iteration_number = 0

    def convert_list_to_2d_array(self, policy_list):
        x, y = self.config["map_size"]
        return [policy_list[i : i + y] for i in xrange(0, len(policy_list), y)]

    def handle_mdp_policy_data(self, policy_list):
        if self.generate_video:
            data_to_publish = self.convert_list_to_2d_array(policy_list.data)
            image_util.save_image_for_iteration(data_to_publish, self.iteration_number)
            self.iteration_number += 1

    def handle_shutdown(self, message):
        print "sim complete!", message.data
        if self.generate_video:
            image_util.generate_video(self.iteration_number)
        

if __name__ == '__main__':
    rl = RobotLogger()
