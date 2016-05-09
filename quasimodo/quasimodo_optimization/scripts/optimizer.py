#! /usr/bin/env python

import roslib; roslib.load_manifest('quasimodo_optimization')
import rospy
import os
import json

import actionlib
import dynamic_reconfigure.client

import quasimodo_optimization.msg
import quasimodo_msgs.msg

class QuasimodoOptimizer(object):
    # create messages that are used to publish feedback/result
    #_feedback = topic_logger.msg.topicLoggerFeedback()
    #_result = topic_logger.msg.topicLoggerResult()

    # how should we store the results? same as always I guess,
    # one map with entries per label and success / all pairs

    def __init__(self, name):
        self.bag_folder = os.path.abspath('/media/nbore/My\ Passport/quasimodo_bags')
        self.labels = ['fire_extinguisher', 'cereal', 'router_box', 'muesli_box', 'owl', 'fruit_basket', 'microwave']
        self.bags = [os.path.join(self.bag_folder, label + '_fused.bag') for label in self.labels]
        self.sub = rospy.Subscriber("/retrieved_labels", quasimodo_msgs.msg.string_array, self.callback)
        self.client = client = actionlib.SimpleActionClient('rosbag_player', quasimodo_optimization.msg.rosbag_playAction)
        self.client.wait_for_server()
        self.current_label = ''
        self.rates = {}
        rospy.loginfo('Server is up')

    # for one parameter setting
    def evaluate_parameters(self, iss_model_resolution, pfhrgb_radius_search, counter):
        self.rates = {}

        # Update the paramters to the next setting
        client = dynamic_reconfigure.client.Client("quasimodo_retrieval_node", timeout=30)
        client.update_configuration({"iss_model_resolution": iss_model_resolution, "pfhrgb_radius_search": pfhrgb_radius_search})

        for (bag, label) in zip(self.bags, self.labels):
            self.current_label = label
            goal = quasimodo_optimization.msg.rosbag_playGoal(command='start', file=bag)
            self.client.send_goal(goal)
            self.client.wait_for_result()

        total_correct = 0
        total = 0
        for key, value in self.rates.iteritems():
            total_correct = total_correct + value[0]
            total = total + value[1]
        self.rates['total'] = (total_correct, total, float(total_correct)/float(total))

        summary = (iss_model_resolution, pfhrgb_radius_search, self.rates)
        with open(str(counter) + '.json', 'w') as f:
            json.dump(summary, f)

    def callback(self, msg):
        correct = 0
        for label in msg.strings:
            if label == self.current_label:
                correct = correct+1
        if self.current_label in self.rates:
            self.rates[self.current_label] = (self.rates[self.current_label][0] + correct,
                                              self.rates[self.current_label][1] + len(msg.strings),
                                              float(self.rates[self.current_label][0])/float(self.rates[self.current_label][1]))
        else:
            self.rates[self.current_label] = (correct, len(msg.strings), float(correct)/float(len(msg.strings)))

    def run(self):
        #gen.add("iss_model_resolution", double_t, 0, "A double parameter", 0.01, 0.005, 0.02)
        #gen.add("pfhrgb_radius_search", double_t, 0, "A double parameter", 0.06, 0.03, 0.08)
        iss_resolutions = [0.002, 0.0025, 0.003, 0.0035, 0.004, 0.0045, 0.005]
        pfhrgb_resolutions = [0.04]
        counter = 0
        for ir in iss_resolutions:
            for pr in pfhrgb_resolutions:
                self.evaluate_parameters(ir, pr, counter)
                counter = counter + 1

if __name__ == '__main__':
    rospy.init_node('optimizer')
    osrv = QuasimodoOptimizer(rospy.get_name())
    osrv.run()
