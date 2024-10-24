#!/usr/bin/env python

import rospy

import dynamic_reconfigure.client

def callback(config):
    rospy.loginfo("Setting params to {loop_match_minimum_response_coarse}, {loop_match_minimum_response_fine}, {minimum_time_interval},\
                  {minimum_travel_distance}, {minimum_travel_heading}, {correlation_search_space_dimension}, \
                  {correlation_search_space_resolution}, {correlation_search_space_smear_deviation} \
                  %nDirs: {loc_file_name_}, {cov_file_name}, {latency_file_name}".format(**config)
                 )

# loop_match_minimum_response_coarse loop_match_minimum_response_fine minimum_time_interval minimum_travel_distance minimum_travel_heading 
# correlation_search_space_dimension correlation_search_space_resolution correlation_search_space_smear_deviation

if __name__ == "__main__":
    rospy.init_node("dynamic_params_client_test")

    client = dynamic_reconfigure.client.Client("slam_toolbox", timeout=30, config_callback=callback)
    r = rospy.Rate(0.2)
    x = 0.01
    alternator = False
    trial_num = 1
    while not rospy.is_shutdown():
        x += 0.25
        if (alternator):
            trial_num += 1
        alternator = not alternator
        if x>10:
            x=0.01
        client.update_configuration({"loop_match_minimum_response_coarse":11, "minimum_travel_distance":(10-x), 
                                     "loc_file_name_" : "loc_poses_%d.txt"%(trial_num), 
                                     "gt_file_name_" : "gt_poses_%d.txt"%(trial_num), 
                                     "cov_file_name" : "covariances_%d.txt"%(trial_num), 
                                     "latency_file_name" : "latencies_%d.txt"%(trial_num)})
        r.sleep()