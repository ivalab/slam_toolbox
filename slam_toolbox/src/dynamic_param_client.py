#!/usr/bin/env python

import rospy

import dynamic_reconfigure.client

def callback(config):
    rospy.loginfo("Setting params to {loop_match_minimum_response_coarse}, {loop_match_minimum_response_fine}, {minimum_time_interval},\
                  {minimum_travel_distance}, {minimum_travel_heading}, {correlation_search_space_dimension}, \
                  {correlation_search_space_resolution}, {correlation_search_space_smear_deviation}".format(**config)
                 )

# loop_match_minimum_response_coarse loop_match_minimum_response_fine minimum_time_interval minimum_travel_distance minimum_travel_heading 
# correlation_search_space_dimension correlation_search_space_resolution correlation_search_space_smear_deviation

if __name__ == "__main__":
    rospy.init_node("dynamic_params_client_test")

    client = dynamic_reconfigure.client.Client("slam_toolbox", timeout=30, config_callback=callback)
    r = rospy.Rate(.5)
    x = 0.01
    b = False
    while not rospy.is_shutdown():
        x += 0.25
        if x>10:
            x=0.01
        client.update_configuration({"loop_match_minimum_response_coarse":x, "minimum_travel_distance":(10-x), "correlation_search_space_resolution":(x/10)})
        r.sleep()