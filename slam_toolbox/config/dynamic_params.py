#!/usr/bin/env python
PACKAGE = "slam_toolbox"
from dynamic_reconfigure.parameter_generator_catkin import *
gen = ParameterGenerator()

# need to find out what reasonable mins/maxs are for all these
#        param name, type, level, description, defualt, min, max
#                            -(the levels of changed parameters are bitwise ORed and passed as a param to the callback)
gen.add("loop_match_minimum_response_coarse", double_t, 0, "GOOD DESCRIPTION NEEDED", .85, 0.01, 10)
gen.add("loop_match_minimum_response_fine", double_t, 0, "GOOD DESCRIPTION NEEDED", .95, 0.01, 10)
gen.add("minimum_time_interval", double_t, 0, "The minimum duration of time between scans to be processed in synchronous mode", 0.5, 0.01, 10)
gen.add("minimum_travel_distance", double_t, 0, "Minimum distance of travel before processing a new scan", 0.5, 0.01, 10)
gen.add("minimum_travel_heading", double_t, 0, "Minimum changing in heading to justify an update", 0.5, 0.01, 10)
gen.add("correlation_search_space_dimension", double_t, 0, "Search grid size to do scan correlation over", 0.5, 0.01, 10)
gen.add("correlation_search_space_resolution", double_t, 0, "Search grid resolution to do scan correlation over", 0.01, 0.0001, 1)
gen.add("correlation_search_space_smear_deviation", double_t, 0, "Amount of multimodal smearing to smooth out responses", 0.1, 0.001, 10)

# gen.add("str_param",    str_t,    0, "A string parameter",  "Hello World")
# gen.add("bool_param",   bool_t,   0, "A Boolean parameter",  True)
exit(gen.generate(PACKAGE, "slam_toolbox", "dynamic_params"))