#!/usr/bin/env python3
# To hide the recurring TF_REPEATED_DATA warnings, use the following command. 
# The second sed command is to remove the buffer_core.cpp warnings that still persist, if you 
# concerned that this might filter out all tf_buffer_warnings and there are others that you may
# want to catch, remove this second filter and you will see occasional buffer_core.cpp warnings
# from when the TF_REPEATED_DATA warning prints out across different lines (like it sometimes does)
# but it will still be substantially less frequent than without any filter
# COMMAND \
#   2> >(sed -e '/TF_REPEATED_DATA/,/buffer_core\.cpp/d' -e '/buffer_core\.cpp/d' >&2) \
#   1> >(tee OUTPUT_FILEPATH)
# e.g.
    # rosrun custom_nav_bench_demo run_single_param_varying_batch2_experiment.py \
    #   2> >(sed -e '/TF_REPEATED_DATA/,/buffer_core\.cpp/d' -e '/buffer_core\.cpp/d' >&2) \
    #   1> >(tee runs/filtered_run_output_single_param_varying_batch2.txt)
import subprocess 
from nav_scripts.gazebo_master import MultiMasterCoordinator
from nav_scripts.testing_scenarios import TestingScenarios, GeneralScenario
from nav_scripts.controller_launcher import ControllerLauncher
import nav_turtlebot.turtlebot_impl
import time
import os
import rospy
import rospkg
from std_srvs.srv import Empty 
from copy import deepcopy
import math

pkg_path = rospkg.RosPack().get_path("custom_nav_bench_demo")

class ShortcutScenario1(GeneralScenario):
    name = "shortcut_bookshelves2"
    world = "shortcut_bookshelves2"

    def __init__(self, task):
        super(ShortcutScenario1, self).__init__(task=task)
        # Poses for gazebo_fake_localization
        # self.init_pose = [-0.1, 0, 1.57]
        # self.target_pose = [5.75, 0, 0]

        # Poses for slam_toolbox
        self.init_pose = [-0.1, 0, 1.57]
        self.target_pose = [0.21, -6.11, 1.57]

    def setupEnvironment(self):
        pass

    def getGazeboLaunchFile(self):
        return pkg_path + "/launch/gazebo_world.launch"

TestingScenarios.registerScenario(ShortcutScenario1)
ControllerLauncher.registerController(name="teb", filepath=pkg_path + "/launch/teb.launch")
# ControllerLauncher.registerController(name="teb_clearing", filepath=pkg_path + "/launch/teb_clearing.launch")

laser_turtlebot = pkg_path + "/launch/spawn_laser_turtlebot.launch"


def run_auto_tests(show_gazebo=False):
    start_time = time.time()
    master = MultiMasterCoordinator(num_masters=1,save_results=True,data_dir="~/simulation_data/single_param_varying_updated")
    master.start()
    
    def getTasks():
        '''
            General testing Setup for Initial Parameter testing

            Parameter Sweep
            - 5 Runs of each configuration (0-4 for seed)
            - parameters varied in isolation (not any multi-D grid searches yet)
            - 11 different values for each parameter
            - start at either defined min/max and then scale by factor of root(2)
            - There will be exceptions non-float parameters like throttle_scans,
              or use_scan_barycenter (initial testing will likely not reveal much
              about these params anyway though)

            Data Saving
            - Since the only thing varying is one parameter, the folder structure
              will simply be base_directory/{param_name}/{param_val}/ and then each
              seed will have its own set of loc and gt traj, latency, and cov files
              all under this folder for that trial
            - A copy of this script (run_experiments.py) will also be placed in the
              base directory of this test campaign becuase it will be useful for the
              analysis script

            ######### ERROR | NOTICE | WARNING | THIS IS BROKEN #########
            There are multiple errors with this and I am just running it to see if it tells us anything
            I think that as of right now, the SLAM map may be carrying drift across tests, the getTasks is
            broken whenever there is anything other than nested for loops (hence the patchwork fix in the 
            first for loop at the inner most level, and the instances where the trial fails are not being saved (I don't think).
            I am not sure about these though! What I am sure of is that some of the trials are missing,
            and in my limited testing before break, this behavior only occured on a run failute. The SLAM
            map drift comes from issues I saw occuring across runs, but the behavior during testing also 
            seemed somewhat inconsistent with the full program... I am really unsure of the root causes.
        '''
        


        root2 = math.sqrt(2)
        param_val = 0.0
        show_gazebo = False
        show_rviz = False # Defaults to true if not populated in task
        controller_args = {'global_planning_freq': 1, 'controller_freq': 5}
        slam_method = 'slam_toolbox' 
        dynamic_param_server = slam_method
        task_id = 0
        slam_param_defaults = {'throttle_scans': 1, 'map_update_interval': 0.02, 'resolution': .05, 'max_laser_range': 5.0, 'minimum_time_interval': 0.5, 
                               'use_scan_barycenter': True, 'minimum_travel_distance': 0.5, 'minimum_travel_heading': 0.5, 
                               'loop_match_minimum_response_coarse': .85, 'loop_match_minimum_response_fine': .95, 
                               'correlation_search_space_dimension': 0.2, 'correlation_search_space_resolution': 0.01, 
                               'correlation_search_space_smear_deviation': 0.1, 'angle_variance_penalty': 2.0, 'fine_search_angle_offset': 0.00175, 
                               'coarse_search_angle_offset': 0.175, 'coarse_angle_resolution': 0.0175, 'minimum_angle_penalty': 0.9, 
                               'minimum_distance_penalty': 0.5
                                }
        
        def gen_11_geometrically_spaced_vals(center_val, multiplier):
            vals = [None]*11
            param_val = center_val/(multiplier**5)
            for idx in range(0, 11):
                vals[idx] = param_val
                param_val*=multiplier
            return vals
        
        # correlation_search_space_dimension (Size (meters) of the search space for scan matching with the map around the initial pose estimate)
            # 0.03535 -> 1.131, passing through default of 0.2 over 11 values
        # correlation_search_space_resolution (scan matching search resolution (can't be greater than the dimension, and won't be (default dim is 0.2)))
            # 0.001767 (1.77mm) -> 0.056 (5.6cm), passing through default of 0.01 (1cm) over 11 values
        # correlation_search_space_smear_deviation (Standard deviation (in meters) for smear applied in the search space)
            # 0.01767 -> 0.5656, passing through default of 0.1 over 11 values
        # The rest are all done similarly and their ranges in DynamicParams.cfg have been checked/updated to allow for those ranges
        default_handling_params = {'correlation_search_space_dimension': 0.2, 'correlation_search_space_resolution': 0.01, 'correlation_search_space_smear_deviation': 0.1,
                                   'distance_variance_penalty': 1.0, 'angle_variance_penalty': 2.0, 'coarse_search_angle_offset': 0.175, 
                                   'coarse_angle_resolution': 0.0175, 'minimum_angle_penalty': 0.9, 'minimum_distance_penalty': 0.5
                                  }
        # just do coarse for now, not 'fine_search_angle_offset': 0.00175,
        
        for scenario in ['shortcut_bookshelves2']: # set
#           for global_planning_freq in [1]: # nav param - set
            for controller in ['teb']: # nav method - set for prelim testing
                # Run once with default params
                task = {'controller': controller, 'slam_method': slam_method, 'seed': 0, 
                            'scenario': scenario, 'robot': laser_turtlebot, 'robot_impl': 'turtlebot',
                            'record': False, 'world_args': {"gazebo_gui":show_gazebo}, 'rviz': show_rviz,
                            'controller_args': controller_args,
                            'slam_args': deepcopy(slam_param_defaults),
                            'data_dir_extension': f'default_params',
                            'dynamic_param_server': dynamic_param_server,
                        }
                # yield task
                # SLAM Params
                for seed in range(0, 5): # Repeat Trial 5 Times per Param
                    task = {'controller': controller, 'slam_method': slam_method, 'seed': seed, 
                            'scenario': scenario, 'robot': laser_turtlebot, 'robot_impl': 'turtlebot',
                            'record': False, 'world_args': {"gazebo_gui":show_gazebo}, 'rviz': show_rviz,
                            'controller_args': controller_args,
                            'dynamic_param_server': dynamic_param_server, 'task_id': task_id
                        }


                    # I am confused about what is happening, but for some reason, the task processor
                    # seems to automatically ingest (and discard) the first task for a given seed, so 
                    # this extra for loop was added to stop dropping the first iteration of the first for loop.
                    # It also does seem to have to be in a for look for it to work... I have no idea why
                    for idx in range(0, 1):
                        task['slam_args'] = deepcopy(slam_param_defaults)
                        task['data_dir_extension'] = f'trash/{task_id}'
                        task_id += 1
                        yield task

                    # Run all testing for parameters in default_handling_params that simply use 11 evenly geometrically 
                    # distributed values around their default values
                    for param_name, defualt_val in default_handling_params.items():
                        print(f"Running {param_name} parameter tests")
                        for param_val in gen_11_geometrically_spaced_vals(center_val=defualt_val, multiplier=root2):
                            task['slam_args'] = deepcopy(slam_param_defaults)
                            task['slam_args'][param_name] = param_val
                            task['data_dir_extension'] = f'{param_name}/{param_val:.5f}'
                            task_id += 1
                            yield task
                    ### Run through the different independently tested parameters ###

                    ## Parameters that require special value ranges (not just 11 evenly geometrically distributed values around their default) ##
                    # max_laser_range (map update time, not new scan processing/pose-graph update)
                        # 1.5625m (5.125ft) -> 50m (164ft) over 11 values, **DOES NOT Pass through Default Value (5m)!
                    param_val = 50.0/(root2**10)
                    for idx in range(0, 11):
                        task['slam_args'] = deepcopy(slam_param_defaults)
                        task['slam_args']['max_laser_range'] = param_val
                        task['data_dir_extension'] = f'max_laser_range/{param_val:.5f}'
                        param_val*=root2
                        task_id += 1
                        yield task

                    
                    

    master.add_tasks(tasks=getTasks())
    master.wait_to_finish()
    master.shutdown()
    end_time = time.time()
    print(f"Total time: {end_time - start_time}")


if __name__ == "__main__":

    run_auto_tests(show_gazebo=True)

