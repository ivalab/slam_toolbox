## Floormap to SLAM Map

1. **Floorplan -> Route-Graph**
   1. Use Alex's script to generate the route-graph.
      1. Input: `map.yaml & map.pgm`
      2. Output: `graph.json`
2. Configure [launch file](../../launch/floorplan/floorplan_to_slam_map.launch) to set the **map** file and **route-graph** file, and **output_dir**.
3. Start the launch file.

        roslaunch slam_toolbox floorplan_to_scan.launch

4. Set the robot start pose in Rviz.
5. The mapping will start shortly ...
6. Terminate the launch file will automatically save the generate pose-graph.



### ISSUE Tracking
- **Scan Generation**
  - Lower the **map resolution** value if the scans are curved (not aligned with the walls), you also need to resize the map to be bigger.
- **Mapping Perforamnce**
  - Lower down the scan rate `(10 -> 5 -> 1Hz)` in the launch file can help.
  - **TODO**: slam_toolbox has a few tunable parameters to trust more of the predicted poses (odom).