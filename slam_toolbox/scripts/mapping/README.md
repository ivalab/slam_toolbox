### Offline Mapping Pipeline

When collected a rosbag with topics of /scan and /odom (/tf) using SLAMbot, the script supports to run an offline mapping for finer 2D reconstruction.
Once finished, the following will be saved to the output directory:
- Occ map
- SLAM Toolbox Pose Graph
- SLAM Toolbox Keyframe Poses
- A dense spline-fitted keyframe poses (1. both in body and camera frames, extrinsics are hard-coded. 2. can be turned off in the script)

Steps

```bash

python run_mapping.py # Assure to set the rosbag in the script


```
The spline fitting node also supports offline batch processing after mapping. Presumly, the keyframes poses are saved under DATA_DIR/SEQ1/slam_toolbox/xxx_poses.txt
```
rosrun slam_toolbox spline_fitting_node -d DATA_DIR -s SEQ1 SEQ2
```
It will save the processed fitted poses under DATA_DIR/gt_poses/SEQ1_body.txt (SEQ_cam.txt)
