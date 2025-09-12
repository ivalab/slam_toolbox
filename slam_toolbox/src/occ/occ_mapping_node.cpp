/**
 * @file occ_mapping.cpp
 * @author Yanwei Du (yanwei.du@gatech.edu)
 * @brief None
 * @version 0.1
 * @date 04-13-2026
 * @copyright Copyright (c) 2026
 */

#include "slam_toolbox/occ/occ_mapping.hpp"

namespace slam_toolbox {

/*****************************************************************************/
OccMapping::OccMapping(ros::NodeHandle& nh)
    : SlamToolbox(nh)
/*****************************************************************************/
{
    ssClear_ = nh.advertiseService("clear_queue",
                                   &OccMapping::clearQueueCallback, this);
    threads_.push_back(
        std::make_unique<boost::thread>(boost::bind(&OccMapping::run, this)));

    loadPoseGraphByParams(nh);

    // Subscribers.

    nh.param("use_loaded_pose", use_loaded_pose_, false);
    if (!use_loaded_pose_) {
        kf_sub_ = nh.subscribe("/visual_slam/pub_keyframe_array_converted", 1,
                               &OccMapping::keyframePoseArrayCallback, this);
    } else {
        std::string kf_poses_filepath;
        nh.getParam("kf_poses_filepath", kf_poses_filepath);
        // kf_poses_ = loadKeyFramePosesFromFile(kf_poses_filepath);
        std::vector<Eigen::VectorXd> samples;
        ov_core::DatasetReader::load_simulated_trajectory(kf_poses_filepath,
                                                          samples);
        if (samples.size() < 10u) {
            std::cout << "Insufficient samples: " << samples.size() << "\n";
            use_loaded_pose_ = false;
        } else {
            // Fit splines.
            spline_.feed_trajectory(samples);
        }
    }
}

std::vector<OccMapping::StampedPose3> OccMapping::loadKeyFramePosesFromFile(
    const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file) {
        std::cout << "Error: Uable to open keyframe pose file ... "
                  << std::endl;
        std::cout << "Error: " << filepath << std::endl;
        std::exit(EXIT_FAILURE);
    }

    std::vector<StampedPose3> poses;
    std::string               line;
    while (std::getline(file, line)) {
        std::istringstream s(line);
        s >> std::ws;
        if (s.eof() || s.peek() == '#') {
            continue;
        }

        StampedPose3 pose;
        if (!(s >> pose.timestamp >> pose.tx >> pose.ty >> pose.tz >> pose.qx >>
              pose.qy >> pose.qz >> pose.qw)) {
            std::cout << "Warning: invalid pose line, skipping: " << line
                      << std::endl;
            continue;
        }

        normalizeQuat(pose.qx, pose.qy, pose.qz, pose.qw);
        poses.emplace_back(pose);
    }

    return poses;
}

/*****************************************************************************/
void OccMapping::run()
/*****************************************************************************/
{
    ros::Rate r(100);
    while (ros::ok()) {
        if (!isPaused(PROCESSING)) {
            PosedScan scan_w_pose(
                nullptr, karto::Pose2());  // dummy, updated in critical section
            bool queue_empty = true;
            {
                boost::mutex::scoped_lock lock(q_mutex_);
                queue_empty = q_.empty();
                if (!queue_empty) {
                    scan_w_pose = q_.front();
                    q_.pop();

                    if (q_.size() > 10) {
                        ROS_WARN_THROTTLE(10.,
                                          "Queue size has grown to: %i. "
                                          "Recommend stopping until message is "
                                          "gone if online mapping.",
                                          (int)q_.size());
                    }
                }
            }
            if (!queue_empty) {
                addScan(getLaser(scan_w_pose.scan), scan_w_pose);
                continue;
            }
        }

        r.sleep();
    }
}

/*****************************************************************************/
void OccMapping::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
/*****************************************************************************/
{
    if (global_correction_) {
        return;
    }
    // store scan header
    scan_header_ = scan->header;

    // no odom info
    karto::Pose2 pose;
    if (use_loaded_pose_) {
        // std::cout << "To be implemented !!! " << std::endl;
        Eigen::Matrix3d Rot;
        Eigen::Vector3d trans;
        bool success = spline_.get_pose(scan->header.stamp.toSec(), Rot, trans);
        if (!success) {
            return;
        }
        double             out_x = trans.x();
        double             out_y = trans.y();
        Eigen::Quaterniond s(Rot);
        std::cout << "quat = " << s.coeffs().transpose() << std::endl;
        double out_theta = yawFromQuat(s.x(), s.y(), s.z(), s.w());
        pose             = Pose2(out_x, out_y, out_theta);

        std::cout << "Set scan pose: theta = " << out_x << ", " << out_y << ", "
                  << out_theta * 180.0 / M_PI << std::endl;

    } else {
        // Use odom info.
        if (!pose_helper_->getOdomPose(pose, scan->header.stamp)) {
            return;
        }
    }

    // ensure the laser can be used
    karto::LaserRangeFinder* laser = getLaser(scan);

    if (!laser) {
        ROS_WARN_THROTTLE(5.,
                          "Failed to create laser device for"
                          " %s; discarding scan",
                          scan->header.frame_id.c_str());
        return;
    }

    {
        boost::mutex::scoped_lock lock(q_mutex_);
        q_.push(PosedScan(scan, pose));
    }
    return;
}

/*****************************************************************************/
bool OccMapping::clearQueueCallback(
    slam_toolbox_msgs::ClearQueue::Request&  req,
    slam_toolbox_msgs::ClearQueue::Response& resp)
/*****************************************************************************/
{
    ROS_INFO("OccMapping: Clearing all queued scans to add to map.");
    while (!q_.empty()) {
        q_.pop();
    }
    resp.status = true;
    return true;
}

/*****************************************************************************/
void OccMapping::keyframePoseArrayCallback(
    const semantic_msgs::KeyFramePoseArray::ConstPtr& kfs_msg)
/*****************************************************************************/
{
    ROS_INFO(
        "OCC-Map-Stitching: KeyFrame Array Received, updating scan poses ...");
    std::vector<StampedPose3> kfs;
    for (const auto& kf_msg : kfs_msg->poses) {
        StampedPose3 kf;
        kf.timestamp = kf_msg.pose.header.stamp.toSec();
        kf.tx        = kf_msg.pose.pose.position.x;
        kf.ty        = kf_msg.pose.pose.position.y;
        kf.tz        = kf_msg.pose.pose.position.z;
        kf.qx        = kf_msg.pose.pose.orientation.x;
        kf.qy        = kf_msg.pose.pose.orientation.y;
        kf.qz        = kf_msg.pose.pose.orientation.z;
        kf.qw        = kf_msg.pose.pose.orientation.w;
        kfs.emplace_back(kf);
    }
    global_correction_ = true;
    this->correctPosesWithVisualPoseGraph(kfs);
    global_correction_ = false;
    ROS_INFO("OCC-Map-Stitching: Occ Map Updated !");
}

/*****************************************************************************/
void OccMapping::correctPosesWithVisualPoseGraph(
    const std::vector<StampedPose3>& kfs)
/*****************************************************************************/
{
    karto::MapperSensorManager* scan_manager =
        this->smapper_->getMapper()->GetMapperSensorManager();
    karto::Name name("Custom Described Lidar");
    auto&       indexed_scans   = scan_manager->GetScans(name);
    int         corrected_count = 0;

    std::vector<karto::LocalizedRangeScan*> scans_to_remove;
    for (auto& iter : indexed_scans) {
        if (iter.second == NULL) {
            continue;
        }
        auto scan = iter.second;

        karto::Pose2 corrected_pose;
        bool         flag =
            interpolatePose(kfs, scan->GetTime(), corrected_pose, false, true);
        if (flag) {
            scan->SetCorrectedPoseAndUpdate(corrected_pose);
            corrected_count++;
        } else {
            scans_to_remove.push_back(scan);
        }
    }
    std::cout << "OccMap Correction: " << corrected_count << " out of "
              << indexed_scans.size() << " scans have been corrected with "
              << kfs.size() << " visual keyframe poses." << std::endl;
    // for (auto scan : scans_to_remove) {
    // scan_manager->RemoveScan(scan);
    // }
}

/*****************************************************************************/
bool OccMapping::deserializePoseGraphCallback(
    slam_toolbox_msgs::DeserializePoseGraph::Request&  req,
    slam_toolbox_msgs::DeserializePoseGraph::Response& resp)
/*****************************************************************************/
{
    if (req.match_type == procType::LOCALIZE_AT_POSE) {
        ROS_ERROR(
            "Requested a localization deserialization "
            "in non-localization mode.");
        return false;
    }
    return SlamToolbox::deserializePoseGraphCallback(req, resp);
}

}  // namespace slam_toolbox

int main(int argc, char** argv) {
    ros::init(argc, argv, "occ_mapping");
    ros::NodeHandle nh("~");
    ros::spinOnce();

    int stack_size;
    if (nh.getParam("stack_size_to_use", stack_size)) {
        ROS_INFO("Node using stack size %i", (int)stack_size);
        const rlim_t  max_stack_size = stack_size;
        struct rlimit stack_limit;
        getrlimit(RLIMIT_STACK, &stack_limit);
        if (stack_limit.rlim_cur < stack_size) {
            stack_limit.rlim_cur = stack_size;
        }
        setrlimit(RLIMIT_STACK, &stack_limit);
    }

    slam_toolbox::OccMapping occ(nh);

    ros::spin();
    return 0;
}