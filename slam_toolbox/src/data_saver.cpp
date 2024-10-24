#include "slam_toolbox/data_saver.hpp"

DataSaver::DataSaver() : dataDir(""), localizationFileName(""), gtFileName(""), covFileName(""), latencyFileName("") {}

// Destructor that closes files
DataSaver::~DataSaver() {
    if (locFile.is_open()) locFile.close();
    if (gtFile.is_open()) gtFile.close();
    if (covFile.is_open()) covFile.close();
    if (latencyFile.is_open()) latencyFile.close();
}

void DataSaver::setDataDir(const std::string& relDataDirPath) {
    dataDir = ros::package::getPath("slam_toolbox") + "/" + relDataDirPath;
}

void DataSaver::setFileNames(const std::string& newLocFileName, const std::string& newGTFileName, 
                             const std::string& newCovFileName, const std::string& newLatencyFileName) {
    if (newLocFileName.empty() || newGTFileName.empty() || newCovFileName.empty() || newLatencyFileName.empty()) {
        ROS_ERROR("localization (SLAM pose), ground truth pose, covariance, and latency filenames cannot be empty");
    }
    // Close the current files, change filenames, and open new files if they are open
    if (newLocFileName != localizationFileName) {
        if (locFile.is_open()) locFile.close();
        localizationFileName = newLocFileName;
        locFile.open(dataDir + "/" + localizationFileName, std::ios::out | std::ios::trunc);
    }
    if (newGTFileName != gtFileName) {
        if (gtFile.is_open()) gtFile.close();
        gtFileName = newGTFileName;
        gtFile.open(dataDir + "/" + gtFileName, std::ios::out | std::ios::trunc);
    }
    if (newCovFileName != covFileName) {
        if (covFile.is_open()) covFile.close();
        covFileName = newCovFileName;
        covFile.open(dataDir + "/" + covFileName, std::ios::out | std::ios::trunc);
    }
    if (newLatencyFileName != latencyFileName) {
        if (latencyFile.is_open()) latencyFile.close();
        latencyFileName = newLatencyFileName;
        latencyFile.open(dataDir + "/" + latencyFileName, std::ios::out | std::ios::trunc);
    }
    // Check if files are successfully opened
    if (!locFile.is_open()) ROS_ERROR("Error opening localization (SLAM poses) file: %s", (dataDir + "/" + localizationFileName).c_str());
    if (!gtFile.is_open()) ROS_ERROR("Error opening ground truth file: %s", (dataDir + "/" + gtFileName).c_str());
    if (!covFile.is_open()) ROS_ERROR("Error opening Covariance file: %s", (dataDir + "/" + covFileName).c_str());
    if (!latencyFile.is_open()) ROS_ERROR("Error opening Latency file: %s", (dataDir + "/" + latencyFileName).c_str());
}

void DataSaver::saveData(const double timestamp, const geometry_msgs::Pose &pose, 
                         const karto::Matrix3 &covariance, const double latency) {
    saveLocalizationData(timestamp, pose);
    // saveGTData(gt_pose); // saved seperately in case a ground truth pose has not been made available yet
    saveCovarianceData(timestamp, covariance);
    saveLatencyData(timestamp, latency);
} 

// Save SLAM localized pose data in TUM format
void DataSaver::saveLocalizationData(const double timestamp, const geometry_msgs::Pose& pose) {
    locFile << std::fixed << std::setprecision(6);
    locFile << timestamp <<" " <<pose.position.x <<" " <<pose.position.y <<" " \
                <<pose.position.z <<" " <<pose.orientation.x <<" " <<pose.orientation.y \
                <<" " <<pose.orientation.z <<" " <<pose.orientation.w <<" " <<std::endl;
}

// Save ground truth gazebo_fake_localization-derived pose data in TUM format
void DataSaver::saveGTData(const geometry_msgs::TransformStamped &gt_pose_stamped) {
    locFile << std::fixed << std::setprecision(6);
    locFile << gt_pose_stamped.header.stamp.toSec() <<" " <<gt_pose_stamped.transform.translation.x <<" " \
            <<gt_pose_stamped.transform.translation.y <<" " <<gt_pose_stamped.transform.translation.z <<" " \
            <<gt_pose_stamped.transform.rotation.x <<" " <<gt_pose_stamped.transform.rotation.y \
            <<" " <<gt_pose_stamped.transform.rotation.z <<" " <<gt_pose_stamped.transform.rotation.w <<" " <<std::endl;
}

// Save Covariance data in flattened (row-major) format
void DataSaver::saveCovarianceData(const double timestamp, const karto::Matrix3& cov) {
    covFile << std::fixed << std::setprecision(6);  // Set float precision formatting
    covFile << timestamp <<" " <<cov(0, 0) <<" " <<cov(0, 1) <<" " <<cov(0, 2) <<" " <<cov(1, 0) \
                <<" " <<cov(1, 1) <<" " <<cov(1, 2) <<" " <<cov(2, 0) <<" " <<cov(2, 1) <<" " \
                <<cov(2, 2) <<std::endl;
}

// Save Latency along with timestamp
void DataSaver::saveLatencyData(const double timestamp, const double latency) {
    latencyFile << std::fixed << std::setprecision(6);  // Set float precision formatting
    latencyFile << timestamp << " " <<latency << std::endl;
}