#include "slam_toolbox/data_saver.hpp"

DataSaver::DataSaver() : dataDir(""), locFileName(""), gtFileName(""), covFileName(""), latencyFileName("") {}

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

void DataSaver::makeAndOpenFile(std::ofstream& file, const std::string& filepath) {
    std::filesystem::path dir = std::filesystem::path(filepath).parent_path();
    std::cout <<"----  DIR " <<dir << "----" <<std::endl;
    std::cout <<"Full path" <<filepath;
    std::filesystem::create_directories(dir);
    file.open(filepath, std::ios::out | std::ios::trunc);
}

void DataSaver::setFileNames(const std::string& newLocFileName, const std::string& newGTFileName, 
                             const std::string& newCovFileName, const std::string& newLatencyFileName) {
    ROS_INFO("In setFileNames: %s, %s, %s, %s", newLocFileName.c_str(), newGTFileName.c_str(), newCovFileName.c_str(), newLatencyFileName.c_str());
    if (newLocFileName.empty() || newGTFileName.empty() || newCovFileName.empty() || newLatencyFileName.empty()) {
        ROS_ERROR("localization (SLAM pose), ground truth pose, covariance, and latency filenames cannot be empty");
    }
    // Close the current files, change filenames, and open new files if they are open
    if (newLocFileName != locFileName) {
        if (locFile.is_open()) locFile.close();
        locFileName = newLocFileName;
        makeAndOpenFile(locFile, dataDir + "/" + locFileName);
    }
    if (newGTFileName != gtFileName) {
        if (gtFile.is_open()) gtFile.close();
        gtFileName = newGTFileName;
        makeAndOpenFile(gtFile, dataDir + "/" + gtFileName);
    }
    if (newCovFileName != covFileName) {
        if (covFile.is_open()) covFile.close();
        covFileName = newCovFileName;
        makeAndOpenFile(covFile, dataDir + "/" + covFileName);
    }
    if (newLatencyFileName != latencyFileName) {
        if (latencyFile.is_open()) latencyFile.close();
        latencyFileName = newLatencyFileName;
        makeAndOpenFile(latencyFile, dataDir + "/" + latencyFileName);
    }
    // Check if files are successfully opened
    if (!locFile.is_open()) ROS_ERROR("Error opening localization (SLAM poses) file: %s", (dataDir + "/" + locFileName).c_str());
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
                <<" " <<pose.orientation.z <<" " <<pose.orientation.w <<std::endl;
}

// Save ground truth gazebo_fake_localization-derived pose data in TUM format
void DataSaver::saveGTData(const geometry_msgs::TransformStamped &gt_pose_stamped) {
    gtFile << std::fixed << std::setprecision(6);
    gtFile << gt_pose_stamped.header.stamp.toSec() <<" " <<gt_pose_stamped.transform.translation.x <<" " \
            <<gt_pose_stamped.transform.translation.y <<" " <<gt_pose_stamped.transform.translation.z <<" " \
            <<gt_pose_stamped.transform.rotation.x <<" " <<gt_pose_stamped.transform.rotation.y \
            <<" " <<gt_pose_stamped.transform.rotation.z <<" " <<gt_pose_stamped.transform.rotation.w <<std::endl;
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