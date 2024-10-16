#include "slam_toolbox/DataSaver.hpp"

DataSaver::DataSaver(const std::string& dataDir) : dataDir(dataDir), tumFileName(""), covFileName(""), latencyFileName("") {}

// Destructor that closes files
DataSaver::~DataSaver() {
    if (tumFile.is_open()) tumFile.close();
    if (covFile.is_open()) covFile.close();
    if (latencyFile.is_open()) latencyFile.close();
}

void DataSaver::setFileNames(const std::string& newTumFileName, const std::string& newCovFileName, const std::string& newLatencyFileName) {
    if (newTumFileName.empty() || newCovFileName.empty() || newLatencyFileName.empty()) {
        ROS_ERROR("TUM (pose), covariance, and latency filenames cannot be empty");
    }
    // Close the current files, change filenames, and open new files if they are open
    if (newTumFileName != tumFileName) {
        if (tumFile.is_open()) tumFile.close();
        tumFileName = newTumFileName;
        tumFile.open(dataDir + "/" + tumFileName, std::ios::out | std::ios::trunc);
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
    if (!tumFile.is_open()) ROS_ERROR("Error opening TUM file: (%s)", tumFileName);
    if (!covFile.is_open()) ROS_ERROR("Error opening Covariance file: (%s)", covFileName);
    if (!latencyFile.is_open()) ROS_ERROR("Error opening Latency file: (%s)", latencyFileName);
}

void DataSaver::saveData(double timestamp, geometry_msgs::pose &pose, karto::Matrix3 &covariance, ros::Time &latency) {
    saveTUMData(timestamp, pose);
    saveCovarianceData(timestamp, covariance);
    saveLatencyData(timestamp, latency);
} 

// Save pose data in TUM format
void DataSaver::saveTUMData(double timestamp, const geometry_msgs::pose& pose) {
    tumFile << std::fixed << std::setprecision(6);
    tumFile << timestamp <<" " <<pose.translation.getX <<" " <<pose.translation.getY <<" " \
                <<pose.translation.getZ <<" " <<pose.orientation.getX <<" " <<pose.orientation.getY \
                <<" " <<pose.orientation.getZ <<" " <<pose.orientation.getW <<" " <<std::endl;
}

// Save Covariance data in flattened (row-major) format
void DataSaver::saveCovarianceData(double timestamp, const karto::Matrix3& cov) {
    covFile << std::fixed << std::setprecision(6);  // Set float precision formatting
    covFile << timestamp <<" " <<cov(0, 0) <<" " <<cov(0, 1) <<" " <<cov(0, 2) <<" " <<cov(1, 0) \
                <<" " <<cov(1, 1) <<" " <<cov(1, 2) <<" " <<cov(2, 0) <<" " <<cov(2, 1) <<" " \
                <<cov(2, 2) <<std::endl;
}

// Save Latency along with timestamp
void DataSaver::saveLatencyData(double timestamp, const ros::Time& latency) {
    latencyFile << timestamp << " " <<latencyData.toNSec() << std::endl;
}