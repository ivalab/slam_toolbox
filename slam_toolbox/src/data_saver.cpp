#include "slam_toolbox/data_saver.hpp"

DataSaver::DataSaver() : dataDir(""), tumFileName(""), covFileName(""), latencyFileName("") {}

// Destructor that closes files
DataSaver::~DataSaver() {
    if (tumFile.is_open()) tumFile.close();
    if (covFile.is_open()) covFile.close();
    if (latencyFile.is_open()) latencyFile.close();
}

void DataSaver::setDataDir(const std::string& dataDirPath) {
    dataDir = dataDirPath;
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
    if (!tumFile.is_open()) ROS_ERROR("Error opening TUM file: (%s)", tumFileName.c_str());
    if (!covFile.is_open()) ROS_ERROR("Error opening Covariance file: (%s)", covFileName.c_str());
    if (!latencyFile.is_open()) ROS_ERROR("Error opening Latency file: (%s)", latencyFileName.c_str());
}

void DataSaver::saveData(const double timestamp, const geometry_msgs::Pose &pose, const karto::Matrix3 &covariance, const double latency) {
    saveTUMData(timestamp, pose);
    saveCovarianceData(timestamp, covariance);
    saveLatencyData(timestamp, latency);
} 

// Save pose data in TUM format
void DataSaver::saveTUMData(const double timestamp, const geometry_msgs::Pose& pose) {
    tumFile << std::fixed << std::setprecision(6);
    tumFile << timestamp <<" " <<pose.position.x <<" " <<pose.position.y <<" " \
                <<pose.position.z <<" " <<pose.orientation.x <<" " <<pose.orientation.y \
                <<" " <<pose.orientation.z <<" " <<pose.orientation.w <<" " <<std::endl;
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
    latencyFile << timestamp << " " <<latency << std::endl;
}