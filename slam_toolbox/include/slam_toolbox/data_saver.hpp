#ifndef DATASAVER_H
#define DATASAVER_H

#include <fstream>
#include <filesystem>
#include <string>

#include "ros/ros.h"
#include "ros/package.h"
#include "karto_sdk/Karto.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>

/*
Manages the saving of data from SLAMToolbox for later analysis.
The pose (saved in TUM format), 3x3 Covariance Matrix (saved in a row-major 9-element vector), 
and laserscan processing latency (in ns) are all saved to seperate files.
For multi-run uses, update the filenames as necessary (for the runs) using changeFileNames
*/
class DataSaver {
private:
    // These will change during the multirun script!
    std::ofstream locFile, gtFile, covFile, latencyFile;
    std::string locFileName, gtFileName, covFileName, latencyFileName;
    std::string dataDir;

    // Internal functions for actually writing the data to the different files
    void saveLocalizationData(const double timestamp, const geometry_msgs::Pose& pose);
    void saveCovarianceData(const double timestamp, const karto::Matrix3& cov);
    void saveLatencyData(const double timestamp, const double latency);

    void makeAndOpenFile(std::ofstream& file, const std::string& filepath);

public:
    DataSaver();
    ~DataSaver();
    // Needed for multirun script
    void setDataDir(const std::string& relDataDirPath);
    void setFileNames(const std::string& newLocFileName, const std::string& newGTFileName, const std::string& newCovFileName, const std::string& newLatencyFileName);

    // Functions to save data one timestamp of data
    // Ground truth data saved seperately in case a ground truth pose has not been made available yet
    void saveData(const double timestamp, const geometry_msgs::Pose &pose, 
                         const karto::Matrix3 &covariance, const double latency);
    // Save ground truth data from gazebo_fake_localization
    // Saved seperately in case a ground truth pose has not been made available yet
    void saveGTData(const geometry_msgs::TransformStamped &gt_pose_stamped);
};

#endif 
