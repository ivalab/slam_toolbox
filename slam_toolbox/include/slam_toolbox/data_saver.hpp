#ifndef DATASAVER_H
#define DATASAVER_H

#include <fstream>
#include <string>

#include "ros/ros.h"
#include "karto_sdk/Karto.h"
#include <geometry_msgs/Pose.h>

/*
Manages the saving of data from SLAMToolbox for later analysis.
The pose (saved in TUM format), 3x3 Covariance Matrix (saved in a row-major 9-element vector), 
and laserscan processing latency (in ns) are all saved to seperate files.
For multi-run uses, update the filenames as necessary (for the runs) using changeFileNames
*/
class DataSaver {
private:
    // These will change during the multirun script!
    std::ofstream tumFile;
    std::ofstream covFile;
    std::ofstream latencyFile;
    std::string tumFileName;
    std::string covFileName;
    std::string latencyFileName;
    std::stromg dataDir;

    // Internal functions for actually writing the data to the different files
    void saveTUMData(double timestamp, const geometry_msgs::pose& pose);
    void saveCovarianceData(double timestamp, const karto::Matrix3& cov);
    void saveLatencyData(double timestamp, const ros::Time& latency);

public:
    DataSaver(const std::string& dataDir);
    ~DataSaver();
    // Needed for multirun script
    void setFileNames(const std::string& newTumFileName, const std::string& newCovFileName, const std::string& newLatencyFileName);

    // Functions to save data one timestamp of data
    void saveData(double timestamp, geometry_msgs::pose& pose, karto::Matrix3& covariance, ros::Time& latency);
};

#endif 
