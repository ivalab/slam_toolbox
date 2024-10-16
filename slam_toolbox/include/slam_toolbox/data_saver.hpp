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
    std::string dataDir;

    // Internal functions for actually writing the data to the different files
    void saveTUMData(const double timestamp, const geometry_msgs::Pose& pose);
    void saveCovarianceData(const double timestamp, const karto::Matrix3& cov);
    void saveLatencyData(const double timestamp, const double latency);

public:
    DataSaver();
    ~DataSaver();
    // Needed for multirun script
    void setDataDir(const std::string& dataDirPath);
    void setFileNames(const std::string& newTumFileName, const std::string& newCovFileName, const std::string& newLatencyFileName);

    // Functions to save data one timestamp of data
    void saveData(const double timestamp, const geometry_msgs::Pose& pose, const karto::Matrix3& covariance, const double latency);
};

#endif 
