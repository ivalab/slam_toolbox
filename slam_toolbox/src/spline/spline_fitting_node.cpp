/**
 * @file test_spline.cpp
 * @author Yanwei Du (yanwei.du@gatech.edu)
 * @brief None
 * @version 0.1
 * @date 09-01-2025
 * @copyright Copyright (c) 2025
 */

#include <cmath>
#include <deque>
#include <iomanip>
#include <sstream>
#include <unistd.h>
#include <vector>
#include <iostream>

#include "slam_toolbox/spline/BSplineSE3.h"
#include "slam_toolbox/spline/dataset_reader.h"
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;
namespace po = boost::program_options;
using namespace ov_core;

namespace internal {
struct StampedPose {
  double timestamp, tx, ty, tz, qx, qy, qz, qw;

  StampedPose(double _timestamp, const Eigen::Matrix3d &R, const Eigen::Vector3d &t) {
    timestamp = _timestamp;
    tx = t.x();
    ty = t.y();
    tz = t.z();
    Eigen::Quaterniond quat(R);
    qx = quat.x();
    qy = quat.y();
    qz = quat.z();
    qw = quat.w();
  }

  friend std::ostream &operator<<(std::ostream &os, const StampedPose &s) {
    os << std::fixed;
    os << std::setprecision(10) << s.timestamp << " " << std::setprecision(6) << s.tx << " " << s.ty << " " << s.tz << " " << s.qx << " "
       << s.qy << " " << s.qz << " " << s.qw;
    return os;
  }

  static std::string header() { return "# timestamp tx ty tz qx qy qz qw"; }
};

void save(const std::string &filename, const std::vector<StampedPose> &poses) {
  std::ofstream myfile(filename);
  myfile << StampedPose::header() << "\n";
  for (const auto &p : poses) {
    myfile << p << "\n";
  }
  myfile.close();
}

struct Config
{
    std::string data_dir;
    std::vector<std::string> sequences;
};

Config ParseArgs(int argc, char** argv)
{
    Config config;

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "Print help message")
        ("data_dir,d", po::value<std::string>(&config.data_dir)->required(),
            "Path to dataset directory")
        ("sequences,s",
            po::value<std::vector<std::string>>()->multitoken()->required(),
            "Sequence names (space or comma separated)");

    po::variables_map vm;

    try
    {
        po::store(po::parse_command_line(argc, argv, desc), vm);

        if (vm.count("help"))
        {
            std::cout << desc << std::endl;
            std::exit(EXIT_SUCCESS);
        }

        po::notify(vm);

        // Retrieve sequences
        config.sequences = vm["sequences"].as<std::vector<std::string>>();
//        config.sequences = SplitCommaSeparated(raw_sequences);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << "\n\n";
        std::cerr << desc << std::endl;
        std::exit(EXIT_FAILURE);
    }

    return config;
}

// Find filename.
std::string findFiles(const std::string& dir, const std::string& partialName) {
    fs::path directory(dir);
    if (!fs::exists(directory) || !fs::is_directory(directory)) return "";

    for (fs::recursive_directory_iterator it(directory); it != fs::recursive_directory_iterator(); ++it) {
        if (fs::is_regular_file(*it)) {
            std::string filename = it->path().filename().string();
            // Check if file ends with .txt and contains partial name
            if (filename.find(partialName) != std::string::npos && it->path().extension() == ".txt") {
                // std::cout << "Found: " << it->path() << std::endl;
                return it->path().string();
            }
        }
    }
    return "";
}



} // namespace internal

int main(int argc, char **argv) {

//  std::string data_root = "/mnt/IVALAB/rosbags/tsrb/GW_CL_SEQS/";
//  std::vector<std::string> seqnames{
      // "20241012", "20250330", "20250331", "20250530", "20250619", "20250831_1", "20250831_2", "20250831_3",
//      "20250912_1",
//    "20250912_2",
//    "20250912_3",
// ;


  internal::Config config = internal::ParseArgs(argc, argv);

  for (const auto &seq_name : config.sequences) {
    std::cout << "Processing " << seq_name << "..." << "\n";
    // std::string filepath = config.data_dir + "/" + seq_name + "/slam_toolbox/slam_toolbox_KeyFrameTrajectory.txt";
    std::string seq_dir = config.data_dir + "/" + seq_name;
    std::string partial_filename = "KeyFrameTrajectory.txt";
    std::string filepath = internal::findFiles(seq_dir, partial_filename);
    if (filepath.empty()) {
        std::cout << seq_name << " does NOT contain any pose file for processing, SKIP! " << std::endl;
        continue;
    }
    // Load samples.
    std::vector<Eigen::VectorXd> samples;
    DatasetReader::load_simulated_trajectory(filepath, samples);
    if (samples.size() < 10u) {
      std::cout << "Insufficient samples: " << samples.size() << "\n";
      continue;
    }
    double start_t = samples.front()(0);
    double end_t = samples.back()(0);
    double dt = 1.0 / 100.0;

    // Fit splines.
    BsplineSE3 spline;
    spline.feed_trajectory(samples);
    Eigen::Matrix3d R, R_cam, R_bc;
    Eigen::Vector3d p, p_cam, p_bc;
    // SLAMbot body to cam extrinsic.
    R_bc << 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0;
    p_bc << 0.125, 0.040, 0.50;
    std::vector<internal::StampedPose> body_poses;
    std::vector<internal::StampedPose> cam_poses;
    while (start_t <= end_t) {
      bool success = spline.get_pose(start_t, R, p);
      if (success) {
        body_poses.emplace_back(start_t, R, p);
        R_cam = R * R_bc;
        p_cam = R * p_bc + p;
        cam_poses.emplace_back(start_t, R_cam, p_cam);
      }
      start_t += dt;
    }

    // Save poses.
    std::string gt_dir = config.data_dir + "/gt_poses/";
    if (!fs::exists(fs::path(gt_dir))) {
        fs::create_directories(fs::path(gt_dir));
    }
    {
      std::string outfile = gt_dir + "/" + seq_name + "_body.txt";
      internal::save(outfile, body_poses);

      outfile = gt_dir + "/" + seq_name + "_cam0.txt";
      internal::save(outfile, cam_poses);
    }
    std::cout << "Done!" << std::endl;
  }
  return 0;
}
