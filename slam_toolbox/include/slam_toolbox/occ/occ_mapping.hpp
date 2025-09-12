/**
 * @file occ_mapping.hpp
 * @author Yanwei Du (yanwei.du@gatech.edu)
 * @brief None
 * @version 0.1
 * @date 04-13-2026
 * @copyright Copyright (c) 2026
 */

#ifndef SLAM_TOOLBOX_OCC_MAPPING_H_
#define SLAM_TOOLBOX_OCC_MAPPING_H_

#include <semantic_msgs/KeyFramePoseArray.h>

#include "slam_toolbox/slam_toolbox_common.hpp"
#include "slam_toolbox/spline/BSplineSE3.h"
#include "slam_toolbox/spline/dataset_reader.h"

namespace slam_toolbox {

class OccMapping : public SlamToolbox {
public:
    using Pose2 = karto::Pose2;
    struct StampedPose3 {
        double timestamp, tx, ty, tz, qx, qy, qz, qw;

        StampedPose3() = default;

        StampedPose3(double _timestamp, const Eigen::Matrix3d& R,
                     const Eigen::Vector3d& t) {
            timestamp = _timestamp;
            tx        = t.x();
            ty        = t.y();
            tz        = t.z();
            Eigen::Quaterniond quat(R);
            qx = quat.x();
            qy = quat.y();
            qz = quat.z();
            qw = quat.w();
        }

        Eigen::VectorXd toEigenVector() const {
            Eigen::Matrix<double, 8, 1> data;
            data << timestamp, tx, ty, tz, qx, qy, qz, qw;
            return data;
        }

        friend std::ostream& operator<<(std::ostream&       os,
                                        const StampedPose3& s) {
            os << std::fixed;
            os << std::setprecision(10) << s.timestamp << " "
               << std::setprecision(6) << s.tx << " " << s.ty << " " << s.tz
               << " " << s.qx << " " << s.qy << " " << s.qz << " " << s.qw;
            return os;
        }

        static std::string header() {
            return "# timestamp tx ty tz qx qy qz qw";
        }
    };

    OccMapping(ros::NodeHandle& nh);
    virtual ~OccMapping() {};
    void run();

protected:
    virtual void laserCallback(
        const sensor_msgs::LaserScan::ConstPtr& scan) override final;

    void keyframePoseArrayCallback(
        const semantic_msgs::KeyFramePoseArray::ConstPtr& kfs);

    void correctPosesWithVisualPoseGraph(const std::vector<StampedPose3>& kfs);

    bool clearQueueCallback(slam_toolbox_msgs::ClearQueue::Request&  req,
                            slam_toolbox_msgs::ClearQueue::Response& resp);
    virtual bool deserializePoseGraphCallback(
        slam_toolbox_msgs::DeserializePoseGraph::Request&  req,
        slam_toolbox_msgs::DeserializePoseGraph::Response& resp) override final;

    std::vector<StampedPose3> loadKeyFramePosesFromFile(
        const std::string& filepath);

    std::queue<PosedScan>     q_;
    ros::ServiceServer        ssClear_;
    boost::mutex              q_mutex_;
    ros::Subscriber           kf_sub_;
    std::atomic<bool>         global_correction_{false};
    bool                      use_loaded_pose_ = false;
    std::vector<StampedPose3> kf_poses_;
    ov_core::BsplineSE3       spline_;

    std::vector<StampedPose3> run_spline_fitting(
        const std::vector<StampedPose3>& in) {
        std::vector<Eigen::VectorXd> traj;
        for (const auto& p : in) {
            traj.push_back(p.toEigenVector());
        }
        ov_core::BsplineSE3 spline;
        spline.feed_trajectory(traj);

        double start_t = in.front().timestamp;
        double end_t   = in.back().timestamp;
        double dt      = 0.01;

        Eigen::Matrix3d Rot;
        Eigen::Vector3d trans;

        std::vector<StampedPose3> out;
        while (start_t <= end_t) {
            bool success = spline.get_pose(start_t, Rot, trans);
            if (success) {
                out.emplace_back(start_t, Rot, trans);
            }
            start_t += dt;
        }
        return out;
    }

    inline double wrapToPi(double a) {
        a = std::fmod(a + M_PI, 2.0 * M_PI);
        if (a < 0) a += 2.0 * M_PI;
        return a - M_PI;
    }

    inline void normalizeQuat(double& qx, double& qy, double& qz, double& qw) {
        const double n = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
        if (n > 0.0) {
            qx /= n;
            qy /= n;
            qz /= n;
            qw /= n;
        }
    }

    inline void slerpQuat(double qx1, double qy1, double qz1, double qw1,
                          double qx2, double qy2, double qz2, double qw2,
                          double t, double& qxo, double& qyo, double& qzo,
                          double& qwo) {
        normalizeQuat(qx1, qy1, qz1, qw1);
        normalizeQuat(qx2, qy2, qz2, qw2);

        double dot = qx1 * qx2 + qy1 * qy2 + qz1 * qz2 + qw1 * qw2;
        if (dot < 0.0) {
            dot = -dot;
            qx2 = -qx2;
            qy2 = -qy2;
            qz2 = -qz2;
            qw2 = -qw2;
        }

        const double dot_threshold = 0.9995;
        if (dot > dot_threshold) {
            qxo = qx1 + t * (qx2 - qx1);
            qyo = qy1 + t * (qy2 - qy1);
            qzo = qz1 + t * (qz2 - qz1);
            qwo = qw1 + t * (qw2 - qw1);
            normalizeQuat(qxo, qyo, qzo, qwo);
            return;
        }

        const double theta0     = std::acos(dot);
        const double theta      = theta0 * t;
        const double sin_theta0 = std::sin(theta0);
        const double sin_theta  = std::sin(theta);
        const double s0 = std::cos(theta) - dot * sin_theta / sin_theta0;
        const double s1 = sin_theta / sin_theta0;
        qxo             = s0 * qx1 + s1 * qx2;
        qyo             = s0 * qy1 + s1 * qy2;
        qzo             = s0 * qz1 + s1 * qz2;
        qwo             = s0 * qw1 + s1 * qw2;
        normalizeQuat(qxo, qyo, qzo, qwo);
    }

    inline double yawFromQuat(double qx, double qy, double qz, double qw) {
        // yaw (around Z) = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        const double s1    = 2.0 * (qw * qz + qx * qy);
        const double c1    = 1.0 - 2.0 * (qy * qy + qz * qz);
        double       angle = std::atan2(s1, c1);
        // if (angle < 0.0) {
        // angle += 2 * M_PI;
        // }
        return angle;
    }

    inline bool interpolatePose(const std::vector<StampedPose3>& samples,
                                double tq, Pose2& pose,
                                bool   clamp_to_ends       = false,
                                bool   allow_interpolation = false,
                                double max_time_diff       = 0.1) {
        double out_x, out_y, out_theta;
        if (samples.empty()) return false;

        if (!allow_interpolation) {
            if (samples.size() == 1) {
                const auto&  s  = samples.front();
                const double dt = std::fabs(tq - s.timestamp);
                if (dt <= max_time_diff) {
                    out_x     = s.tx;
                    out_y     = s.ty;
                    out_theta = yawFromQuat(s.qx, s.qy, s.qz, s.qw);
                    pose      = Pose2(out_x, out_y, out_theta);
                    return true;
                }
                return false;
            }

            auto it_hi =
                std::lower_bound(samples.begin(), samples.end(), tq,
                                 [](const StampedPose3& s, double tval) {
                                     return s.timestamp < tval;
                                 });
            const StampedPose3* nearest = nullptr;
            if (it_hi == samples.begin()) {
                nearest = &(*it_hi);
            } else if (it_hi == samples.end()) {
                nearest = &samples.back();
            } else {
                const auto& s1 = *(it_hi - 1);
                const auto& s2 = *it_hi;
                nearest = (tq - s1.timestamp <= s2.timestamp - tq) ? &s1 : &s2;
            }

            if (nearest) {
                const double dt = std::fabs(tq - nearest->timestamp);
                if (dt <= max_time_diff) {
                    out_x     = nearest->tx;
                    out_y     = nearest->ty;
                    out_theta = yawFromQuat(nearest->qx, nearest->qy,
                                            nearest->qz, nearest->qw);
                    pose      = Pose2(out_x, out_y, out_theta);
                    return true;
                }
            }
            return false;
        }

        // Ensure time is within range
        if (tq < samples.front().timestamp) {
            if (!clamp_to_ends) return false;
            out_x     = samples.front().tx;
            out_y     = samples.front().ty;
            out_theta = yawFromQuat(samples.front().qx, samples.front().qy,
                                    samples.front().qz, samples.front().qw);
            pose      = Pose2(out_x, out_y, out_theta);
            return true;
        }
        if (tq > samples.back().timestamp) {
            if (!clamp_to_ends) return false;
            out_x     = samples.back().tx;
            out_y     = samples.back().ty;
            out_theta = yawFromQuat(samples.back().qx, samples.back().qy,
                                    samples.back().qz, samples.back().qw);
            pose      = Pose2(out_x, out_y, out_theta);
            return true;
        }

        // Find the first sample with t >= tq
        auto it_hi = std::lower_bound(samples.begin(), samples.end(), tq,
                                      [](const StampedPose3& s, double tval) {
                                          return s.timestamp < tval;
                                      });
        if (it_hi == samples.begin()) {
            // tq == first time
            out_x     = it_hi->tx;
            out_y     = it_hi->ty;
            out_theta = yawFromQuat(it_hi->qx, it_hi->qy, it_hi->qz, it_hi->qw);
            pose      = Pose2(out_x, out_y, out_theta);
            return true;
        }
        if (it_hi == samples.end()) {
            // tq == last time (handled above, but keep safe)
            const auto& s = samples.back();
            out_x         = s.tx;
            out_y         = s.ty;
            out_theta     = yawFromQuat(s.qx, s.qy, s.qz, s.qw);
            pose          = Pose2(out_x, out_y, out_theta);
            return true;
        }

        const auto& s1 = *(it_hi - 1);  // t1 <= tq
        const auto& s2 = *it_hi;        // t2 >= tq

        const double t1 = s1.timestamp, t2 = s2.timestamp;
        if (t2 == t1) {
            // Degenerate (duplicate timestamps) → use s1
            out_x     = s1.tx;
            out_y     = s1.ty;
            out_theta = yawFromQuat(s1.qx, s1.qy, s1.qz, s1.qw);
            pose      = Pose2(out_x, out_y, out_theta);
            return true;
        }

        const double alpha = (tq - t1) / (t2 - t1);  // in [0,1]

        // Linear interp in XY
        out_x = (1.0 - alpha) * s1.tx + alpha * s2.tx;
        out_y = (1.0 - alpha) * s1.ty + alpha * s2.ty;

        // Quaternion interpolation for yaw extraction
        double qx, qy, qz, qw;
        slerpQuat(s1.qx, s1.qy, s1.qz, s1.qw, s2.qx, s2.qy, s2.qz, s2.qw, alpha,
                  qx, qy, qz, qw);
        out_theta = yawFromQuat(qx, qy, qz, qw);
        pose      = Pose2(out_x, out_y, out_theta);
        return true;
    }
};

}  // namespace slam_toolbox

#endif