//
// Created by mnowicki on 05.12.18.
//

#ifndef LOAM_VELODYNE_POSEUTILS_H
#define LOAM_VELODYNE_POSEUTILS_H

#include <Eigen/Dense>
#include <vector>

class PoseUtils {

public:
    static Eigen::Matrix4d inverse(const Eigen::Matrix4d &pose) {
        Eigen::Matrix4d invPose = Eigen::Matrix4d::Identity();
        invPose.block<3, 3>(0, 0) = pose.block<3, 3>(0, 0).transpose();
        invPose.block<3, 1>(0, 3) = -invPose.block<3, 3>(0, 0) * pose.block<3, 1>(0, 3);
        return invPose;
    }

};


#endif //LOAM_VELODYNE_POSEUTILS_H
