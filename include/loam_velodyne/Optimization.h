//
// Created by mnowicki on 08.01.19.
//

#ifndef LOAM_VELODYNE_OPTIMIZATION_H
#define LOAM_VELODYNE_OPTIMIZATION_H

#include "LieAlgebra.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "math_utils.h"

class Optimization {
public:

    // TODO: It will return the smallst eigenvalue for tests
    static void poseOptimizationIteration(const Eigen::Matrix4d &curPose,
                                   pcl::PointCloud <pcl::PointXYZI> &pointsConstraints,
                                   pcl::PointCloud <pcl::PointXYZI> &coeffsConstraints,
                                   int iterCount,
                                   bool &isDegenerate,
                                   Eigen::Matrix<float, 6, 6> &matP,
                                   Eigen::Matrix4d &dT,
                                   double &deltaR,
                                   double &deltaT,
                                   const double coeffWeight,
                                   Eigen::Matrix<float, 1, 6> &eigenvalues,
                                   Eigen::Matrix<float, 6, 6> &eigenvectors);
};
#endif //LOAM_VELODYNE_OPTIMIZATION_H
