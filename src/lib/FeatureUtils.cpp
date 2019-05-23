//
// Created by mnowicki on 27.11.18.
//

#include "loam_velodyne/FeatureUtils.h"
#include <iostream>

Eigen::Vector3d FeatureUtils::computeVectorPointToEdge(Eigen::Vector3d &point, Eigen::Vector4d &edgePoint1, Eigen::Vector4d &edgePoint2) {
    // Vectors vec02 (feature to point on edge), vec21 (two points on the edge)
    Eigen::Vector3d vec02 = edgePoint2.head<3>() - point;
    Eigen::Vector3d vec21 = edgePoint1.head<3>() - edgePoint2.head<3>();

    // Normalizing edge vector
    Eigen::Vector3d vec21n = vec21.normalized();

    // Projecting vec20 on the edge
    return vec02 + (-vec02).dot(vec21n) * vec21n;
}

bool FeatureUtils::computeEdgeConstraint(Eigen::Vector3d &point, Eigen::Vector4d &edgePoint1, Eigen::Vector4d &edgePoint2,
                                  Eigen::Vector4d &constraint, const double &sMult) {

    // Computes the vector from selected point to the edge
    Eigen::Vector3d vectorPointToEdge = FeatureUtils::computeVectorPointToEdge(point, edgePoint1, edgePoint2);

    // Distance from feature to line
    double distance = vectorPointToEdge.norm();

    // Linear weight from 1 (for 0 m) to 0.1 (for 1 m), equal to 0 for distance > 1 m
    float s = 1 - sMult * fabs(distance);

    // if the selected point has a valid line correspondence (closer than 1 meter) then add to laserCloudOri
    if (s > 0.1) {
        // Normalized direction
        constraint.head<3>() = -s * vectorPointToEdge / distance;

        // Current error
        constraint(3) = s * distance;

        return true;
    }
    return false;
}

Eigen::Vector4d FeatureUtils::computePlaneEq(const Eigen::MatrixXd &points) {
    Eigen::Vector4d planeEq, mean;
    Eigen::Matrix3d covariance;
    double curvature;

    FeatureUtils::computePlaneEq(points, planeEq, mean, covariance, curvature);

    return planeEq;
}

bool FeatureUtils::computePlaneEq(const Eigen::MatrixXd &points, Eigen::Vector4d &planeEq, Eigen::Vector4d &mean, Eigen::Matrix3d &covariance, double &curvature) {
    planeEq = Eigen::Vector4d::Zero();

    // Computing mean
    mean = Eigen::Vector4d::Zero();
    for (int j = 0; j < points.cols(); j++)
        mean += points.col(j);
    mean /= points.cols();

    // Points without mean
    Eigen::MatrixXd demeanPts = points;
    for (int j = 0; j < demeanPts.cols(); ++j)
        demeanPts.col(j) -= mean;

    // Covariance
    covariance = demeanPts.topRows<3>() * demeanPts.topRows<3>().transpose();

    // EigenValue
    Eigen::SelfAdjointEigenSolver <Eigen::Matrix3d> evd(covariance);

    // Curvature check
    curvature = evd.eigenvalues()(0) / evd.eigenvalues().sum();

   
  //  if (curvature > 0.00015) // https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=6948422&tag=1
  //      return false;
    // Plane equation
    planeEq.head<3>() = evd.eigenvectors().col(0);
    planeEq(3) = -planeEq.head<3>().dot(mean.head<3>());

    // Consistency check -  // Moved to equation calculation
    // for (int i = 0; i < points.cols(); i++) {
//        double distance = FeatureUtils::computeDistancePointToPlane(planeEq, points.col(i).head<3>());
    //     if (distance > 0.15)
    //         return false;
    // }

    return true;
}

bool FeatureUtils::computeSurfConstraint(Eigen::Vector3d &point,
                                  Eigen::Vector4d &planeEq,
                                  Eigen::Vector4d &constraint,
                                  const double &sMult) {
    // Distance of selected point to plane
    double distance = FeatureUtils::computeDistancePointToPlane(planeEq, point);

    // Linear weight from 1 (for 0 m) to 0.1 (for 1 m), equal to 0 for distance > 1 m
    double pointNorm = point.norm();
    float s = 1 - sMult * fabs(distance) / sqrt(pointNorm); // TODO: Why sqrt(pointNorm?)? distance / sqrt(distanceOfMeasurement)

    // if the selected point has a valid plane correspondence then use it in optimization
    if (s > 0.1) {

        // Normalized direction
        constraint.head<3>() = s * planeEq.head<3>();

        // Current error
        constraint(3) = s * distance;

        return true;
    }
    return false;
}

double FeatureUtils::computeDistanceBetweenPoints(Eigen::Vector4d a, Eigen::Vector4d b){

    double diffX = a(0) - b(0);
    double diffY = a(1) - b(1);
    double diffZ = a(2) - b(2);

  return sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ);
}

double FeatureUtils::computeAngleBetweenPlanes(Eigen::Vector4d a, Eigen::Vector4d b){

    double l = a.head<3>().dot(b.head<3>());
    double m = a.head<3>().norm() * b.head<3>().norm();
    double w = fabs(l) / m;
    double angle = acos(w)*180.0/M_PI;
    return angle;   
}

double FeatureUtils::computeDistanceBetweenPlanes(Eigen::Vector4d a, Eigen::Vector4d b){

    double l = fabs( a(3) - b(3));
    double m = a.head<3>().norm();
   //std::cout << "a:  " << a.head<3>().norm() << "  b:  " << b.head<3>().norm() << std::endl;
    double w = l / m;
    return w;

}

double FeatureUtils::computeDistancePointToPlane(const Eigen::Vector4d &planeEq, const Eigen::Vector3d &p) {
    return planeEq.head<3>().dot(p) + planeEq(3);
}

double FeatureUtils::computeDistancePointToPlane(const Eigen::Vector4d &planeEq, const Eigen::Vector4d &p) {
    return planeEq.head<3>().dot(p.head<3>()) + planeEq(3);
}