//
// Created by mnowicki on 27.11.18.
//

#ifndef LOAM_VELODYNE_FEATURE_UTILS_H
#define LOAM_VELODYNE_FEATURE_UTILS_H

#include <Eigen/Dense>
#include <vector>

typedef Eigen::Matrix<float, 6, 1> Vector6f;
typedef Eigen::Matrix<float, 6, 1> Vector6d;

typedef std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d> > vectorOfEigenVector4d;
typedef std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > vectorOfEigenVector3d;

class FeatureUtils {

public:

    /*
     * Computes the distance between point and the edge (represented by two points)
     */
    static Eigen::Vector3d computeVectorPointToEdge(Eigen::Vector3d &point, Eigen::Vector4d &edgePoint1, Eigen::Vector4d &edgePoint2);

    /*
     * Computes the constraint based on the point and two point belonging to the edge
     */
    static bool computeEdgeConstraint(Eigen::Vector3d &point, Eigen::Vector4d &edgePoint1, Eigen::Vector4d &edgePoint2,
                                           Eigen::Vector4d &constraint, const double &sMult = 0.9);

    /*
     * Takes the matrix of 3D points (4 by numberOfPoints) and computes the plane equation
     */
    static Eigen::Vector4d computePlaneEq(const Eigen::MatrixXd &points);

    /*
     * Takes the matrix of 3D points (4 by numberOfPoints) and computes the plan equation, points' mean, points' covariance,
     * and plane's curvature. Returns true if ok, false if curvature check or points consistency check is not ok
     */
    static bool computePlaneEq(const Eigen::MatrixXd &points, Eigen::Vector4d &planeEq, Eigen::Vector4d &mean,
            Eigen::Matrix3d &covariance, double &curvature);

    /*
     * Computes the constraint based on the point and the planeEq
     */
    static bool computeSurfConstraint(Eigen::Vector3d &point, Eigen::Vector4d &planeEq,
                                      Eigen::Vector4d &constraint, const double &sMult = 0.9);

    /*
     * Computes the distance between two points
     */
    static double computeDistanceBetweenPoints(Eigen::Vector4d point1, Eigen::Vector4d point2); 

    /*
     * Computes the angle between two planes
     */

    static double computeAngleBetweenPlanes(Eigen::Vector4d a, Eigen::Vector4d b); 

     /*
     * Computes the distance between two planes
     */ 
     static double computeDistanceBetweenPlanes(Eigen::Vector4d a, Eigen::Vector4d b);

     /*
      * Computes the distance between point and plane
      */
     static double computeDistancePointToPlane(const Eigen::Vector4d &planeEq, const Eigen::Vector3d &p);

     static double computeDistancePointToPlane(const Eigen::Vector4d &planeEq, const Eigen::Vector4d &p);
};

#endif //LOAM_VELODYNE_FEATURE_UTILS_H
