////
//// Created by mnowicki on 13.12.18.
////
//
//#ifndef LOAM_VELODYNE_TMP_H
//#define LOAM_VELODYNE_TMP_H
//
//#include <Eigen/Eigen>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <opencv2/opencv.hpp>
//#include "LieAlgebra.h"
//
//
//class TMP {
//
//public:
//    static inline float rad2deg(float radians) {
//        return (float) (radians * 180.0 / M_PI);
//    }
//
//    static void optimizationPass(const Eigen::Matrix4d &pose,
//                                 pcl::PointCloud<pcl::PointXYZI> &pointsForConstraints,
//                                 pcl::PointCloud<pcl::PointXYZI> &coeffsForConstraints,
//                                 bool checkDegeneracy,
//                                 bool &isDegenerate,
//                                 cv::Mat &matCutDirections,
//                                 Eigen::Matrix4d &dT,
//                                 double &deltaR,
//                                 double &deltaT,
//                                 double costWeight) {
//
//
//        int laserCloudSelNum = pointsForConstraints.points.size();
//
//        cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
//        cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
//        cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
//        // error function
//        cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
//        cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
//        // solution increment
//        cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
//
//        for (int i = 0; i < laserCloudSelNum; i++) {
//            const pcl::PointXYZI &pointInLocal = pointsForConstraints.points[i];
//            const pcl::PointXYZI &coeff = coeffsForConstraints.points[i];
//
//            const Eigen::Vector3d &n = coeff.getVector3fMap().cast<double>();
//
//            const Eigen::Matrix3d &R = pose.block<3, 3>(0, 0);
//
//            Eigen::Matrix3d skewP = LieAlgebra::skew(pointInLocal.getVector3fMap().cast<double>());
//            Eigen::Vector3d ar = n.transpose() * R * (-skewP);
//            Eigen::Vector3d at = n.transpose() * R;
//
//            matA.at<float>(i, 0) = ar(0);
//            matA.at<float>(i, 1) = ar(1);
//            matA.at<float>(i, 2) = ar(2);
//            matA.at<float>(i, 3) = at(0);
//            matA.at<float>(i, 4) = at(1);
//            matA.at<float>(i, 5) = at(2);
//            matB.at<float>(i, 0) = -costWeight * coeff.intensity;
//        }
//
//        //            std::cout << "matB.t() * matB = " << matB.t() * matB << std::endl;
//
//        // Gauss Newton step
//        cv::transpose(matA, matAt);
//        matAtA = matAt * matA;
//        matAtB = matAt * matB;
//        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);
//
//        //            std::cout << "matAtA = " << std::endl << matAtA << std::endl;
//
//        matCutDirections = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));
//        // if first iteration
//        if (checkDegeneracy) {
//            cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
//            cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
//            cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));
//
//            cv::eigen(matAtA, matE, matV);
//            matV.copyTo(matV2);
//
//            //                std::cout << "matE = " << std::endl << matE << std::endl;
//            // if some eigenvector is small then a problem
//            // is not well constrained in its direction
//            isDegenerate = false; // TODO: CRUCIAL CHANGE!
//            float eignThre[6] = {100, 100, 100, 100, 100, 100};
//            for (int i = 5; i >= 0; i--) {
//                // if eigenvalue lower than a threshold
//                if (matE.at<float>(0, i) < eignThre[i]) {
//                    // zeroing associated eigenvector
//                    for (int j = 0; j < 6; j++) {
//                        matV2.at<float>(i, j) = 0;
//                    }
//                    isDegenerate = true;
//                } else {
//                    break;
//                }
//            }
//            std::cout << "Inno: " << matE.col(0)  << std::endl;
//            std::cout << "Inno matV: " << std::endl << matV << std::endl;
//            std::cout << "Inno matV2: " << std::endl << matV2 << std::endl;
//
//            //                std::cout << "matV = " << std::endl << matV << std::endl;
//            //                std::cout << "matV2 = " << std::endl << matV2 << std::endl;
//            //                // matrix that cuts out the update in poorly constrained directions
//            matCutDirections = matV.inv() * matV2;
//            std::cout << "Inno matP: " << std::endl << matCutDirections << std::endl;
//        }
//
//        // cutting out the update in poorly constrained directions
//        if (isDegenerate) {
//            std::cout << "INNO matP = " << std::endl << matCutDirections << std::endl;
//
//            cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
//            matX.copyTo(matX2);
//            matX = matCutDirections * matX2;
//
//            std::cout << "INNO matX = " << std::endl << matX << std::endl << "----" << std::endl;
//        }
//
//        // updating transformation
//
//
//        Eigen::Vector6d eps;
//        eps << matX.at<float>(0, 0),
//                matX.at<float>(1, 0),
//                matX.at<float>(2, 0),
//                matX.at<float>(3, 0),
//                matX.at<float>(4, 0),
//                matX.at<float>(5, 0);
//
//        //            std::cout << "eps = " << eps.transpose() << std::endl;
//
//        deltaR = rad2deg(eps.head<3>().norm());
//        deltaT = eps.tail<3>().norm() * 100.0;
//
//        dT = LieAlgebra::exp(eps);
//    }
//
//
//};
//
//#endif //LOAM_VELODYNE_TMP_H
