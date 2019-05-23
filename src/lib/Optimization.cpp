//
// Created by mnowicki on 08.01.19.
//

#include "loam_velodyne/Optimization.h"

void Optimization::poseOptimizationIteration(const Eigen::Matrix4d &curPose,
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
                                      Eigen::Matrix<float, 6, 6> &eigenvectors) {

    int pointSelNum = pointsConstraints.size();


    // Rotating by scanner mounting angle
    Eigen::Matrix3d rotX = Eigen::Matrix3d::Zero();
    double tenDegInRad = 0.0 * 3.1415265/180.0;
    rotX(0,0) = 1.0;
    rotX(1,1) = cos(tenDegInRad);
    rotX(1,2) = -sin(tenDegInRad);
    rotX(2,1) = sin(tenDegInRad);
    rotX(2,2) = cos(tenDegInRad);

    // Jacobian and right side
    Eigen::Matrix<float, Eigen::Dynamic, 6> matJ(pointSelNum, 6);
    Eigen::Matrix<float, 6, Eigen::Dynamic> matJt(6, pointSelNum);
    Eigen::VectorXf matB(pointSelNum);

    // For every point fill the Jacobians
    for (int i = 0; i < pointSelNum; i++) {
        const pcl::PointXYZI &pointOri = pointsConstraints.points[i];
        pcl::PointXYZI coeff = coeffsConstraints.points[i];

        // Normal of constraints
        const Eigen::Vector3d &n = coeff.getVector3fMap().cast<double>();

        // Current rotation
        const Eigen::Matrix3d &R = curPose.block<3, 3>(0, 0);

        // Computing jacobians
        Eigen::Matrix3d skewP = LieAlgebra::skew(pointOri.getVector3fMap().cast<double>());
        Eigen::Vector3d jacobianRotation = n.transpose() * rotX * R * (-skewP);
        Eigen::Vector3d jacobianTranslation = n.transpose() * rotX * R;
//        Eigen::Vector3d jacobianRotation = n.transpose() * R * (-skewP);
//        Eigen::Vector3d jacobianTranslation = n.transpose() * R;

        // Filling A matrix
        matJ(i, 0) = jacobianRotation(0); // 0
        matJ(i, 1) = jacobianRotation(1);
        matJ(i, 2) = jacobianRotation(2);  // 0
        matJ(i, 3) = jacobianTranslation(0);
        matJ(i, 4) = jacobianTranslation(1); // 0
        matJ(i, 5) = jacobianTranslation(2);
        matB(i, 0) = -coeffWeight * coeff.intensity;
    }

    // Gauss Newton step - J^T * J * X = J^T * B
    matJt = matJ.transpose();
    Eigen::Matrix<float, 6, 6> matJtJ = matJt * matJ;
    Eigen::Matrix<float, 6, 1> matJtB = matJt * matB;

    Eigen::Matrix<float, 6, 6> forcedSymmetry = 0.5f * (matJtJ + matJtJ.transpose().eval()).eval();
    matJtJ = forcedSymmetry;
//    std::cout << "Checking symmetry: " << std::endl << matJtJ << std::endl << " --- " << std::endl;

    // Finding the increment
    Eigen::Matrix<float, 6, 1> matX = matJtJ.colPivHouseholderQr().solve(matJtB);

    // For first iteration we verify eigenvalues
    double smallestEigenvalue = 0;
    if (iterCount == 0) {

        // Finding eigenvalues and eigenvectors of JtJ
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, 6, 6> > esolver(matJtJ);
        Eigen::Matrix<float, 1, 6> matE = esolver.eigenvalues().real();
        Eigen::Matrix<float, 6, 6> matVcols = esolver.eigenvectors().real();

        // Saving for future references
        eigenvalues = matE;
        eigenvectors = matVcols;
//
//        // TODO All eigenvalues
//        std::cout << "Eigenvalues: " << std::endl << matE << std::endl;
//        std::cout << "Cond number: " << std::endl << matE(0,5)/matE(0,0) << std::endl;
//
//        // TODO: TESTS
//        smallestEigenvalue = matE(0, 0);

        // Reversing the order and saving as rows
        Eigen::Matrix<float, 6, 6> matV = matVcols.rowwise().reverse();
        matV.transposeInPlace();

//        std::cout << "Eigenvector of the smallest eigenvalue: " << std::endl << matV.row(5) << std::endl << " ---- " << std::endl;

        Eigen::Matrix<float, 6, 6> matV2 = matV;

        isDegenerate = false;
        float eignThre[6] = {10, 10, 10, 10, 10, 10}; // TODO: It is different between versions
//        float eignThre[6] = {100, 100, 100, 100, 100, 100};
        for (int i = 0; i < 6; i++) {
            if (matE(0, i) < eignThre[i]) {
                for (int j = 0; j < 6; j++) {
                    // Eigenvectors as rows
                    matV2(5 - i, j) = 0;
                }
                isDegenerate = true;
            } else {
                break;
            }
        }
        matP = matV.inverse() * matV2;
    }

    // Reducing the influence in directions that are poorly constrained
    if (isDegenerate) {
        Eigen::Matrix<float, 6, 1> matX2(matX);
        matX = matP * matX2;
    }

    // Computing the increment
    Eigen::Vector6d eps(matX.col(0));

    // Zeroing components
//    eps(0) = 0; // Rotation X
//    eps(2) = 0; // Rotation Z
//    eps(4) = 0; // Translation Y

    // Adding the increment to estimated pose
    dT = LieAlgebra::exp(eps);

    // Rotating back
    Eigen::Matrix4d rotationBack = Eigen::Matrix4d::Identity();
    rotationBack.block<3,3>(0,0) = rotX.inverse();
    dT = rotationBack * dT;

    // Stop conditions
    deltaR = loam::rad2deg(eps.head<3>().norm());
    deltaT = eps.tail<3>().norm() * 100.0;
}