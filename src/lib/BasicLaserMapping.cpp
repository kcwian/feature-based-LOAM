// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.


#include "loam_velodyne/BasicLaserMapping.h"
#include "loam_velodyne/nanoflann_pcl.h"
#include "../../include/loam_velodyne/math_utils.h"

#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include <chrono>       // high_resolution_clock - Wall clock 
#include <ctime>        // clock() - CPU Time

namespace loam {

    using std::sqrt;
    using std::fabs;
    using std::asin;
    using std::atan2;
    using std::pow;


    BasicLaserMapping::BasicLaserMapping(const float &scanPeriod, const size_t &maxIterations) :
            _scanPeriod(scanPeriod),
            _mapFrameNum(5),
            _mapFrameCount(0),
            currentFrame(0),
            _maxIterations(maxIterations),
            _deltaTAbort(0.05),
            _deltaRAbort(0.05),
            _laserCloudCenWidth(10),
            _laserCloudCenHeight(5),
            _laserCloudCenDepth(10),
            _laserCloudWidth(21),
            _laserCloudHeight(11),
            _laserCloudDepth(21),
            _save_trajectory_to_file(true),

            _laserCloudNum(_laserCloudWidth * _laserCloudHeight * _laserCloudDepth),
            _laserCloudEdgeLast(new pcl::PointCloud<pcl::PointXYZI>()),
            _laserCloudSurfLast(new pcl::PointCloud<pcl::PointXYZI>()),
            _laserCloudFullRes(new pcl::PointCloud<pcl::PointXYZI>()),
            _laserCloudEdgeLastDS(new pcl::PointCloud<pcl::PointXYZI>()),
            _laserCloudSurfLastDS(new pcl::PointCloud<pcl::PointXYZI>()),
            _cloudForVisualization(new pcl::PointCloud<pcl::PointXYZI>()) {

        // initialize frame counter
        _mapFrameCount = _mapFrameNum - 1;

        // setup cloud vectors
        _laserCloudEdgeArray.resize(_laserCloudNum);
        _laserCloudSurfArray.resize(_laserCloudNum);

        for (size_t i = 0; i < _laserCloudNum; i++) {
            _laserCloudEdgeArray[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
            _laserCloudSurfArray[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
        }

        // setup down size filters
        _downSizeFilterEdge.setLeafSize(0.2, 0.2, 0.2);
        _downSizeFilterSurf.setLeafSize(0.4, 0.4, 0.4);

        // Initialize poses
        curOdometryPose = Eigen::Matrix4d::Identity();
        curMapWithOdo = Eigen::Matrix4d::Identity();
        lastOdometryPose = Eigen::Matrix4d::Identity();
        lastMapPose = Eigen::Matrix4d::Identity();

        eigenvalueStream.open("mapping_eigenvalues.txt");
    }


    void BasicLaserMapping::updateCurrentPose() {

        // Computing current pose as last mapping pose + increment from odometry
        Eigen::Matrix4d incrementFromOdo = PoseUtils::inverse(lastOdometryPose) * curOdometryPose;
        curMapWithOdo = lastMapPose * incrementFromOdo;
    }


    void BasicLaserMapping::transformUpdate() {

        // TODO: Case with IMU
//        if (0 < _imuHistory.size()) {
//            size_t imuIdx = 0;
//
//            while (imuIdx < _imuHistory.size() - 1 &&
//                   toSec(_laserOdometryTime - _imuHistory[imuIdx].stamp) + _scanPeriod > 0) {
//                imuIdx++;
//            }
//
//            IMUState2 imuCur;
//
//            if (imuIdx == 0 || toSec(_laserOdometryTime - _imuHistory[imuIdx].stamp) + _scanPeriod > 0) {
//                // scan time newer then newest or older than oldest IMU message
//                imuCur = _imuHistory[imuIdx];
//            } else {
//                float ratio = (toSec(_imuHistory[imuIdx].stamp - _laserOdometryTime) - _scanPeriod)
//                              / toSec(_imuHistory[imuIdx].stamp - _imuHistory[imuIdx - 1].stamp);
//
//                IMUState2::interpolate(_imuHistory[imuIdx], _imuHistory[imuIdx - 1], ratio, imuCur);
//            }
//
//            _transformMapWithOdoIncrement.rot_x = 0.998 * _transformMapWithOdoIncrement.rot_x.rad() + 0.002 * imuCur.pitch.rad();
//            _transformMapWithOdoIncrement.rot_z = 0.998 * _transformMapWithOdoIncrement.rot_z.rad() + 0.002 * imuCur.roll.rad();
//        }

        lastOdometryPose = curOdometryPose;
        lastMapPose = curMapWithOdo;
    }

    void BasicLaserMapping::moveFromLocalToGlobal(const pcl::PointXYZI &pi, pcl::PointXYZI &po, const Eigen::Matrix4d &pose) {
        // to homogeneous
        Eigen::Vector4d point;
        point = pi.getVector4fMap().cast<double>();
        point(3) = 1;

        // transform
        Eigen::Vector4d pointAfter = pose * point;

        // to cartesian
        po.getVector3fMap() = pointAfter.head<3>().cast<float>();
        po.intensity = pi.intensity;

    }

    void BasicLaserMapping::transformFullResToMap() {
        // transform full resolution input cloud to map
        for (auto &pt : *_laserCloudFullRes)
//            moveFromLocalToGlobal(pt, pt);
            moveFromLocalToGlobal(pt, pt, curMapWithOdo);
    }

    bool BasicLaserMapping::createCloudForVisualization(const std::vector<size_t> &laserCloudSurroundInd) {
        // create new map cloud according to the input output ratio
        _mapFrameCount++;
        if (_mapFrameCount < _mapFrameNum)
            return false;

        _mapFrameCount = 0;

        // accumulate map cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurround(new pcl::PointCloud<pcl::PointXYZI>());
        for (const size_t ind : laserCloudSurroundInd) {
            *laserCloudSurround += *_laserCloudEdgeArray[ind];
            *laserCloudSurround += *_laserCloudSurfArray[ind];
        }

        // down size map cloud
        _cloudForVisualization->clear();
        _downSizeFilterEdge.setInputCloud(laserCloudSurround);
        _downSizeFilterEdge.filter(*_cloudForVisualization);
        return true;
    }

    bool BasicLaserMapping::process(Time const &laserOdometryTime) {
        // Timestamp
        _laserOdometryTime = laserOdometryTime;
        currentFrame++;

        clock_t clockProcessBegin = clock();
        clock_t clockStartCalculationsBegin = clock();

        // relate incoming data to map
        updateCurrentPose();

        // Moving points in cubes if necessary
        moveCubes();                                        //  Hangs here

        // Settings the ids of valid (laserCloudValidInd) and surround cloud (laserCloudSurroundInd)
        std::vector <size_t> laserCloudValidInd, laserCloudSurroundInd;
        checkFov(laserCloudValidInd, laserCloudSurroundInd);

        // prepare valid map of edges and surfaces for pose optimization
        pcl::PointCloud<pcl::PointXYZI>::Ptr mapEdges(new pcl::PointCloud<pcl::PointXYZI>()), mapSurfs(new pcl::PointCloud<pcl::PointXYZI>());
        for (auto const &ind : laserCloudValidInd) {
            *mapEdges += *_laserCloudEdgeArray[ind];
            *mapSurfs += *_laserCloudSurfArray[ind];
        }

        // Downsampled edges and surfs from last cloud
        _laserCloudEdgeLastDS->clear();
        _downSizeFilterEdge.setInputCloud(_laserCloudEdgeLast);
        _downSizeFilterEdge.filter(*_laserCloudEdgeLastDS);
        size_t laserCloudEdgeDSNum = _laserCloudEdgeLastDS->size();

        _laserCloudSurfLastDS->clear();
        _downSizeFilterSurf.setInputCloud(_laserCloudSurfLast);
        _downSizeFilterSurf.filter(*_laserCloudSurfLastDS);
        size_t laserCloudSurfDSNum = _laserCloudSurfLastDS->size();


        clock_t clockStartCalculationsEnd = clock();
        timeMeasurement._numOfCallsMappingStartCalculations++;
        timeMeasurement._totalTimeMappingStartCalculations += double(clockStartCalculationsEnd - clockStartCalculationsBegin);

        // run pose optimization
        bool performedOpt = poseOptimization(mapEdges, mapSurfs);            // Optimization time measurement inside function

        clock_t clockEndCalculationsBegin = clock();

        // The points are added only in case of successful optimization
        if (performedOpt || !mapInitialized) {

            if (!mapInitialized && performedOpt)
                mapInitialized = true;

            addEdgesAndSurfsToCubes();


            // store Point to existing features or create new feature
            if (mapManager.isEnabled())
            {
                for (int i = 0; i < laserCloudSurfDSNum; i++)
                {
                    pcl::PointXYZI pointInGlobal;
//                    moveFromLocalToGlobal(_laserCloudSurfLastDS->points[i], pointInGlobal);
                    moveFromLocalToGlobal(_laserCloudSurfLastDS->points[i], pointInGlobal, curMapWithOdo);
                    Eigen::Vector4d pointTmp;
                    pointTmp.head<3>() = pointInGlobal.getVector3fMap().cast<double>();
                    mapManager.addPlanePoint2(pointTmp);
                }
                 mapManager.updatePlaneFeatures();  
                 static int cntD = 0;
                 if (cntD++ > 10) {
                     cntD = 0;
                     std::cout << "Map Surf Points: " << mapSurfs->points.size() << std::endl;
                 }
            }

            // down size all valid (within field of view) feature cube clouds
            for (auto const &ind : laserCloudValidInd) {
                pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>());

                _downSizeFilterEdge.setInputCloud(_laserCloudEdgeArray[ind]);
                _downSizeFilterEdge.filter(*tmp);
                _laserCloudEdgeArray[ind].swap(tmp);

                tmp->clear();
                _downSizeFilterSurf.setInputCloud(_laserCloudSurfArray[ind]);
                _downSizeFilterSurf.filter(*tmp);
                _laserCloudSurfArray[ind].swap(tmp);
            }
        } else {
            std::cout << "Optimization failed so new points were not added!" << std::endl;
        }


        transformFullResToMap();

        _createdCloudForVisualization = createCloudForVisualization(laserCloudValidInd);

        clock_t clockEndCalculationsEnd = clock();
        timeMeasurement._numOfCallsMappingEndCalculations++;
        timeMeasurement._totalTimeMappingEndCalculations += double(clockEndCalculationsEnd - clockEndCalculationsBegin);


        clock_t clockProcessEnd = clock();
        timeMeasurement._numOfCallsMappingProcess++;
        timeMeasurement._totalTimeMappingProcess += double(clockProcessEnd - clockProcessBegin);

        return true;
    }


    void BasicLaserMapping::updateIMU(IMUState2 const &newState) {
        _imuHistory.push(newState);
    }

    void BasicLaserMapping::updateOdometry(const Eigen::Vector3d &t, const Eigen::Quaterniond &q) {
        curOdometryPose.block<3, 3>(0, 0) = q.toRotationMatrix();
        curOdometryPose.block<3, 1>(0, 3) = t;
    }

    bool BasicLaserMapping::poseOptimization(pcl::PointCloud<pcl::PointXYZI>::Ptr mapEdges, pcl::PointCloud<pcl::PointXYZI>::Ptr mapSurfs) {
        // Not enough features to perform mapping
        if (mapEdges->size() <= 10 || mapSurfs->size() <= 100)
            return false;

        clock_t clockSetInputCloudBegin = clock();

        // Creating kd trees based on input data
        nanoflann::KdTreeFLANN <pcl::PointXYZI> kdtreeEdgeFromMap;
        nanoflann::KdTreeFLANN <pcl::PointXYZI> kdtreeSurfFromMap;

        kdtreeEdgeFromMap.setInputCloud(mapEdges);
        kdtreeSurfFromMap.setInputCloud(mapSurfs);

        clock_t clockSetInputCloudEnd = clock();
        timeMeasurement._numOfCallsMappingSetInputCloud++;
        timeMeasurement._totalTimeMappingSetInputCloud += double(clockSetInputCloudEnd - clockSetInputCloudBegin);

        // Information about the directions that should ignored in the optimization
        bool isDegenerate = false;
        Eigen::Matrix<float, 6, 6> matP;

        // TODO
        size_t laserCloudEdgeDSNum = _laserCloudEdgeLastDS->size(), laserCloudSurfDSNum = _laserCloudSurfLastDS->size();

        // Information about the constraints
        pcl::PointCloud <pcl::PointXYZI> pointsConstraints, coeffsConstraints;

        // For each iteration in the optimization
        for (size_t iterCount = 0; iterCount < _maxIterations; iterCount++) {
            pointsConstraints.clear();
            coeffsConstraints.clear();

            clock_t clockConditionsEdgesBegin = clock();

            // For all edges
            for (int i = 0; i < laserCloudEdgeDSNum; i++) {
                pcl::PointXYZI pointInLocal = _laserCloudEdgeLastDS->points[i];
                pcl::PointXYZI pointInGlobal;
                moveFromLocalToGlobal(pointInLocal, pointInGlobal, curMapWithOdo);

                clock_t clockKSearchBegin = clock();

                // Kd tree search for 5 nearest neighbours
                std::vector<int> pointSearchInd(5, 0);
                std::vector<float> pointSearchSqDis(5, 0);
                kdtreeEdgeFromMap.nearestKSearch(pointInGlobal, 5, pointSearchInd, pointSearchSqDis);

                clock_t clockKSearchEnd = clock();
                timeMeasurement._numOfCallsMappingKdSearchEdges++;
                timeMeasurement._totalTimeMappingKdSearchEdges += double(clockKSearchEnd - clockKSearchBegin);


                if (pointSearchSqDis[4] < 1.0) {
                    // Rewriting points to eigen
                    Eigen::Matrix<double, 4, 5> points = Eigen::Matrix<double, 4, 5>::Zero();
                    for (int j = 0; j < 5; j++)
                        points.col(j).head<3>() = mapEdges->points[pointSearchInd[j]].getVector3fMap().cast<double>();

                    // Computing mean of points
                    Eigen::Vector4d computedMean = Eigen::Vector4d::Zero();
                    for (int j = 0; j < points.cols(); j++)
                        computedMean += points.col(j);
                    computedMean /= points.cols();

                    // Points without mean
                    Eigen::Matrix<double, 4, 5> demeanPts = points;
                    for (int j = 0; j < demeanPts.cols(); ++j)
                        demeanPts.col(j) -= computedMean;

                    // Covariance but without additional division that just scales the eigenvalues
                    Eigen::Matrix3d covariance = demeanPts.topRows<3>() * demeanPts.topRows<3>().transpose();

                    // Computing eigenvalues and eigenvectors
                    Eigen::SelfAdjointEigenSolver <Eigen::Matrix3d> evd(covariance);

                    // Second eigenvalue is at least 3 times smaller than the first one (there is significant change in one direction)
                    Eigen::Vector4d edgeVector = Eigen::Vector4d::Zero();
                    if (evd.eigenvalues()(2) > 3 * evd.eigenvalues()(1)) {
                        edgeVector.head<3>() = evd.eigenvectors().col(2);

                        // Simulating the points of the edge
                        Eigen::Vector4d edgePoint1 = computedMean + 0.1 * edgeVector;
                        Eigen::Vector4d edgePoint2 = computedMean - 0.1 * edgeVector;

                        // Computing the constraint
                        Eigen::Vector3d pointInGlobalEig = pointInGlobal.getVector3fMap().cast<double>();
                        Eigen::Vector4d constraint;
                        bool ok = FeatureUtils::computeEdgeConstraint(pointInGlobalEig, edgePoint1, edgePoint2,
                                                                      constraint);

                        // If constraint was ok then add it to optimization
                        if (ok) {
                            pointsConstraints.push_back(pointInLocal);

                            pcl::PointXYZI jacobianCoeff;
                            jacobianCoeff.x = constraint(0);
                            jacobianCoeff.y = constraint(1);
                            jacobianCoeff.z = constraint(2);
                            jacobianCoeff.intensity = constraint(3);

                            coeffsConstraints.push_back(jacobianCoeff);
                        }
                    }
                }
            }

            clock_t clockConditionEdgesEnd = clock();
            timeMeasurement._numOfCallsMappingConditionsEdges++;
            timeMeasurement._totalTimeMappingConditionsEdges += double(clockConditionEdgesEnd - clockConditionsEdgesBegin);

            clock_t clockConditionsSurfacesBegin = clock();
                    
            int constraintNum1 = 0;
            int constraintNum2 = 0;    
            static int optimizationCnt = 0;
              static unsigned int numOfBadPoints = 0; 
            optimizationCnt++;
            if(mapManager.isEnabled() == false || mapManager.isOptimizationEnabled() == false || optimizationCnt < 0) // Oryginal optimization
            {
                mapManager.debuggingPoints().clear();
                mapManager.pointsUsedForOptimalization().clear();
                mapManager.oryginalOptimalizationPoints().clear();
                for (int i = 0; i < laserCloudSurfDSNum; i++) {
                   // if(constraintNum1 > 2000)
                    //    break;
                    pcl::PointXYZI pointInLocal = _laserCloudSurfLastDS->points[i];
                    pcl::PointXYZI pointInGlobal;
                    moveFromLocalToGlobal(pointInLocal, pointInGlobal, curMapWithOdo);

                    clock_t clockKSearchBegin = clock();

                    std::vector<int> pointSearchInd(5, 0);
                    std::vector<float> pointSearchSqDis(5, 0);
                    kdtreeSurfFromMap.nearestKSearch(pointInGlobal, 5, pointSearchInd, pointSearchSqDis);

                    clock_t clockKSearchEnd = clock();
                    timeMeasurement._numOfCallsMappingKdSearchSurfaces++;
                    timeMeasurement._totalTimeMappingKdSearchSurfaces += double(clockKSearchEnd - clockKSearchBegin);

                    //--------Added to Debug---------
                    if (mapManager.isEnabled() == true)
                    {
                        Eigen::Vector4d pointInGlobalEig4d;
                        pointInGlobalEig4d = pointInGlobal.getVector4fMap().cast<double>();
                        PlaneFeature matchedPF;
                        bool foundPF = mapManager.matchPointToPlane(pointInGlobalEig4d, matchedPF); // Finds plane with Point-Plane and Point-Point distance < 1
                        if (foundPF == true ) //&& matchedPF.pointsNum() > 20)
                        {
                             mapManager.addPointsUsedForOptimalization(pointInGlobal);
                            if (pointSearchSqDis[0] > 1)
                            {
                                mapManager.addDebuggingPoint(pointInGlobal); // Points used by me but not them
                            }
                        }
                    }
                    //----------Added to Debug-----------


                    if (pointSearchSqDis[4] < 1.0) {

                        // Rewriting points to eigen
                        Eigen::MatrixXd points = Eigen::MatrixXd::Zero(4, 5);
                        for (int j = 0; j < 5; j++)
                            points.col(
                                    j).head<3>() = mapSurfs->points[pointSearchInd[j]].getVector3fMap().cast<double>();

                        // Computing plane equation (A, B, C, D)
                        Eigen::Vector4d planeEq = FeatureUtils::computePlaneEq(points);

                        // if all points of the plane are closer to it than 0.2 then the plane is valid
                        bool ok = true;
                        for (int j = 0; j < 5; j++) {
                            double distanceToPlane = fabs(points.col(j).head<3>().dot(planeEq.head<3>()) + planeEq(3));

                            if (distanceToPlane > 0.2) {
                                ok = false;
                                break;
                            }
                        }

                        // If plane eq is valid
                        if (ok) {
                            // Computing the constraint
                            Eigen::Vector4d constraint;
                            Eigen::Vector3d pointInGlobalEig = pointInGlobal.getVector3fMap().cast<double>();
                            ok = FeatureUtils::computeSurfConstraint(pointInGlobalEig, planeEq, constraint);

                            // Adding it to optimization if it is ok
                            if (ok) {
                                pointsConstraints.push_back(pointInLocal);
                                if (mapManager.isEnabled())
                                {
                                    mapManager.addOryginalOptimalizationPoints(pointInGlobal);
                                }
                                pcl::PointXYZI jacobianCoeff;
                                jacobianCoeff.x = constraint(0);
                                jacobianCoeff.y = constraint(1);
                                jacobianCoeff.z = constraint(2);
                                jacobianCoeff.intensity = constraint(3);

                                coeffsConstraints.push_back(jacobianCoeff);
                                constraintNum1++;

                            }
                        }
                    }
                }
            }
      
            else if (mapManager.isOptimizationEnabled() == true )
            {   
                mapManager.debuggingPoints().clear();
                mapManager.pointsUsedForOptimalization().clear();
                mapManager.oryginalOptimalizationPoints().clear();
                for (int i = 0; i < laserCloudSurfDSNum; i++)
                {
                    pcl::PointXYZI pointInLocal = _laserCloudSurfLastDS->points[i];
                    pcl::PointXYZI pointInGlobal;
                    moveFromLocalToGlobal(pointInLocal, pointInGlobal, curMapWithOdo);

                    clock_t clockKSearchBegin = clock();

                      std::vector<int> pointSearchInd(5, 0);
                      std::vector<float> pointSearchSqDis(5, 0);
                      kdtreeSurfFromMap.nearestKSearch(pointInGlobal, 5, pointSearchInd, pointSearchSqDis);

                    Eigen::Vector4d pointInGlobalEig;
                    pointInGlobalEig = pointInGlobal.getVector4fMap().cast<double>();

                    PlaneFeature matchedPF;
                    bool foundPF = mapManager.matchPointToPlane(pointInGlobalEig, matchedPF); // Finds plane with Point-Plane and Point-Point distance < 1
                    
                      
                    clock_t clockKSearchEnd = clock();
                    timeMeasurement._numOfCallsMappingKdSearchSurfaces++;
                    timeMeasurement._totalTimeMappingKdSearchSurfaces += double(clockKSearchEnd - clockKSearchBegin);

                    Eigen::Vector4d planeEqOryg;
                    double distanceToPlane = 999;
                    bool ok2 = true;
                    Eigen::Vector4d planeEq;
                    if (pointSearchSqDis[4] < 1.0)
                    {
                        // Rewriting points to eigen
                        Eigen::MatrixXd points = Eigen::MatrixXd::Zero(4, 5);
                        for (int j = 0; j < 5; j++)
                            points.col(j).head<3>() = mapSurfs->points[pointSearchInd[j]].getVector3fMap().cast<double>();

                        // Computing plane equation (A, B, C, D)
                        planeEq = FeatureUtils::computePlaneEq(points);

                        // if all points of the plane are closer to it than 0.2 then the plane is valid

                        for (int j = 0; j < 5; j++)
                        {
                            double distanceToPlane = fabs(points.col(j).head<3>().dot(planeEq.head<3>()) + planeEq(3));

                            if (distanceToPlane > 0.2)
                            {
                                ok2 = false;
                                break;
                            }
                        }
                    }
                    else
                    {
                        ok2 = false;
                    }

                    if (ok2 == true)
                        mapManager.addOryginalOptimalizationPoints(pointInGlobal);
                    
                    if (foundPF == false)
                        continue;

                  //  if (matchedPF.pointsNum() < 50)
                    //  continue;

                    double pointToPlaneDistance = FeatureUtils::computeDistancePointToPlane(matchedPF.planeEq(), pointInGlobalEig);
                    double pointToPlaneDistanceOryg = FeatureUtils::computeDistancePointToPlane(planeEq, pointInGlobalEig);
                    // Same condition is implemented in matchPointToPlane function
                    //if (fabs(pointToPlaneDistance) < 1)
                    {
                        // if all points of the plane are closer to it than 0.2 then the plane is valid
                        bool ok = true;
                        // Computing the constraint
                        Eigen::Vector4d constraint;
                        Eigen::Vector3d pointInGlobalEig3d = pointInGlobal.getVector3fMap().cast<double>();
                        Eigen::Vector4d planeEq2 = matchedPF.planeEq();
                        ok = FeatureUtils::computeSurfConstraint(pointInGlobalEig3d, planeEq2, constraint);
                        // Adding it to optimization if it is ok
                            if (ok)
                            {
                                // Displays Used planeEq and calculated distance
                               //     std::cout << "--1:--  " << planeEq2.transpose() << " Dist: " << pointToPlaneDistance << " --2:--  " << planeEq.transpose() << " Dist:  " << pointToPlaneDistanceOryg << std::endl;
                                if (ok2 == false)
                                {
                                    mapManager.addDebuggingPoint(pointInGlobal);
                                    numOfBadPoints++;
                                }
                                mapManager.addPointsUsedForOptimalization(pointInGlobal); // My optmimization points

                                pointsConstraints.push_back(pointInLocal);
                                pcl::PointXYZI jacobianCoeff;
                                jacobianCoeff.x = constraint(0);
                                jacobianCoeff.y = constraint(1);
                                jacobianCoeff.z = constraint(2);
                                jacobianCoeff.intensity = constraint(3);
                                coeffsConstraints.push_back(jacobianCoeff);
                                constraintNum2++;
                            }
                    }
                }               
            }
            if (mapManager.isEnabled())
            {
                static int cnt = 0;
                cnt++;
                if (cnt > 30)
                {
                    cnt = 0;
                    std::cout << "Num oryg: " << mapManager.oryginalOptimalizationPoints().size() << "     Num of my points:     " << mapManager.pointsUsedForOptimalization().size() << std::endl;
                }
            }
              

            clock_t clockConditionsSurfacesEnd = clock();
            timeMeasurement._numOfCallsMappingConditionsSurfaces++;
            timeMeasurement._totalTimeMappingConditionsSurfaces += double(clockConditionsSurfacesEnd - clockConditionsSurfacesBegin);

            clock_t clockOptimizationBegin = clock();

            size_t laserCloudSelNum = pointsConstraints.size();
            if (laserCloudSelNum < 50)
                continue;

            // Preparation for a single iteration
            double deltaR = 0.0, deltaT = 0.0;
            double coeffWeight = 1;
            Eigen::Matrix4d dT;

            // Computes a single iteration pass of odometry
            Eigen::Matrix<float, 1, 6> eigenvalues;
            Eigen::Matrix<float, 6, 6> eigenvectors;
            Optimization::poseOptimizationIteration(curMapWithOdo, pointsConstraints, coeffsConstraints, iterCount, isDegenerate, matP,
                                      dT, deltaR, deltaT, coeffWeight, eigenvalues, eigenvectors);

            if(iterCount == 0) {
                eigenvalueStream << currentFrame << " ";
                eigenvalueStream << eigenvalues << " ";
                for (int colId = 0; colId < 6; colId++)
                    eigenvalueStream << eigenvectors.col(colId).transpose()<< " ";
                eigenvalueStream << std::endl;
            }

            // T_i = T_{i-1} * dT
            curMapWithOdo = curMapWithOdo * dT;

            clock_t clockOptimizationEnd = clock();
            timeMeasurement._numOfCallsMappingOptimalization++;
            timeMeasurement._totalTimeMappingOptimalization += double(clockOptimizationEnd - clockOptimizationBegin);

            if (deltaR < _deltaRAbort && deltaT < _deltaTAbort)
                break;
        }

        transformUpdate();
        return true;
    }

    void BasicLaserMapping::moveCubes() {
     
        int centerCubeI = int((curMapWithOdo(0,3) + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenWidth;
        int centerCubeJ = int((curMapWithOdo(1,3) + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenHeight;
        int centerCubeK = int((curMapWithOdo(2,3) + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenDepth;

        if (curMapWithOdo(0,3) + CUBE_HALF < 0) centerCubeI--;
        if (curMapWithOdo(1,3) + CUBE_HALF < 0) centerCubeJ--;
        if (curMapWithOdo(2,3) + CUBE_HALF < 0) centerCubeK--;

        int cnt = 0;
        while (centerCubeI < 3) {                                                                       // Hung here 2 time // CenterCubeI overflow
            for (int j = 0; j < _laserCloudHeight; j++) {
                for (int k = 0; k < _laserCloudDepth; k++) {
                    for (int i = _laserCloudWidth - 1; i >= 1; i--) {
                        const size_t indexA = toIndex(i, j, k);
                        const size_t indexB = toIndex(i - 1, j, k);
                        std::swap(_laserCloudEdgeArray[indexA], _laserCloudEdgeArray[indexB]);
                        std::swap(_laserCloudSurfArray[indexA], _laserCloudSurfArray[indexB]);
                        cnt ++;
                        if(cnt > 400)
                        {
                            std::cout << "Hang 1 : " << curMapWithOdo(0,3) << " " << curMapWithOdo(1,3) << " " << curMapWithOdo(2,3) << std::endl;
                            exit(0);
                        }
                    }
                }
            }
            centerCubeI++;
            _laserCloudCenWidth++;
        }

        int cnt2 = 0;
        while (centerCubeI >= _laserCloudWidth - 3) {                                   // Hung 2 time here
            for (int j = 0; j < _laserCloudHeight; j++) {
                for (int k = 0; k < _laserCloudDepth; k++) {
                    for (int i = 0; i < _laserCloudWidth - 1; i++) {
                        const size_t indexA = toIndex(i, j, k);
                        const size_t indexB = toIndex(i + 1, j, k);
                        std::swap(_laserCloudEdgeArray[indexA], _laserCloudEdgeArray[indexB]);
                        std::swap(_laserCloudSurfArray[indexA], _laserCloudSurfArray[indexB]);
                        cnt2 ++;
                        if(cnt2 > 400) {
                            std::cout << "Hang 2:" << curMapWithOdo(0,3) << " " << curMapWithOdo(1,3) << " " << curMapWithOdo(2,3) << std::endl;
                            exit(0);
                        }
                    }
                }
            }
            centerCubeI--;
            _laserCloudCenWidth--;
        }

        while (centerCubeJ < 3) {
            for (int i = 0; i < _laserCloudWidth; i++) {                                                            
                for (int k = 0; k < _laserCloudDepth; k++) {
                    for (int j = _laserCloudHeight - 1; j >= 1; j--) {
                        const size_t indexA = toIndex(i, j, k);
                        const size_t indexB = toIndex(i, j - 1, k);
                        std::swap(_laserCloudEdgeArray[indexA], _laserCloudEdgeArray[indexB]);
                        std::swap(_laserCloudSurfArray[indexA], _laserCloudSurfArray[indexB]);
                    }
                }
            }
            centerCubeJ++;
            _laserCloudCenHeight++;
        }

        while (centerCubeJ >= _laserCloudHeight - 3) {
            for (int i = 0; i < _laserCloudWidth; i++) {
                for (int k = 0; k < _laserCloudDepth; k++) {
                    for (int j = 0; j < _laserCloudHeight - 1; j++) {
                        const size_t indexA = toIndex(i, j, k);
                        const size_t indexB = toIndex(i, j + 1, k);
                        std::swap(_laserCloudEdgeArray[indexA], _laserCloudEdgeArray[indexB]);
                        std::swap(_laserCloudSurfArray[indexA], _laserCloudSurfArray[indexB]);
                    }
                }
            }
            centerCubeJ--;
            _laserCloudCenHeight--;
        }

        while (centerCubeK < 3) {
            for (int i = 0; i < _laserCloudWidth; i++) {
                for (int j = 0; j < _laserCloudHeight; j++) {
                    for (int k = _laserCloudDepth - 1; k >= 1; k--) {
                        const size_t indexA = toIndex(i, j, k);
                        const size_t indexB = toIndex(i, j, k - 1);
                        std::swap(_laserCloudEdgeArray[indexA], _laserCloudEdgeArray[indexB]);
                        std::swap(_laserCloudSurfArray[indexA], _laserCloudSurfArray[indexB]);
                    }
                }
            }
            centerCubeK++;
            _laserCloudCenDepth++;
        }

        while (centerCubeK >= _laserCloudDepth - 3) {
            for (int i = 0; i < _laserCloudWidth; i++) {
                for (int j = 0; j < _laserCloudHeight; j++) {
                    for (int k = 0; k < _laserCloudDepth - 1; k++) {
                        const size_t indexA = toIndex(i, j, k);
                        const size_t indexB = toIndex(i, j, k + 1);
                        std::swap(_laserCloudEdgeArray[indexA], _laserCloudEdgeArray[indexB]);
                        std::swap(_laserCloudSurfArray[indexA], _laserCloudSurfArray[indexB]);
                    }
                }
            }
            centerCubeK--;
            _laserCloudCenDepth--;
        }
    }

    void BasicLaserMapping::checkFov(std::vector <size_t> &laserCloudValidInd, std::vector <size_t> &laserCloudSurroundInd) {

        // Used to determine the local direction
        pcl::PointXYZI pointOnYAxis;
        pointOnYAxis.x = 0.0;
        pointOnYAxis.y = 10.0;
        pointOnYAxis.z = 0.0;
//        moveFromLocalToGlobal(pointOnYAxis, pointOnYAxis);
        moveFromLocalToGlobal(pointOnYAxis, pointOnYAxis, curMapWithOdo);

        int centerCubeI = int((curMapWithOdo(0,3) + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenWidth;
        int centerCubeJ = int((curMapWithOdo(1,3) + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenHeight;
        int centerCubeK = int((curMapWithOdo(2,3) + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenDepth;

        laserCloudValidInd.clear();
        laserCloudSurroundInd.clear();
        for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++) {
            for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++) {
                for (int k = centerCubeK - 2; k <= centerCubeK + 2; k++) {
                    if (i >= 0 && i < _laserCloudWidth &&
                        j >= 0 && j < _laserCloudHeight &&
                        k >= 0 && k < _laserCloudDepth) {

                        float centerX = 50.0f * (i - _laserCloudCenWidth);
                        float centerY = 50.0f * (j - _laserCloudCenHeight);
                        float centerZ = 50.0f * (k - _laserCloudCenDepth);

                        pcl::PointXYZI transform_pos;
                        transform_pos.x = curMapWithOdo(0,3);
                        transform_pos.y = curMapWithOdo(1,3);
                        transform_pos.z = curMapWithOdo(2,3);

                        bool isInLaserFOV = false;
                        for (int ii = -1; ii <= 1; ii += 2) {
                            for (int jj = -1; jj <= 1; jj += 2) {
                                for (int kk = -1; kk <= 1; kk += 2) {
                                    pcl::PointXYZI corner;
                                    corner.x = centerX + 25.0f * ii;
                                    corner.y = centerY + 25.0f * jj;
                                    corner.z = centerZ + 25.0f * kk;

                                    float squaredSide1 = calcSquaredDiff(transform_pos, corner);
                                    float squaredSide2 = calcSquaredDiff(pointOnYAxis, corner);

                                    float check1 = 100.0f + squaredSide1 - squaredSide2
                                                   - 10.0f * sqrt(3.0f) * sqrt(squaredSide1);

                                    float check2 = 100.0f + squaredSide1 - squaredSide2
                                                   + 10.0f * sqrt(3.0f) * sqrt(squaredSide1);

                                    if (check1 < 0 && check2 > 0) {
                                        isInLaserFOV = true;
                                    }
                                }
                            }
                        }

                        size_t cubeIdx = i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k;
                        if (isInLaserFOV) {
                            laserCloudValidInd.push_back(cubeIdx);
                        }
                        laserCloudSurroundInd.push_back(cubeIdx);
                    }
                }
            }
        }
    }

    void BasicLaserMapping::addEdgesAndSurfsToCubes() {
        size_t laserCloudEdgeDSNum = _laserCloudEdgeLastDS->size();
        // store down sized edge DS points in corresponding cube clouds
        for (int i = 0; i < laserCloudEdgeDSNum; i++) {
            pcl::PointXYZI pointInGlobal;
//            moveFromLocalToGlobal(_laserCloudEdgeLastDS->points[i], pointInGlobal);
            moveFromLocalToGlobal(_laserCloudEdgeLastDS->points[i], pointInGlobal, curMapWithOdo);

            int cubeI = int((pointInGlobal.x + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenWidth;
            int cubeJ = int((pointInGlobal.y + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenHeight;
            int cubeK = int((pointInGlobal.z + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenDepth;

            if (pointInGlobal.x + CUBE_HALF < 0) cubeI--;
            if (pointInGlobal.y + CUBE_HALF < 0) cubeJ--;
            if (pointInGlobal.z + CUBE_HALF < 0) cubeK--;

            if (cubeI >= 0 && cubeI < _laserCloudWidth &&
                cubeJ >= 0 && cubeJ < _laserCloudHeight &&
                cubeK >= 0 && cubeK < _laserCloudDepth) {
                size_t cubeInd = cubeI + _laserCloudWidth * cubeJ + _laserCloudWidth * _laserCloudHeight * cubeK;
                _laserCloudEdgeArray[cubeInd]->push_back(pointInGlobal);
            }
        }

        size_t laserCloudSurfStackNum = _laserCloudSurfLastDS->size();
        // store down sized surface stack points in corresponding cube clouds
        for (int i = 0; i < laserCloudSurfStackNum; i++) {
            pcl::PointXYZI pointInGlobal;
//            moveFromLocalToGlobal(_laserCloudSurfLastDS->points[i], pointInGlobal);
            moveFromLocalToGlobal(_laserCloudEdgeLastDS->points[i], pointInGlobal, curMapWithOdo);

            int cubeI = int((pointInGlobal.x + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenWidth;
            int cubeJ = int((pointInGlobal.y + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenHeight;
            int cubeK = int((pointInGlobal.z + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenDepth;

            if (pointInGlobal.x + CUBE_HALF < 0) cubeI--;
            if (pointInGlobal.y + CUBE_HALF < 0) cubeJ--;
            if (pointInGlobal.z + CUBE_HALF < 0) cubeK--;

            if (cubeI >= 0 && cubeI < _laserCloudWidth &&
                cubeJ >= 0 && cubeJ < _laserCloudHeight &&
                cubeK >= 0 && cubeK < _laserCloudDepth) {
                size_t cubeInd = cubeI + _laserCloudWidth * cubeJ + _laserCloudWidth * _laserCloudHeight * cubeK;
                _laserCloudSurfArray[cubeInd]->push_back(pointInGlobal);
            }
        }
    }


} // end namespace loam