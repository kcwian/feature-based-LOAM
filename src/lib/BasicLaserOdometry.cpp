#include "loam_velodyne/BasicLaserOdometry.h"

#include "../../include/loam_velodyne/math_utils.h"
#include <pcl/filters/filter.h>
#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include <chrono>       // high_resolution_clock - Wall clock 
#include <ctime>        // clock() - CPU Time
#include "loam_velodyne/FeatureUtils.h"
#include "loam_velodyne/LieAlgebra.h"

#include "loam_velodyne/Optimization.h"

using namespace std::chrono;

namespace loam {

    using std::sin;
    using std::cos;
    using std::asin;
    using std::atan2;
    using std::sqrt;
    using std::fabs;
    using std::pow;


    BasicLaserOdometry::BasicLaserOdometry(float scanPeriod, size_t maxIterations) :
            _scanPeriod(scanPeriod),
            _systemInited(false),
            _frameCount(0),
            _maxIterations(maxIterations),
            _deltaTAbort(0.1),
            _deltaRAbort(0.1),
            _numOfCallsOdometryKdSearchCorners(0),
            _totalTimeOdometryKdSearchCorners(0),
            _numOfCallsOdometryKdSearchSurfaces(0),
            _totalTimeOdometryKdSearchSurfaces(0),
            _numOfCallsOdometryProcess(0),
            _totalTimeOdometryProcess(0),
            _numOfCallsOdometryConditionsCorners(0),
            _totalTimeOdometryConditionsCorners(0),
            _numOfCallsOdometryConditionsSurfaces(0),
            _totalTimeOdometryConditionsSurfaces(0),
            _numOfCallsOdometryOptimalization(0),
            _totalTimeOdometryOptimalization(0),
            _numOfCallsOdometryEndCalculations(0),
            _totalTimeOdometryEndCalculations(0),
            _curEdgePoints(new pcl::PointCloud<pcl::PointXYZI>()),
            _curEdgePointsDS(new pcl::PointCloud<pcl::PointXYZI>()),
            _curSurfPoints(new pcl::PointCloud<pcl::PointXYZI>()),
            _curSurfPointsDS(new pcl::PointCloud<pcl::PointXYZI>()),
            _laserCloud(new pcl::PointCloud<pcl::PointXYZI>()),
            _lastEdgePointsDS(new pcl::PointCloud<pcl::PointXYZI>()),
            _lastSurfPointsDS(new pcl::PointCloud<pcl::PointXYZI>()) {

        invPoseInPrev = Eigen::Matrix4d::Identity();
        accPoseInGlobal = Eigen::Matrix4d::Identity();

        poseIntInGlobal = Eigen::Matrix4d::Identity();
        invPoseInPrevScan = Eigen::Matrix4d::Identity();

        eigenvalueStream.open("odometry_eigenvalues.txt");
    }

    void BasicLaserOdometry::transformToStart(const pcl::PointXYZI &pi, pcl::PointXYZI &po,
                                            const Eigen::Matrix4d &invPose) {
        transformToStart(pi, po, LieAlgebra::log(invPose));
    }

    void BasicLaserOdometry::transformToStart(const pcl::PointXYZI &pi, pcl::PointXYZI &po, const Vector6d &invPoseLog) {
        // Computing the linear location in the scan
        float s = (1.f / _scanPeriod) * (pi.intensity - int(pi.intensity));

        Vector6d invPoseInterpolatedLog = s * invPoseLog;
        Eigen::Matrix4d invPoseInterpolated = LieAlgebra::exp(invPoseInterpolatedLog);

        // to homogeneous
        Eigen::Vector4d coordPi;
        coordPi = pi.getVector4fMap().cast<double>();
        coordPi(3) = 1;

        // transform
        Eigen::Vector4d coordPo = LieAlgebra::inv(invPoseInterpolated) * coordPi;

        // to cartesian
        po.getVector3fMap() = coordPo.head<3>().cast<float>();
        po.intensity = pi.intensity;
    }

    size_t BasicLaserOdometry::transformToEnd(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) {
        size_t cloudSize = cloud->points.size();

        for (size_t i = 0; i < cloudSize; i++) {
            pcl::PointXYZI &point = cloud->points[i];

            pcl::PointXYZI pointInStart;
            transformToStart(point, pointInStart, invPoseInPrev);
            
            // transform to start
            Eigen::Vector4d pointInStartEig = pointInStart.getVector4fMap().cast<double>();
            pointInStartEig(3) = 1;

            // transform to end
            Eigen::Vector4d pointInEnd = invPoseInPrev * pointInStartEig;

            // TODO: Ignored IMU PART
//                        point.x += _transform.pos.x() - _imuShiftFromStart.x();
//            point.y += _transform.pos.y() - _imuShiftFromStart.y();
//            point.z += _transform.pos.z() - _imuShiftFromStart.z();
//
//            rotateZXY(point, _imuRollStart, _imuPitchStart, _imuYawStart);
//            rotateYXZ(point, -_imuYawEnd, -_imuPitchEnd, -_imuRollEnd);

            point.getArray3fMap() = pointInEnd.head<3>().cast<float>();
        }

        return cloudSize;
    }


//    void BasicLaserOdometry::pluginIMURotation(const Angle &bcx, const Angle &bcy, const Angle &bcz,
//                                               const Angle &blx, const Angle &bly, const Angle &blz,
//                                               const Angle &alx, const Angle &aly, const Angle &alz,
//                                               Angle &acx, Angle &acy, Angle &acz) {
//        float sbcx = bcx.sin();
//        float cbcx = bcx.cos();
//        float sbcy = bcy.sin();
//        float cbcy = bcy.cos();
//        float sbcz = bcz.sin();
//        float cbcz = bcz.cos();
//
//        float sblx = blx.sin();
//        float cblx = blx.cos();
//        float sbly = bly.sin();
//        float cbly = bly.cos();
//        float sblz = blz.sin();
//        float cblz = blz.cos();
//
//        float salx = alx.sin();
//        float calx = alx.cos();
//        float saly = aly.sin();
//        float caly = aly.cos();
//        float salz = alz.sin();
//        float calz = alz.cos();
//
//        float srx = -sbcx * (salx * sblx + calx * caly * cblx * cbly + calx * cblx * saly * sbly)
//                    - cbcx * cbcz * (calx * saly * (cbly * sblz - cblz * sblx * sbly)
//                                     - calx * caly * (sbly * sblz + cbly * cblz * sblx) + cblx * cblz * salx)
//                    - cbcx * sbcz * (calx * caly * (cblz * sbly - cbly * sblx * sblz)
//                                     - calx * saly * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sblz);
//        acx = -asin(srx);
//
//        float srycrx = (cbcy * sbcz - cbcz * sbcx * sbcy) * (calx * saly * (cbly * sblz - cblz * sblx * sbly)
//                                                             - calx * caly * (sbly * sblz + cbly * cblz * sblx) +
//                                                             cblx * cblz * salx)
//                       - (cbcy * cbcz + sbcx * sbcy * sbcz) * (calx * caly * (cblz * sbly - cbly * sblx * sblz)
//                                                               - calx * saly * (cbly * cblz + sblx * sbly * sblz) +
//                                                               cblx * salx * sblz)
//                       + cbcx * sbcy * (salx * sblx + calx * caly * cblx * cbly + calx * cblx * saly * sbly);
//        float crycrx = (cbcz * sbcy - cbcy * sbcx * sbcz) * (calx * caly * (cblz * sbly - cbly * sblx * sblz)
//                                                             - calx * saly * (cbly * cblz + sblx * sbly * sblz) +
//                                                             cblx * salx * sblz)
//                       - (sbcy * sbcz + cbcy * cbcz * sbcx) * (calx * saly * (cbly * sblz - cblz * sblx * sbly)
//                                                               - calx * caly * (sbly * sblz + cbly * cblz * sblx) +
//                                                               cblx * cblz * salx)
//                       + cbcx * cbcy * (salx * sblx + calx * caly * cblx * cbly + calx * cblx * saly * sbly);
//        acy = atan2(srycrx / acx.cos(), crycrx / acx.cos());
//
//        float srzcrx = sbcx * (cblx * cbly * (calz * saly - caly * salx * salz) -
//                               cblx * sbly * (caly * calz + salx * saly * salz) + calx * salz * sblx)
//                       - cbcx * cbcz * ((caly * calz + salx * saly * salz) * (cbly * sblz - cblz * sblx * sbly)
//                                        + (calz * saly - caly * salx * salz) * (sbly * sblz + cbly * cblz * sblx)
//                                        - calx * cblx * cblz * salz)
//                       + cbcx * sbcz * ((caly * calz + salx * saly * salz) * (cbly * cblz + sblx * sbly * sblz)
//                                        + (calz * saly - caly * salx * salz) * (cblz * sbly - cbly * sblx * sblz)
//                                        + calx * cblx * salz * sblz);
//        float crzcrx = sbcx * (cblx * sbly * (caly * salz - calz * salx * saly) -
//                               cblx * cbly * (saly * salz + caly * calz * salx) + calx * calz * sblx)
//                       + cbcx * cbcz * ((saly * salz + caly * calz * salx) * (sbly * sblz + cbly * cblz * sblx)
//                                        + (caly * salz - calz * salx * saly) * (cbly * sblz - cblz * sblx * sbly)
//                                        + calx * calz * cblx * cblz)
//                       - cbcx * sbcz * ((saly * salz + caly * calz * salx) * (cblz * sbly - cbly * sblx * sblz)
//                                        + (caly * salz - calz * salx * saly) * (cbly * cblz + sblx * sbly * sblz)
//                                        - calx * calz * cblx * sblz);
//        acz = atan2(srzcrx / acx.cos(), crzcrx / acx.cos());
//    }


    // TODO: When IMU is used
//    void BasicLaserOdometry::accumulateRotation(Angle cx, Angle cy, Angle cz,
//                                                Angle lx, Angle ly, Angle lz,
//                                                Angle &ox, Angle &oy, Angle &oz) {
//        float srx = lx.cos() * cx.cos() * ly.sin() * cz.sin()
//                    - cx.cos() * cz.cos() * lx.sin()
//                    - lx.cos() * ly.cos() * cx.sin();
//        ox = -asin(srx);
//
//        float srycrx = lx.sin() * (cy.cos() * cz.sin() - cz.cos() * cx.sin() * cy.sin())
//                       + lx.cos() * ly.sin() * (cy.cos() * cz.cos() + cx.sin() * cy.sin() * cz.sin())
//                       + lx.cos() * ly.cos() * cx.cos() * cy.sin();
//        float crycrx = lx.cos() * ly.cos() * cx.cos() * cy.cos()
//                       - lx.cos() * ly.sin() * (cz.cos() * cy.sin() - cy.cos() * cx.sin() * cz.sin())
//                       - lx.sin() * (cy.sin() * cz.sin() + cy.cos() * cz.cos() * cx.sin());
//        oy = atan2(srycrx / ox.cos(), crycrx / ox.cos());
//
//        float srzcrx = cx.sin() * (lz.cos() * ly.sin() - ly.cos() * lx.sin() * lz.sin())
//                       + cx.cos() * cz.sin() * (ly.cos() * lz.cos() + lx.sin() * ly.sin() * lz.sin())
//                       + lx.cos() * cx.cos() * cz.cos() * lz.sin();
//        float crzcrx = lx.cos() * lz.cos() * cx.cos() * cz.cos()
//                       - cx.cos() * cz.sin() * (ly.cos() * lz.sin() - lz.cos() * lx.sin() * ly.sin())
//                       - cx.sin() * (ly.sin() * lz.sin() + ly.cos() * lz.cos() * lx.sin());
//        oz = atan2(srzcrx / ox.cos(), crzcrx / ox.cos());
//    }

    // TODO: When IMU is used
//    void BasicLaserOdometry::updateIMU(pcl::PointCloud<pcl::PointXYZ> const &imuTrans) {
//        assert(4 == imuTrans.size());
//        _imuPitchStart = imuTrans.points[0].x;
//        _imuYawStart = imuTrans.points[0].y;
//        _imuRollStart = imuTrans.points[0].z;
//
//        _imuPitchEnd = imuTrans.points[1].x;
//        _imuYawEnd = imuTrans.points[1].y;
//        _imuRollEnd = imuTrans.points[1].z;
//
//        _imuShiftFromStart = imuTrans.points[2];
//        _imuVeloFromStart = imuTrans.points[3];
//    }

    void BasicLaserOdometry::process() {

        // Called at the initialization
        if (!_systemInited) {
            _curEdgePointsDS.swap(_lastEdgePointsDS);
            _curSurfPointsDS.swap(_lastSurfPointsDS);

            _lastCornerKDTree.setInputCloud(_lastEdgePointsDS);
            _lastSurfaceKDTree.setInputCloud(_lastSurfPointsDS);

            // TODO: When IMU is used
//            _transformSum.rot_x += _imuPitchStart;
//            _transformSum.rot_z += _imuRollStart;

            _systemInited = true;
            return;
        }


        _frameCount++;
        // TODO: When IMU is used
        //        _transform.pos -= _imuVeloFromStart * _scanPeriod;


        clock_t clockProcessBegin = clock();

        // Checking if we have sufficient features from last scan
        size_t lastEdgeCloudSize = _lastEdgePointsDS->points.size();
        size_t lastSurfaceCloudSize = _lastSurfPointsDS->points.size();

        if (lastEdgeCloudSize > 10 && lastSurfaceCloudSize > 100) {

            // Removing Nans from cloud
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*_curEdgePoints, *_curEdgePoints, indices);

            // The sizes of the current scan
            size_t curEdgeCloudSize = _curEdgePoints->points.size();
            size_t curSurfCloudSize = _curSurfPoints->points.size();

            // Place to store found neighbours: Nx2 for edges and Nx3 for surfs
            std::vector< std::vector<int>> pointSearchEdgeInd(curEdgeCloudSize, std::vector<int>(2,-1));
            std::vector< std::vector<int>> pointSearchSurfInd(curSurfCloudSize, std::vector<int>(3,-1));

            // Variables to detect if optimization is well-constrained in all directions
            bool isDegenerate = false;
            Eigen::Matrix<float, 6, 6> matP;

            // Iterating to determine the solution
            for (size_t iterCount = 0; iterCount < _maxIterations; iterCount++) {

                // Information about the constraints for optimization
                pcl::PointCloud <pcl::PointXYZI> pointsConstraints, coeffsConstraints;

                // Getting the log map of transformation
                Eigen::Vector6d invPoseInPrevLog = LieAlgebra::log(invPoseInPrev);

                clock_t clockConditionsCornersBegin = clock();

                // For every edge constraint
                for (int i = 0; i < curEdgeCloudSize; i++) {

                    // Compute its location at the beginning of the scan based on current transformation estimate
                    pcl::PointXYZI pointAtStart;
                    transformToStart(_curEdgePoints->points[i], pointAtStart, invPoseInPrevLog);

                    // Every 5th iteration we look for the closest point
                    if (iterCount % 5 == 0) {

                        // Removing NaNs
                        std::vector<int> indices;
                        pcl::removeNaNFromPointCloud(*_lastEdgePointsDS, *_lastEdgePointsDS, indices);

                        clock_t clockKSearchBegin = clock();

                        // Finding the closest point
                        std::vector<int> pointSearchInd(1);
                        std::vector<float> pointSearchSqDis(1);
                        _lastCornerKDTree.nearestKSearch(pointAtStart, 1, pointSearchInd, pointSearchSqDis);

                        clock_t clockKSearchEnd = clock();
                        _numOfCallsOdometryKdSearchCorners++;
                        _totalTimeOdometryKdSearchCorners += double(clockKSearchEnd - clockKSearchBegin);

                        // The closest point is closer than 25 meters
                        int closestPointInd = -1, secondClosestPointInd = -1;
                        if (pointSearchSqDis[0] < 25) {

                            // We determine the scan line id for the closest point
                            closestPointInd = pointSearchInd[0];
                            int closestPointScanID = int(_lastEdgePointsDS->points[closestPointInd].intensity);

                            // Finding closest point in the next 2 rings
                            float dist2secondClosestPoint = 25;
                            for (int j = closestPointInd + 1; j < curEdgeCloudSize; j++) {

                                // The point is further than 2 rings from scan
                                if (int(_lastEdgePointsDS->points[j].intensity) > closestPointScanID + 2.5)
                                    break;

                                // The point is not on the same line as the closest point
                                if (int(_lastEdgePointsDS->points[j].intensity) > closestPointScanID) // TODO: WHY?
                                {
                                    // Distance between points
                                    float pointSqDis = calcSquaredDiff(_lastEdgePointsDS->points[j], pointAtStart);

                                    // It is closer to the original one than the secondBest
                                    if (pointSqDis < dist2secondClosestPoint) {
                                        dist2secondClosestPoint = pointSqDis;
                                        secondClosestPointInd = j;
                                    }
                                }
                            }

                            // Finding closest point in the previous 2 rings
                            for (int j = closestPointInd - 1; j >= 0; j--) {

                                // The point is further than 2 rings from scan
                                if (int(_lastEdgePointsDS->points[j].intensity) < closestPointScanID - 2.5)
                                    break;

                                // The point is not on the same line
                                if (int(_lastEdgePointsDS->points[j].intensity) < closestPointScanID) // TODO: WHY?
                                {
                                    // Distance between points
                                    float pointSqDis = calcSquaredDiff(_lastEdgePointsDS->points[j], pointAtStart);

                                    // It is closer to the original one than the secondBest
                                    if (pointSqDis < dist2secondClosestPoint) {
                                        dist2secondClosestPoint = pointSqDis;
                                        secondClosestPointInd = j;
                                    }
                                }
                            }
                        }

                        // Two closest points: one from kd-tree (closestPointInd) and one from 4 neighbouring rings (secondClosestPointInd)
                        pointSearchEdgeInd[i][0] = closestPointInd;
                        pointSearchEdgeInd[i][1] = secondClosestPointInd;
                    }

                    // If we found the 2 closest points (now or last time)
//                    if (pointSearchEdgeInd[i][0] >= 0 && pointSearchEdgeInd[i][1] >= 0) {
                        if (pointSearchEdgeInd[i][0] >= 0 && pointSearchEdgeInd[i][2] >= 0) {

                        // Edge points in Eigen
                        Eigen::Vector4d edgePoint1 = _lastEdgePointsDS->points[pointSearchEdgeInd[i][0]].getVector4fMap().cast<double>();
                        Eigen::Vector4d edgePoint2 = _lastEdgePointsDS->points[pointSearchEdgeInd[i][1]].getVector4fMap().cast<double>();

                        // The same as an original // TODO: It is 0 and then 1.8 for odometry, 0.9 for mapping - test different values
                        float s = 0;
                        if (iterCount >= 5) {
                            s = 1.8f;
                        }

                        // Computing the constraint
                        Eigen::Vector3d pointInGlobalEig = pointAtStart.getVector3fMap().cast<double>();
                        Eigen::Vector4d constraint;
                        bool ok = FeatureUtils::computeEdgeConstraint(pointInGlobalEig, edgePoint1, edgePoint2,
                                                                      constraint, s);

                        // If constraint was ok then add it to optimization
                        if (ok) {
                            pointsConstraints.push_back(_curEdgePoints->points[i]);

                            pcl::PointXYZI jacobianCoeff;
                            jacobianCoeff.x = constraint(0);
                            jacobianCoeff.y = constraint(1);
                            jacobianCoeff.z = constraint(2);
                            jacobianCoeff.intensity = constraint(3);

                            coeffsConstraints.push_back(jacobianCoeff);
                        }
                    }
                }

                clock_t clockConditionsCornersEnd = clock();
                _numOfCallsOdometryConditionsCorners++;
                _totalTimeOdometryConditionsCorners += double(clockConditionsCornersEnd - clockConditionsCornersBegin);

                clock_t clockConditionsSurfacesBegin = clock();

                // For every current surface point
                for (int i = 0; i < curSurfCloudSize; i++) {

                    // Compute its location from the start of the scan
                    pcl::PointXYZI pointAtStart;
                    transformToStart(_curSurfPoints->points[i], pointAtStart, invPoseInPrevLog);

                    // Every 5th iteration we look for closest point to reduce computation
                    if (iterCount % 5 == 0) {
                        clock_t clockKSearchSurfBegin = clock();

                        // Looking for the closest surf point
                        std::vector<int> pointSearchInd(1);
                        std::vector<float> pointSearchSqDis(1);
                        _lastSurfaceKDTree.nearestKSearch(pointAtStart, 1, pointSearchInd, pointSearchSqDis);

                        clock_t clockKSearchSurfEnd = clock();
                        _numOfCallsOdometryKdSearchSurfaces++;
                        _totalTimeOdometryKdSearchSurfaces += double(clockKSearchSurfEnd - clockKSearchSurfBegin);

                        // If the closest is closer than 25 meters
                        int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
                        if (pointSearchSqDis[0] < 25) {

                            // We determine the ring line of the point
                            closestPointInd = pointSearchInd[0];
                            int closestPointScanID = int(_lastSurfPointsDS->points[closestPointInd].intensity);

                            // Finding 2nd and 3rd closest point from next 2 scan rings
                            float dist2SecondClosest = 25, dist2thirdClosest = 25;
                            for (int j = closestPointInd + 1; j < curSurfCloudSize; j++) {

                                // We only consider points in two neighouring rings
                                if (int(_lastSurfPointsDS->points[j].intensity) > closestPointScanID + 2.5)
                                    break;

                                // Computing distance
                                float pointSqDis = calcSquaredDiff(_lastSurfPointsDS->points[j], pointAtStart);

                                // We only consider points ...
                                if (int(_lastSurfPointsDS->points[j].intensity) <= closestPointScanID)
                                {
                                    if (pointSqDis < dist2SecondClosest) {
                                        dist2SecondClosest = pointSqDis;
                                        minPointInd2 = j;
                                    }
                                }
                                else
                                {
                                    if (pointSqDis < dist2thirdClosest) {
                                        dist2thirdClosest = pointSqDis;
                                        minPointInd3 = j;
                                    }
                                }
                            }

                            // Finding closest point from previous 2 scan rings
                            for (int j = closestPointInd - 1; j >= 0; j--) {
                                if (int(_lastSurfPointsDS->points[j].intensity) < closestPointScanID - 2.5)
                                {
                                    break;
                                }

                                float pointSqDis = calcSquaredDiff(_lastSurfPointsDS->points[j], pointAtStart);

                                if (int(_lastSurfPointsDS->points[j].intensity) >= closestPointScanID)
                                {
                                    if (pointSqDis < dist2SecondClosest) {
                                        dist2SecondClosest = pointSqDis;
                                        minPointInd2 = j;
                                    }
                                }
                                else
                                {
                                    if (pointSqDis < dist2thirdClosest) {
                                        dist2thirdClosest = pointSqDis;
                                        minPointInd3 = j;
                                    }
                                }
                            }
                        }

                        // Indices of three points used to comput plane equation
                        pointSearchSurfInd[i][0] = closestPointInd;
                        pointSearchSurfInd[i][1] = minPointInd2;
                        pointSearchSurfInd[i][2] = minPointInd3;
                    }

                    // if we have enough points to compute plane equation
//                    if (pointSearchSurfInd[i][0] >= 0 && pointSearchSurfInd[i][1] >= 0 && pointSearchSurfInd[i][2] >= 0) {
                    if (pointSearchSurfInd[i][1] >= 0 && pointSearchSurfInd[i][2] >= 0) {

                        // Rewriting points to eigen
                        Eigen::MatrixXd points = Eigen::MatrixXd::Zero(4, 3);
                        for (int j=0;j<3;j++)
                            points.col(j).head<3>() = _lastSurfPointsDS->points[pointSearchSurfInd[i][j]].getVector3fMap().cast<double>();

                        // Computing plane equation (A, B, C, D)
                        Eigen::Vector4d planeEq = FeatureUtils::computePlaneEq(points);

                        // The same as an original // TODO: It is 0 and then 1.8 for odometry, 0.9 for mapping - test different values
                        float s = 0;
                        if (iterCount >= 5) {
                            s = 1.8f;
                        }

                        // Computing the constraint
                        Eigen::Vector4d constraint;
                        Eigen::Vector3d pointInGlobalEig = pointAtStart.getVector3fMap().cast<double>();
                        bool ok = FeatureUtils::computeSurfConstraint(pointInGlobalEig, planeEq, constraint, s); // TODO: It is without sqrt(dist) for now

                        // Adding it to optimization if it is ok
                        if (ok) {
                            pointsConstraints.push_back(_curSurfPoints->points[i]);

                            pcl::PointXYZI jacobianCoeff;
                            jacobianCoeff.x = constraint(0);
                            jacobianCoeff.y = constraint(1);
                            jacobianCoeff.z = constraint(2);
                            jacobianCoeff.intensity = constraint(3);

                            coeffsConstraints.push_back(jacobianCoeff);
                        }
                    }
                }

                clock_t clockConditionsSurfacesEnd = clock();
                _numOfCallsOdometryConditionsSurfaces++;
                _totalTimeOdometryConditionsSurfaces += double(
                        clockConditionsSurfacesEnd - clockConditionsSurfacesBegin);

                // We will not optimize if we have less than 10 constraints
                int pointSelNum = pointsConstraints.size();
                if (pointSelNum < 10) {
                    continue;
                }

                clock_t clockOptimizationBegin = clock();

//                // Jacobian and right side
//                Eigen::Matrix<float, Eigen::Dynamic, 6> matJ(pointSelNum, 6);
//                Eigen::Matrix<float, 6, Eigen::Dynamic> matJt(6, pointSelNum);
//                Eigen::VectorXf matB(pointSelNum);
//
//                // For every point fill the Jacobians
//                for (int i = 0; i < pointSelNum; i++) {
//                    const pcl::PointXYZI &pointOri = pointsConstraints.points[i];
//                    pcl::PointXYZI coeff = coeffsConstraints.points[i];
//
//                    // Normal of constraints
//                    const Eigen::Vector3d &n = coeff.getVector3fMap().cast<double>();
//
//                    // Current rotation
//                    Eigen::Matrix4d T = LieAlgebra::inv(invPoseInPrev);
//                    const Eigen::Matrix3d &R = T.block<3, 3>(0, 0);
//
//                    // Computing jacobians
//                    Eigen::Matrix3d skewP = LieAlgebra::skew(pointOri.getVector3fMap().cast<double>());
//                    Eigen::Vector3d jacobianRotation = n.transpose() * R * (-skewP);
//                    Eigen::Vector3d jacobianTranslation = n.transpose() * R;
//
//                    // Filling A matrix
//                    matJ(i, 0) = jacobianRotation(0);
//                    matJ(i, 1) = jacobianRotation(1);
//                    matJ(i, 2) = jacobianRotation(2);
//                    matJ(i, 3) = jacobianTranslation(0);
//                    matJ(i, 4) = jacobianTranslation(1);
//                    matJ(i, 5) = jacobianTranslation(2);
//                    matB(i, 0) = -0.05 * coeff.intensity;
//                }
//
//                // Gauss Newton step - J^T * J * X = J^T * B
//                matJt = matJ.transpose();
//                Eigen::Matrix<float, 6, 6> matJtJ = matJt * matJ;
//                Eigen::Matrix<float, 6, 1> matJtB = matJt * matB;
//
//                // Finding the increment
//                Eigen::Matrix<float, 6, 1> matX = matJtJ.colPivHouseholderQr().solve(matJtB);
//
//                // For first iteration we verify eigenvalues
//                if (iterCount == 0) {
//
//                    // Finding eigenvalues and eigenvectors of JtJ
//                    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, 6, 6> > esolver(matJtJ);
//                    Eigen::Matrix<float, 1, 6> matE = esolver.eigenvalues().real();
//                    Eigen::Matrix<float, 6, 6> matVcols = esolver.eigenvectors().real();
//
//                    // Reversing the order and saving as rows
//                    Eigen::Matrix<float, 6, 6> matV = matVcols.rowwise().reverse();
//                    matV.transposeInPlace();
//
//                    Eigen::Matrix<float, 6, 6> matV2 = matV;
//
//                    isDegenerate = false;
////                    float eignThre[6] = {10, 10, 10, 10, 10, 10}; // TODO: It is different between versions
//                    float eignThre[6] = {100, 100, 100, 100, 100, 100};
//                    for (int i = 0; i < 6; i++) {
//                        if (matE(0, i) < eignThre[i]) {
//                            for (int j = 0; j < 6; j++) {
//                                // Eigenvectors as rows
//                                matV2(5-i,j) = 0;
//                            }
//                            isDegenerate = true;
//                        } else {
//                            break;
//                        }
//                    }
//                    matP = matV.inverse() * matV2;
//                }
//
//                // Reducing the influence in directions that are poorly constrained
//                if (isDegenerate) {
//                    Eigen::Matrix<float, 6, 1> matX2(matX);
//                    matX = matP * matX2;
//                }
//
//                // Computing the increment
//                Vector6d eps(matX.col(0));

                // Adding the increment to estimated pose
//                Eigen::Matrix4d dT = LieAlgebra::exp(eps);
//                invPoseInPrev = LieAlgebra::inv(dT) * invPoseInPrev;

                // Stop conditions
//                double deltaR = rad2deg(eps.head<3>().norm());
//                double deltaT = eps.tail<3>().norm() * 100.0;

                double deltaR = 0.0, deltaT = 0.0;
                double coeffWeight = 0.05;
                Eigen::Matrix4d poseInPrev = LieAlgebra::inv(invPoseInPrev);
                Eigen::Matrix4d dT;

                // Computes a single iteration pass of odometry
                // TODO: Saving eigenvalues and eigenvectors
                Eigen::Matrix<float, 1, 6> eigenvalues;
                Eigen::Matrix<float, 6, 6> eigenvectors;
                Optimization::poseOptimizationIteration(poseInPrev, pointsConstraints, coeffsConstraints, iterCount, isDegenerate, matP,
                                                dT, deltaR, deltaT, coeffWeight, eigenvalues, eigenvectors);

                if(iterCount == 0) {
                    eigenvalueStream << _frameCount << " ";
                    eigenvalueStream << eigenvalues << " ";
                    for (int colId = 0; colId < 6; colId++)
                        eigenvalueStream << eigenvectors.col(colId).transpose()<< " ";
                    eigenvalueStream << std::endl;
                }

                // Adding increment
                invPoseInPrev = LieAlgebra::inv(dT) * invPoseInPrev;

                clock_t clockOptimizationEnd = clock();
                _numOfCallsOdometryOptimalization++;
                _totalTimeOdometryOptimalization += double(clockOptimizationEnd - clockOptimizationBegin);

                if (deltaR < _deltaRAbort && deltaT < _deltaTAbort) {
                    break;

                }
            }
        }

        clock_t clockEndCalculationsBegin = clock();

//        Angle rx, ry, rz;
//        accumulateRotation(_transformSum.rot_x,
//                           _transformSum.rot_y,
//                           _transformSum.rot_z,
//                           -_transform.rot_x,
//                           -_transform.rot_y.rad() * 1.05,
//                           -_transform.rot_z,
//                           rx, ry, rz);

        // TODO: Totally ignoring the IMU aspect
        Eigen::Matrix4d poseInPrev = LieAlgebra::inv(invPoseInPrev);

        if(!poseInPrev.allFinite()){
            poseInPrev = Eigen::Matrix4d::Identity();
            invPoseInPrev =  Eigen::Matrix4d::Identity();
            //exit(0); // Sometimes helps - Laser Odometry restarts when crashed at the beginning
        }

        accPoseInGlobal = accPoseInGlobal * poseInPrev;
//
//        Vector3 v(_transform.pos.x() - _imuShiftFromStart.x(),
//                  _transform.pos.y() - _imuShiftFromStart.y(),
//                  _transform.pos.z() * 1.05 - _imuShiftFromStart.z());
//        rotateZXY(v, rz, rx, ry);
//        Vector3 trans = _transformSum.pos - v;
//
//        pluginIMURotation(rx, ry, rz,
//                          _imuPitchStart, _imuYawStart, _imuRollStart,
//                          _imuPitchEnd, _imuYawEnd, _imuRollEnd,
//                          rx, ry, rz);
//
//        _transformSum.rot_x = rx;
//        _transformSum.rot_y = ry;
//        _transformSum.rot_z = rz;
//        _transformSum.pos = trans;

        // Moving points to the end of the scan
        transformToEnd(_curEdgePointsDS);
        transformToEnd(_curSurfPointsDS);

        // Current DS points are becoming last DS points
        _curEdgePointsDS.swap(_lastEdgePointsDS);
        _curSurfPointsDS.swap(_lastSurfPointsDS);

        // We update the kd trees if everything is ok // TODO: What happens when we do not have enough features for update?
        lastEdgeCloudSize = _lastEdgePointsDS->points.size();
        lastSurfaceCloudSize = _lastSurfPointsDS->points.size();

        if (lastEdgeCloudSize > 10 && lastSurfaceCloudSize > 100) {
            _lastCornerKDTree.setInputCloud(_lastEdgePointsDS);          //  Probably Takes much time
            _lastSurfaceKDTree.setInputCloud(_lastSurfPointsDS);        // Problably Takes much time
        }

        clock_t clockEndCalculationsEnd = clock();
        _numOfCallsOdometryEndCalculations++;
        _totalTimeOdometryEndCalculations += double(clockEndCalculationsEnd - clockEndCalculationsBegin);

        clock_t clockProcessEnd = clock();
        _numOfCallsOdometryProcess++;
        _totalTimeOdometryProcess += double(clockProcessEnd - clockProcessBegin);

    }


} // end namespace loam
