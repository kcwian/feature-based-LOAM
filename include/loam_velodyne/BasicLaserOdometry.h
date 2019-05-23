#pragma once

#include "Twist.h"
#include "nanoflann_pcl.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "loam_velodyne/LieAlgebra.h"
#include "loam_velodyne/math_utils.h"

#include <fstream>

namespace loam {

    /** \brief Implementation of the LOAM laser odometry component.
     *
     */
    class BasicLaserOdometry {
    public:
        /**
         * Constructor settings the parameters
         */
        explicit BasicLaserOdometry(float scanPeriod = 0.1, size_t maxIterations = 25);

        /**
         * Main processing loop
         */
        void process();

        /**
         *
         * @param imuTrans
         */
//        void updateIMU(pcl::PointCloud<pcl::PointXYZ> const &imuTrans);

        /**
         * Getting the current clouds
         * @return
         */
        auto &curEdgePoints() { return _curEdgePoints; }
        auto &curEdgePointsDS() { return _curEdgePointsDS; }
        auto &curSurfPoints() { return _curSurfPoints; }
        auto &curSurfPointsDS() { return _curSurfPointsDS; }

        /**
         * Current full clouds
         * @return
         */
        auto &laserCloud() { return _laserCloud; }

        /**
         * Getting the last clouds
         * @return
         */
        auto const &lastCornerCloud() { return _lastEdgePointsDS; }
        auto const &lastSurfaceCloud() { return _lastSurfPointsDS; }

        /**
         * Getting the currently accumulated laser odometry
         */
        auto const &laserOdomInInit() { return accPoseInGlobal; }

        /**
         * Settings the parameters
         * @param val
         */
        void setScanPeriod(float val) { _scanPeriod = val; }
        void setMaxIterations(size_t val) { _maxIterations = val; }
        void setDeltaTAbort(float val) { _deltaTAbort = val; }
        void setDeltaRAbort(float val) { _deltaRAbort = val; }

        /**
         * Getting the number of processed frames
         * @return
         */
        auto frameCount() const { return _frameCount; }

        /**
         * Getting the set parameters
         * @return
         */
        auto scanPeriod() const { return _scanPeriod; }
        auto maxIterations() const { return _maxIterations; }
        auto deltaTAbort() const { return _deltaTAbort; }
        auto deltaRAbort() const { return _deltaRAbort; }

        /**
         * Time considerations
         * @return
         */
        size_t numOfCallsOdometryKdSearchCorners() { return _numOfCallsOdometryKdSearchCorners; }
        double totalTimeOdometryKdSearchCorners() { return _totalTimeOdometryKdSearchCorners; }

        size_t numOfCallsOdometryKdSearchSurfaces() { return _numOfCallsOdometryKdSearchSurfaces; }
        double totalTimeOdometryKdSearchSurfaces() { return _totalTimeOdometryKdSearchSurfaces; }

        size_t numOfCallsOdometryProcess() { return _numOfCallsOdometryProcess; }
        double totalTimeOdometryProcess() { return _totalTimeOdometryProcess; }

        size_t numOfCallsOdometryConditionsCorners() { return _numOfCallsOdometryConditionsCorners; }
        double totalTimeOdometryConditionsCorners() { return _totalTimeOdometryConditionsCorners; }

        size_t numOfCallsOdometryConditionsSurfaces() { return _numOfCallsOdometryConditionsSurfaces; }
        double totalTimeOdometryConditionsSurfaces() { return _totalTimeOdometryConditionsSurfaces; }

        size_t numOfCallsOdometryOptimalization() { return _numOfCallsOdometryOptimalization; }
        double totalTimeOdometryOptimalization() { return _totalTimeOdometryOptimalization; }

        size_t numOfCallsOdometryEndCalculations() { return _numOfCallsOdometryEndCalculations; }
        double totalTimeOdometryEndCalculations() { return _totalTimeOdometryEndCalculations; }

        /**
         * Transform the given point cloud to the end of the sweep
         * @param cloud the point cloud to transform
         */
        size_t transformToEnd(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);

        // Current position w.r.t. last pose saved as inverse
        Eigen::Matrix4d invPoseInPrevScan;

        // Global position coming from odometry integration
        Eigen::Matrix4d poseIntInGlobal;

        // Stream to save eigenvalues
        std::ofstream eigenvalueStream;

    private:
        /** \brief Transform the given point to the start of the sweep.
         *
         * @param pi the point to transform
         * @param po the point instance for storing the result
         */
        void transformToStart(const pcl::PointXYZI &pi, pcl::PointXYZI &po, const Eigen::Vector6d &invPoseLog);
        void transformToStart(const pcl::PointXYZI &pi, pcl::PointXYZI &po, const Eigen::Matrix4d &invPose);


//        void pluginIMURotation(const Angle &bcx, const Angle &bcy, const Angle &bcz,
//                               const Angle &blx, const Angle &bly, const Angle &blz,
//                               const Angle &alx, const Angle &aly, const Angle &alz,
//                               Angle &acx, Angle &acy, Angle &acz);

//        void accumulateRotation(Angle cx, Angle cy, Angle cz,
//                                Angle lx, Angle ly, Angle lz,
//                                Angle &ox, Angle &oy, Angle &oz);

    private:
        float _scanPeriod;       // time per scan
        long _frameCount;        // number of processed frames
        size_t _maxIterations;   // maximum number of iterations
        bool _systemInited;      // initialization flag

        // optimization abort thresholds
        float _deltaTAbort, _deltaRAbort;

        // Last edge & surface points cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr _lastEdgePointsDS, _lastSurfPointsDS;

        // Current edge & surface clouds, DS means downsampled
        pcl::PointCloud<pcl::PointXYZI>::Ptr _curEdgePoints, _curEdgePointsDS;
        pcl::PointCloud<pcl::PointXYZI>::Ptr _curSurfPoints, _curSurfPointsDS;

        // Full resolution cloud - just stored in BasicLaserOdometry as nothing is changed with it
        pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloud;

        // kd-trees
        nanoflann::KdTreeFLANN<pcl::PointXYZI> _lastCornerKDTree, _lastSurfaceKDTree;

        // Inverse Pose w.r.t the last pose & accumulated pose from laser odometry
        Eigen::Matrix4d invPoseInPrev, accPoseInGlobal;

//        Angle _imuRollStart, _imuPitchStart, _imuYawStart;
//        Angle _imuRollEnd, _imuPitchEnd, _imuYawEnd;
//
//        Vector3 _imuShiftFromStart;
//        Vector3 _imuVeloFromStart;

        size_t _numOfCallsOdometryKdSearchCorners; ///< Number of calls of KdSearch Corners
        double _totalTimeOdometryKdSearchCorners; ///< Total Time spent on KdSearch Corners

        size_t _numOfCallsOdometryKdSearchSurfaces; ///< Number of calls of KdSearch Surfaces
        double _totalTimeOdometryKdSearchSurfaces; ///< Total Time spent on KdSearch Surfaces

        size_t _numOfCallsOdometryProcess; ///< Number of calls of process()
        double _totalTimeOdometryProcess; ///< Total Time spent on process()

        size_t _numOfCallsOdometryConditionsCorners; ///< Number of calls of conditions calculations
        double _totalTimeOdometryConditionsCorners; ///< Total Time spent on conditions calculations

        size_t _numOfCallsOdometryConditionsSurfaces; ///< Number of calls of surfaces conditions calculations
        double _totalTimeOdometryConditionsSurfaces; ///< Total Time spent on surfaces conditions calculations

        size_t _numOfCallsOdometryOptimalization; ///< Number of calls of optimalization
        double _totalTimeOdometryOptimalization; ///< Total Time spent on optimalization

        size_t _numOfCallsOdometryEndCalculations; ///< Number of calls of end calculations
        double _totalTimeOdometryEndCalculations; ///< Total Time spent on end calculations
    };

} // end namespace loam
