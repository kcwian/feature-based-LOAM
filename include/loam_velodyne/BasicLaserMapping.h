#pragma once
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


#include "Twist.h"
#include "CircularBuffer.h"
#include "time_utils.h"
#include "FeatureUtils.h"
#include "PoseUtils.h"
#include "MapManager.h"
#include "TimeMeasurement.h"

#include <Eigen/Eigen>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include "Optimization.h"

namespace loam {

    /** IMU state data. */
    typedef struct IMUState2 {
        /** The time of the measurement leading to this state (in seconds). */
        Time stamp;

        /** The current roll angle. */
        Angle roll;

        /** The current pitch angle. */
        Angle pitch;

        /** \brief Interpolate between two IMU states.
         *
         * @param start the first IMU state
         * @param end the second IMU state
         * @param ratio the interpolation ratio
         * @param result the target IMU state for storing the interpolation result
         */
        static void interpolate(const IMUState2 &start,
                                const IMUState2 &end,
                                const float &ratio,
                                IMUState2 &result) {
            float invRatio = 1 - ratio;

            result.roll = start.roll.rad() * invRatio + end.roll.rad() * ratio;
            result.pitch = start.pitch.rad() * invRatio + end.pitch.rad() * ratio;
        };
    } IMUState2;

    class BasicLaserMapping {
    public:
        // scanPeriod - time interval between scans in seconds
        // maxIterations - maximum number of iterations in mapping
        explicit BasicLaserMapping(const float &scanPeriod = 0.1, const size_t &maxIterations = 10);

        // Method used to process incoming data
        bool process(Time const &laserOdometryTime);

        // Updating IMU measurements
        void updateIMU(IMUState2 const &newState);

        // Odometry
        void updateOdometry(const Eigen::Vector3d &t, const Eigen::Quaterniond &q);

        // Used to set the last received clouds
        auto &laserCloud() { return *_laserCloudFullRes; }
        auto &laserCloudCornerLast() { return *_laserCloudEdgeLast; }
        auto &laserCloudSurfLast() { return *_laserCloudSurfLast; }

        // Parameters
        void setScanPeriod(float val) { _scanPeriod = val; }
        void setMaxIterations(size_t val) { _maxIterations = val; }
        void setDeltaTAbort(float val) { _deltaTAbort = val; }
        void setDeltaRAbort(float val) { _deltaRAbort = val; }

        // Getting downsizeFilters
        auto &downSizeFilterCorner() { return _downSizeFilterEdge; }
        auto &downSizeFilterSurf() { return _downSizeFilterSurf; }

        // Getting parameters
//        auto scanPeriod() const { return _scanPeriod; }
//        auto maxIterations() const { return _maxIterations; }
//        auto deltaTAbort() const { return _deltaTAbort; }
//        auto deltaRAbort() const { return _deltaRAbort; }

        // Getting transformations
        Eigen::Matrix4d getLastPose() { return lastMapPose; }
        Eigen::Matrix4d getLastOdometryPose() { return lastOdometryPose; }

        // Getting cloud for visualization
        auto const &laserCloudSurroundDS() const { return *_cloudForVisualization; }

        // Checks if new map for visualization was prepared
        bool hasCloudForVisualization() const { return _createdCloudForVisualization; }

        // Saving trajectory for further evaluation
        bool save_trajectory_to_file() const { return _save_trajectory_to_file; }

        // Object storing the time measurements
        TimeMeasurement timeMeasurement;

        // Introduced mapManager for created features
        MapManager mapManager;

    private:
        // Run an optimization
        bool poseOptimization(pcl::PointCloud<pcl::PointXYZI>::Ptr mapEdges, pcl::PointCloud<pcl::PointXYZI>::Ptr mapSurfs);

        // Sets the current pose as LastMapping + OdometryIncrement
        void updateCurrentPose();

        // Updates transformations after optimization
        void transformUpdate();

        // Changes the coordinates systems
        void moveFromLocalToGlobal(const pcl::PointXYZI &pi, pcl::PointXYZI &po, const Eigen::Matrix4d &pose);

        // Moves the local map to the global LC
        void transformFullResToMap();

        // Creates a downsamples map of the surrounding environment for visualization. Every 5th time of mapping
        bool createCloudForVisualization(const std::vector<size_t> &laserCloudSurroundInd);

        // Moving cubes based on current location
        void moveCubes();

        // Checking points from cubes in fov
        void checkFov(std::vector <size_t> &laserCloudValidInd, std::vector <size_t> &laserCloudSurroundInd);

        // Method called to add edges and surf to the cubes
        void addEdgesAndSurfsToCubes();

        // Getting the index of the cube
        size_t toIndex(int i, int j, int k) const {
            return i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k;
        }

    private:
        // Timestamp
        Time _laserOdometryTime;

        // Time between scans
        float _scanPeriod;

        // Optimization parameters: maximum number of iterations & optimization abort thresholds
        size_t _maxIterations;
        float _deltaTAbort, _deltaRAbort;

        // Parameters for cubes
        const double CUBE_SIZE = 50.0, CUBE_HALF = 25.0;
        int _laserCloudCenWidth, _laserCloudCenHeight,_laserCloudCenDepth;
        const size_t _laserCloudWidth, _laserCloudHeight, _laserCloudDepth, _laserCloudNum;

        // Clouds received from ROS: edges, surfs and full cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudEdgeLast, _laserCloudSurfLast, _laserCloudFullRes;

        // Downsampled clouds with current edges and surfs
        pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudEdgeLastDS, _laserCloudSurfLastDS;

        // Downsampled cloud for visualization
        pcl::PointCloud<pcl::PointXYZI>::Ptr _cloudForVisualization;

        // Cubes of clouds used as a ``map'''
        std::vector <pcl::PointCloud<pcl::PointXYZI>::Ptr> _laserCloudEdgeArray, _laserCloudSurfArray;

        // Keeping last version and doing new below
        Eigen::Matrix4d curOdometryPose, curMapWithOdo, lastOdometryPose, lastMapPose;

        // history of IMU states
        CircularBuffer<IMUState2> _imuHistory;

        // VoxelGrid for edges and surfaces
        pcl::VoxelGrid <pcl::PointXYZI> _downSizeFilterEdge, _downSizeFilterSurf;

        // Is map already intialized?
        bool mapInitialized = false;

        // Variables used to determine when update the cloud for visualization
        const long _mapFrameNum;
        long _mapFrameCount;
        bool _createdCloudForVisualization = false;

        // Should we save trajectory to the file
        bool _save_trajectory_to_file;

        // Stream to save eigenvalues
        long currentFrame;
        std::ofstream eigenvalueStream;
    };

} // end namespace loam





