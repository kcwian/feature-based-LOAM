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

#include "loam_velodyne/MultiScanRegistration.h"
#include "../../include/loam_velodyne/math_utils.h"

#include <pcl_conversions/pcl_conversions.h>


namespace loam {

    MultiScanMapper::MultiScanMapper(const float &lowerBound,
                                     const float &upperBound,
                                     const uint16_t &nScanRings,
                                     const ScannerType scannerType)
            : _lowerBound(lowerBound),
              _upperBound(upperBound),
              _nScanRings(nScanRings),
              _factor((nScanRings - 1) / (upperBound - lowerBound)),
              _scannerType(scannerType) {

    }

    void MultiScanMapper::set(const float &lowerBound,
                              const float &upperBound,
                              const uint16_t &nScanRings) {
        _lowerBound = lowerBound;
        _upperBound = upperBound;
        _nScanRings = nScanRings;
        _factor = (nScanRings - 1) / (upperBound - lowerBound);
    }


    int MultiScanMapper::getRingForAngle(const float &angle) {
        return int(((angle * 180 / M_PI) - _lowerBound) * _factor + 0.5);
    }


    MultiScanRegistration::MultiScanRegistration(const MultiScanMapper &scanMapper)
            : _scanMapper(scanMapper) {};


    bool MultiScanRegistration::setup(ros::NodeHandle &node, ros::NodeHandle &privateNode) {
        RegistrationParams config;
        if (!setupROS(node, privateNode, config))
            return false;

        configure(config);
        return true;
    }

    bool MultiScanRegistration::setupROS(ros::NodeHandle &node, ros::NodeHandle &privateNode,
                                         RegistrationParams &config_out) {
        if (!ScanRegistration::setupROS(node, privateNode, config_out))
            return false;

        // fetch scan mapping params
        std::string lidarName;

        if (privateNode.getParam("lidar", lidarName)) {
            if (lidarName == "VLP-16") {
                _scanMapper = MultiScanMapper::Velodyne_VLP_16();
            } else if (lidarName == "HDL-32") {
                _scanMapper = MultiScanMapper::Velodyne_HDL_32();
            } else if (lidarName == "HDL-64E") {
                _scanMapper = MultiScanMapper::Velodyne_HDL_64E();
            } else if (lidarName == "MRS-6000") {
                _scanMapper = MultiScanMapper::SICK_MRS_6000();
            } else {
                ROS_ERROR("Invalid lidar parameter: %s (only \"VLP-16\", \"HDL-32\" and \"HDL-64E\" are supported)",
                          lidarName.c_str());
                return false;
            }

            ROS_INFO("Set  %s  scan mapper.", lidarName.c_str());
            if (!privateNode.hasParam("scanPeriod")) {
                config_out.scanPeriod = 0.1;
                ROS_INFO("Set scanPeriod: %f", config_out.scanPeriod);
            }
        } else {
            float vAngleMin, vAngleMax;
            int nScanRings;

            if (privateNode.getParam("minVerticalAngle", vAngleMin) &&
                privateNode.getParam("maxVerticalAngle", vAngleMax) &&
                privateNode.getParam("nScanRings", nScanRings)) {
                if (vAngleMin >= vAngleMax) {
                    ROS_ERROR("Invalid vertical range (min >= max)");
                    return false;
                } else if (nScanRings < 2) {
                    ROS_ERROR("Invalid number of scan rings (n < 2)");
                    return false;
                }

                _scanMapper.set(vAngleMin, vAngleMax, nScanRings);
                ROS_INFO("Set linear scan mapper from %g to %g degrees with %d scan rings.", vAngleMin, vAngleMax,
                         nScanRings);
            }
        }

        // subscribe to input cloud topic
        _subLaserCloud = node.subscribe<sensor_msgs::PointCloud2>
                ("/multi_scan_points", 2, &MultiScanRegistration::handleCloudMessage, this);

        return true;
    }


    void MultiScanRegistration::handleCloudMessage(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
        if (_systemDelay > 0) {
            --_systemDelay;
            return;
        }

        // fetch new input cloud
        pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
        pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);

        process(laserCloudIn, fromROSTime(laserCloudMsg->header.stamp));
    }


    void MultiScanRegistration::process(const pcl::PointCloud<pcl::PointXYZ> &laserCloudIn, const Time &scanTime) {
        static const double incorrectMeasurementDistance = 10000;
        size_t cloudSize = laserCloudIn.size();

        pcl::PointCloud<pcl::PointXYZ> laserCloudInFiltered; // Modified Point Cloud

        if (enableModifications)
        {
            // Creating depth image from MRS-6000 data
            if (_scanMapper.getScannerType() == MultiScanMapper::ScannerType::MRS_6000)
            {
                if (!_laserDepthImage.empty())
                    _laserDepthImage.release();
                _laserDepthImage.create(24, 924, CV_32FC1);
                _laserDepthImage = 0;

                // Creates Depth Image and marks invalid points
                for (int i = 0; i < cloudSize ; i++)
                {
                    int ind1 = i / 924;
                    int ind2 = i % 924;
                    if (!pcl_isfinite(laserCloudIn[i].x) || !pcl_isfinite(laserCloudIn[i].y) || !pcl_isfinite(laserCloudIn[i].z)) // Necessary ?
                    {
                        _laserDepthImage.at<float>(ind1, ind2) = 0; // Value of Invalid Points
                        continue;
                    }
                    float val = sqrt(laserCloudIn[i].z * laserCloudIn[i].z + laserCloudIn[i].y * laserCloudIn[i].y + laserCloudIn[i].x * laserCloudIn[i].x);
                    if (val > 0.0001)
                    {
                        _laserDepthImage.at<float>(ind1, ind2) = val;
                    }
                    else
                    {
                        _laserDepthImage.at<float>(ind1, ind2) = 0; // Value of Invalid Points
                    }
                }
                // Normalizes image for visual effect but changes distances
                cv::normalize(_laserDepthImage,_laserDepthImage, 0, 255, cv::NORM_MINMAX, CV_32FC1);
                 // Converts image to RGB for visual effect
                cv::cvtColor(_laserDepthImage, _laserDepthImage, cv::COLOR_GRAY2BGR);

                // Marks all invalid points and their n-size neighbours
                int n = 1; // Kernel window size - must be odd number
                cv::Mat outputImage = _laserDepthImage.clone();
                for (int i = 0; i < _laserDepthImage.rows-1; i++)
                {
                    for (int j = 0; j <_laserDepthImage.cols-1; j++)
                    {
                        if (_laserDepthImage.at<cv::Vec3f>(i, j)[0] == 0)
                        {
                            int dx = (n - 1) / 2;
                            for (int k = i - dx; k <= i + dx; k++)
                                for (int l = j - dx; l <= j + dx; l++)
                                    if (k >= 0 && k < _laserDepthImage.rows && l >= 0 && l < _laserDepthImage.cols)
                                    {
                                       outputImage.at<cv::Vec3f>(k, l)[0] = 255; // Blue Points
                                       outputImage.at<cv::Vec3f>(k, l)[1] = 0;
                                       outputImage.at<cv::Vec3f>(k, l)[2] = 0;                                   
                                    }
                        }
                    }
                }

                // Marks all points and neigbours that differ more than maxDiffDist meters
                int m = 1; // Nieparzysta
                float maxDistDiff = 10; // Image normalization should be disabled
                for (int i = 0; i < _laserDepthImage.rows-1; i++)
                {
                    for (int j = 0; j <_laserDepthImage.cols-1; j++)
                    {
                        if (_laserDepthImage.at<cv::Vec3f>(i, j)[0] > 0) // Skips invalid points as they were marked in loop before
                        {
                            int dx = (m - 1) / 2;
                            bool found = false;
                            for (int k = i - dx; k <= i + dx; k++){
                                for (int l = j - dx; l <= j + dx; l++)
                                {
                                    if (k >= 0 && k < _laserDepthImage.rows && l >= 0 && l < _laserDepthImage.cols)
                                    {
                                        float dist = fabs(_laserDepthImage.at<cv::Vec3f>(i, j)[0] - _laserDepthImage.at<cv::Vec3f>(k, l)[0]); 
                                        if (dist > maxDistDiff)
                                        {
                                            for (int k2 = i - dx; k2 <= i + dx; k2++)
                                                for (int l2 = j - dx; l2 <= j + dx; l2++)
                                                {
                                                    //  outputImage.at<cv::Vec3f>(k, l)[0] = 0; 
                                                    //  outputImage.at<cv::Vec3f>(k, l)[1] = 255; // Green Points
                                                    //  outputImage.at<cv::Vec3f>(k, l)[0] = 0; 
                                                }
                                                found = true;
                                                break;
                                        }                                                                     
                                    }
                                }
                                if (found)
                                    break;
                            }
                        }
                    }
                }
                // Recreates points cloud from map image - removes marked points
                for (int i = 0; i < cloudSize ; i++)
                {
                    int ind1 = i / 924;
                    int ind2 = i % 924;
                    if (outputImage.at<cv::Vec3f>(ind1, ind2)[0] != 255 && outputImage.at<cv::Vec3f>(ind1, ind2)[1] != 255 && outputImage.at<cv::Vec3f>(ind1, ind2)[2] != 255)
                        laserCloudInFiltered.push_back(laserCloudIn[i]);
                }

                _laserDepthImage = outputImage;
                _laserDepthImage.convertTo(_laserDepthImage, CV_8UC3);
                cv::resize(_laserDepthImage, _laserDepthImage, cv::Size(), 2, 8, cv::INTER_LINEAR);

                // Save image to file
               // static int nFile = 1;
                //std::string path = ros::package::getPath("loam_velodyne") + "/scans_img/img" + std::to_string(nFile) + ".png";
               // cv::imwrite(path, _laserDepthImage);
            }

            else if (_scanMapper.getScannerType() == MultiScanMapper::ScannerType::VLP_16)
            {
                // Not implemented yet
                laserCloudInFiltered = laserCloudIn;

                // Depth Image is created at the end of this function as it requires relTime calculated
            }
        }

        // determine scan start and end orientations - stays the same when using laserCloudFiltered ?
        float startOri = -std::atan2(laserCloudIn[0].y, laserCloudIn[0].x);
        float endOri = -std::atan2(laserCloudIn[cloudSize - 1].y,
                                   laserCloudIn[cloudSize - 1].x) + 2 * float(M_PI);
        if (endOri - startOri > 3 * M_PI) {
            endOri -= 2 * M_PI;
        } else if (endOri - startOri < M_PI) {
            endOri += 2 * M_PI;
        }

        if (enableModifications){
            cloudSize = laserCloudInFiltered.size();
        }
        else{
            cloudSize = laserCloudIn.size();
        }

        bool halfPassed = false;
        pcl::PointXYZI point;
        _laserCloudScans.resize(_scanMapper.getNumberOfScanRings());
        // clear all scanline points
        std::for_each(_laserCloudScans.begin(), _laserCloudScans.end(), [](auto &&v) { v.clear(); });

        // extract valid points from input cloud
        for (int i = 0; i < cloudSize; i++) {
            if (enableModifications){
                point.x = laserCloudInFiltered[i].y;
                point.y = laserCloudInFiltered[i].z;
                point.z = laserCloudInFiltered[i].x;
            }
            else {
                point.x = laserCloudIn[i].y;
                point.y = laserCloudIn[i].z;
                point.z = laserCloudIn[i].x;
            }

            // skip NaN and INF valued points
            if (!pcl_isfinite(point.x) ||
                !pcl_isfinite(point.y) ||
                !pcl_isfinite(point.z)) {
                continue;
            }

            // skip zero valued points && Velodyne points with artificial distance
            double distSquared = point.x * point.x + point.y * point.y + point.z * point.z;
            if (distSquared < 0.0001 ||
                fabs(distSquared - incorrectMeasurementDistance * incorrectMeasurementDistance) < 100) {
                continue;
            }

            // calculate vertical point angle and scan ID when it is not MRS_6000
            // then compute the relativeTime
            int scanID = 0;
            float relTime = 0;
            if (_scanMapper.getScannerType() != MultiScanMapper::ScannerType::MRS_6000) {
                float angle = std::atan(point.y / std::sqrt(point.x * point.x + point.z * point.z));
                scanID = _scanMapper.getRingForAngle(angle);
                if (scanID >= _scanMapper.getNumberOfScanRings() || scanID < 0) {
                    continue;
                }

                // calculate horizontal point angle
                float ori = -std::atan2(point.x, point.z);
                if (!halfPassed) {
                    if (ori < startOri - M_PI / 2) {
                        ori += 2 * M_PI;
                    } else if (ori > startOri + M_PI * 3 / 2) {
                        ori -= 2 * M_PI;
                    }

                    if (ori - startOri > M_PI) {
                        halfPassed = true;
                    }
                } else {
                    ori += 2 * M_PI;

                    if (ori < endOri - M_PI * 3 / 2) {
                        ori += 2 * M_PI;
                    } else if (ori > endOri + M_PI / 2) {
                        ori -= 2 * M_PI;
                    }
                }

                // calculate relative scan time based on point orientation
                relTime = config().scanPeriod * (ori - startOri) / (endOri - startOri);
            }
            // In case of MRS_6000 each line contains 924 points and we can use it to find scanID
            else {
                scanID = i / 924;

                // MRS 6300 (CCW) rotates in the other direction than Velodyne (CW) :)
                startOri = 1.0472;
                endOri = -1.0472;

                // Time for one group * (group ID + shift in the group)
                // MRS6300 captures 4 groups from the top, scanIDs (0 to 23) are numbered from the bottom.
                // Taking a scan takes scanPeriod, one group takes scanPeriod / 4
                // shiftGroup -> 0 for top group, 1/4*scanPeriod for 2nd group, 2/4*scanPeriod for 3rd group, 3/4*scan Period for 4th group
                // shiftInGroup -> depends on the horizontal angle, MRS6300 scans CCW, startOri=60deg, endOri=-60deg, relativeAngleChange * 1/4*scanPeriod
                float ori = -std::atan2(point.x, point.z);
                int shiftGroup = ceil((23 - scanID) / 6);
                float shiftInGroup = (startOri - ori) / (2 * M_PI);
                relTime = config().scanPeriod / 4.0 * (shiftGroup + shiftInGroup);
            }

            point.intensity = scanID + relTime;
            projectPointToStartOfSweep(point, relTime);
            if (scanID < 24)                               // MRS gives 24 * 4 lines, but only first 24 are valid (Multiecho)
                _laserCloudScans[scanID].push_back(point); //// ToDo: Wysypuje się czasami
        }

        if (enableModifications)
        {
            // Velodyne Depth Maph - requires relTime calculated    
            if (_scanMapper.getScannerType() == MultiScanMapper::ScannerType::VLP_16)
            {
                if (!_laserDepthImage.empty())
                    _laserDepthImage.release();

                _laserDepthImage.create(16, 1824, CV_32FC1);
                _laserDepthImage = 0;

                std::vector<std::pair<int, float>> vector_index_relativeTime;

                //  Creating depth image in order of points aquisition
                //  for (int i = 0; i < cloudSize; i++)
                //  {
                //     int ind1 = i / 1824;
                //     int ind2 = i % 1824;
                //     float val = sqrt(laserCloudIn[i].z * laserCloudIn[i].z + laserCloudIn[i].y * laserCloudIn[i].y + laserCloudIn[i].x * laserCloudIn[i].x);
                //     if (val > 0)
                //         _laserDepthImage.at<float>(ind1, ind2) = val;
                //  }

                std::vector<pcl::PointXYZI> pointsToSort;
                for (int i = 0; i < _laserCloudScans.size(); i++)
                {
                    // Rewrites and sorts points with same ScanID based on relative scan time
                    pointsToSort.clear();
                    for (int j = 0; j < _laserCloudScans[i].size(); j++)
                    {
                        pointsToSort.push_back(_laserCloudScans[i][j]);
                    }
                    std::sort(pointsToSort.begin(), pointsToSort.end(), [](pcl::PointXYZI &point1, pcl::PointXYZI &point2) -> bool { return point1.intensity < point2.intensity; });

                    // Writes sorted points to Depth Image
                    for (int j = 0; j < pointsToSort.size(); j++)
                        _laserDepthImage.at<float>(i, j) = sqrt(pointsToSort.at(j).z * pointsToSort.at(j).z + pointsToSort.at(j).y * pointsToSort.at(j).y + pointsToSort.at(j).x * pointsToSort.at(j).x);
                }

                cv::normalize(_laserDepthImage,_laserDepthImage, 0, 255, cv::NORM_MINMAX, CV_32FC1);
                 // Converts image to RGB for visual effect
                cv::cvtColor(_laserDepthImage, _laserDepthImage, cv::COLOR_GRAY2BGR);
                cv::resize(_laserDepthImage, _laserDepthImage, cv::Size(), 2, 8, cv::INTER_LINEAR);
                _laserDepthImage.convertTo(_laserDepthImage, CV_8UC3);

            }
        }

        processScanlines(scanTime, _laserCloudScans);
        publishResult();
    }

} // end namespace loam
