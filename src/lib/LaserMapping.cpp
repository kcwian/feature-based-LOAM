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

#include "loam_velodyne/LaserMapping.h"
#include "loam_velodyne/common.h"

namespace loam
{

LaserMapping::LaserMapping(const float& scanPeriod, const size_t& maxIterations)
{
   // initialize mapping odometry and odometry tf messages
   _odomAftMapped.header.frame_id = "/camera_init";
   _odomAftMapped.child_frame_id = "/aft_mapped";

   _aftMappedTrans.frame_id_ = "/camera_init";
   _aftMappedTrans.child_frame_id_ = "/aft_mapped";
}


bool LaserMapping::setup(ros::NodeHandle& node, ros::NodeHandle& privateNode)
{
   // fetch laser mapping params
   float fParam;
   int iParam;

   if (privateNode.getParam("scanPeriod", fParam))
   {
      if (fParam <= 0)
      {
         ROS_ERROR("Invalid scanPeriod parameter: %f (expected > 0)", fParam);
         return false;
      }
      else
      {
         setScanPeriod(fParam);
         ROS_INFO("Set scanPeriod: %g", fParam);
      }
   }

   if (privateNode.getParam("maxIterations", iParam))
   {
      if (iParam < 1)
      {
         ROS_ERROR("Invalid maxIterations parameter: %d (expected > 0)", iParam);
         return false;
      }
      else
      {
         setMaxIterations(iParam);
         ROS_INFO("Set maxIterations: %d", iParam);
      }
   }

   if (privateNode.getParam("deltaTAbort", fParam))
   {
      if (fParam <= 0)
      {
         ROS_ERROR("Invalid deltaTAbort parameter: %f (expected > 0)", fParam);
         return false;
      }
      else
      {
         setDeltaTAbort(fParam);
         ROS_INFO("Set deltaTAbort: %g", fParam);
      }
   }

   if (privateNode.getParam("deltaRAbort", fParam))
   {
      if (fParam <= 0)
      {
         ROS_ERROR("Invalid deltaRAbort parameter: %f (expected > 0)", fParam);
         return false;
      }
      else
      {
         setDeltaRAbort(fParam);
         ROS_INFO("Set deltaRAbort: %g", fParam);
      }
   }

   if (privateNode.getParam("cornerFilterSize", fParam))
   {
      if (fParam < 0.001)
      {
         ROS_ERROR("Invalid cornerFilterSize parameter: %f (expected >= 0.001)", fParam);
         return false;
      }
      else
      {
         downSizeFilterCorner().setLeafSize(fParam, fParam, fParam);
         ROS_INFO("Set corner down size filter leaf size: %g", fParam);
      }
   }

   if (privateNode.getParam("surfaceFilterSize", fParam))
   {
      if (fParam < 0.001)
      {
         ROS_ERROR("Invalid surfaceFilterSize parameter: %f (expected >= 0.001)", fParam);
         return false;
      }
      else
      {
         downSizeFilterSurf().setLeafSize(fParam, fParam, fParam);
         ROS_INFO("Set surface down size filter leaf size: %g", fParam);
      }
   }

   if (privateNode.getParam("features_minDstPtToPl", fParam))
   {
      if (fParam <= 0)
      {
         ROS_ERROR("Invalid features_minDstPtToPl parameter: %f (expected > 0)", fParam);
         return false;
      }
      else
      {
         mapManager.setMinDistancePointToPlaneFeature(fParam);
         ROS_INFO("Set features_minDstPtToPl %g", fParam);
      }
   }

   if (privateNode.getParam("features_minDstPtToPt", fParam))
   {
      if (fParam <= 0)
      {
         ROS_ERROR("Invalid features_minDstPtToPt parameter: %f (expected > 0)", fParam);
         return false;
      }
      else
      {
         mapManager.setMinDistancePointToClosestPointFromPlaneFeature(fParam);
         ROS_INFO("Set features_minDstPtToPt %g", fParam);
      }
   }

   if (privateNode.getParam("matching_minDstPtToPl", fParam))
   {
      if (fParam <= 0)
      {
         ROS_ERROR("Invalid matching_minDstPtToPl parameter: %f (expected > 0)", fParam);
         return false;
      }
      else
      {
         mapManager.setMatchingMinDstPtToPl(fParam);
         ROS_INFO("Set matching_minDstPtToPl %g", fParam);
      }
   }

    if (privateNode.getParam("matching_minDstPtToPt", fParam))
   {
      if (fParam <= 0)
      {
         ROS_ERROR("Invalid matching_minDstPtToPt parameter: %f (expected > 0)", fParam);
         return false;
      }
      else
      {
         mapManager.setMatchingMinDstPtToPt(fParam);
         ROS_INFO("Set matching_minDstPtToPt %g", fParam);
      }
   }

    if (privateNode.getParam("deleting1_PtNum", iParam))
   {
      if (fParam <= 0)
      {
         ROS_ERROR("Invalid deleting1_PtNum parameter: %f (expected > 0)", iParam);
         return false;
      }
      else
      {
         mapManager.setDeleting1PtNum(iParam);
         ROS_INFO("Set deleting1_PtNum %i", iParam);
      }
   }

    if (privateNode.getParam("deleting2_PtNum", iParam))
   {
      if (fParam <= 0)
      {
         ROS_ERROR("Invalid deleting2_PtNum parameter: %i (expected > 0)", iParam);
         return false;
      }
      else
      {
         mapManager.setDeleting2PtNum(iParam);
         ROS_INFO("Set deleting2_PtNum %i", iParam);
      }
   }

   if (privateNode.getParam("deleting2_iterations", iParam))
   {
      if (fParam <= 0)
      {
         ROS_ERROR("Invalid deleting2_iterations parameter: %i (expected > 0)", iParam);
         return false;
      }
      else
      {
         mapManager.setDeleting2Iterations(iParam);
         ROS_INFO("Set deleting2_iterations %i", iParam);
      }
   }

   



   //   if (privateNode.getParam("mapFilterSize", fParam))
   //   {
   //      if (fParam < 0.001)
   //      {
   //         ROS_ERROR("Invalid mapFilterSize parameter: %f (expected >= 0.001)", fParam);
   //         return false;
   //      }
   //      else
   //      {
   //         downSizeFilterMap().setLeafSize(fParam, fParam, fParam);
   //         ROS_INFO("Set map down size filter leaf size: %g", fParam);
   //      }
   //   }

   // advertise laser mapping topics
   _pubLaserCloudSurround = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 1);
   _pubLaserCloudFullRes  = node.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_registered", 2);
   _pubOdomAftMapped      = node.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 5);

   // subscribe to laser odometry topics
   _subLaserCloudCornerLast = node.subscribe<sensor_msgs::PointCloud2>
      ("/laser_cloud_corner_last", 2, &LaserMapping::laserCloudCornerLastHandler, this);

   _subLaserCloudSurfLast = node.subscribe<sensor_msgs::PointCloud2>
      ("/laser_cloud_surf_last", 2, &LaserMapping::laserCloudSurfLastHandler, this);

   _subLaserOdometry = node.subscribe<nav_msgs::Odometry>
      ("/laser_odom_to_init", 5, &LaserMapping::laserOdometryHandler, this);

   _subLaserCloudFullRes = node.subscribe<sensor_msgs::PointCloud2>
      ("/velodyne_cloud_3", 2, &LaserMapping::laserCloudFullResHandler, this);

   // subscribe to IMU topic
   _subImu = node.subscribe<sensor_msgs::Imu>("/imu/data", 50, &LaserMapping::imuHandler, this);

   if (mapManager.isEnabled()) {

   _pubPlaneFeatures = node.advertise<sensor_msgs::PointCloud2>("/plane_features", 1);

   _pubPlaneFeatures2 = node.advertise<sensor_msgs::PointCloud2>("/plane_features_biggest", 1);

    _pubDebuggingPoints = node.advertise<sensor_msgs::PointCloud2>("/debugging_points", 1);

    _pubPointsUsedForOptimalization = node.advertise<sensor_msgs::PointCloud2>("/pointsForMyOptimalization", 1);

    _pubOryginalOptimalizationPoints = node.advertise<sensor_msgs::PointCloud2>("/pointsForOrygOptimalization", 1);

   }


   return true;
}



void LaserMapping::laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsLastMsg)
{
   _timeLaserCloudCornerLast = cornerPointsLastMsg->header.stamp;
   laserCloudCornerLast().clear();
   pcl::fromROSMsg(*cornerPointsLastMsg, laserCloudCornerLast());
   _newLaserCloudCornerLast = true;
}

void LaserMapping::laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr& surfacePointsLastMsg)
{
   _timeLaserCloudSurfLast = surfacePointsLastMsg->header.stamp;
   laserCloudSurfLast().clear();
   pcl::fromROSMsg(*surfacePointsLastMsg, laserCloudSurfLast());
   _newLaserCloudSurfLast = true;
}

void LaserMapping::laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullResMsg)
{
   _timeLaserCloudFullRes = laserCloudFullResMsg->header.stamp;
   laserCloud().clear();
   pcl::fromROSMsg(*laserCloudFullResMsg, laserCloud());
   _newLaserCloudFullRes = true;
}

void LaserMapping::laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry)
{
   _timeLaserOdometry = laserOdometry->header.stamp;

   geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
   Eigen::Quaterniond q(geoQuat.w, geoQuat.x, geoQuat.y, geoQuat.z);
   Eigen::Vector3d t;
   t(0) = laserOdometry->pose.pose.position.x;
   t(1) = laserOdometry->pose.pose.position.y;
   t(2) = laserOdometry->pose.pose.position.z;

   updateOdometry(t, q);

   _odometryMsg = *laserOdometry; // Saves to variable to write to file

   _newLaserOdometry = true;
}

void LaserMapping::imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn)
{
   double roll, pitch, yaw;
   tf::Quaternion orientation;
   tf::quaternionMsgToTF(imuIn->orientation, orientation);
   tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
   updateIMU({ fromROSTime(imuIn->header.stamp) , roll, pitch });
}

void LaserMapping::spin()
{
   ros::Rate rate(100);
   bool status = ros::ok();
   std::string path = ros::package::getPath("loam_velodyne") + "/validation/aft_mapped_to_init_trajectory.txt";
   std::string path2 = ros::package::getPath("loam_velodyne") + "/validation/laser_odom_to_init_trajectory.txt";
   std::fstream file, file2;

   if (save_trajectory_to_file()){
      file.open (path, std::fstream::out);   // Creates empty file
      file.close();
      file.open (path, std::fstream::app);   // Opens file in append mode

      file2.open (path2, std::fstream::out);   // Creates empty file
      file2.close();
      file2.open (path2, std::fstream::app);   // Opens file in append mode
   }

  static ros::Time odometryMsgLastTime = ros::Time(0);
  static ros::Time aftMappedLastTime = ros::Time(0);

   while (status)
   {
      ros::spinOnce();

      // try processing buffered data
      process();
      status = ros::ok();
      if (save_trajectory_to_file())
      {
         if (aftMappedLastTime != _odomAftMapped.header.stamp)
         {
            saveTrajectoryFile(file, _odomAftMapped);
            aftMappedLastTime = _odomAftMapped.header.stamp;
         }

         if (odometryMsgLastTime != _odometryMsg.header.stamp)
         {
            saveTrajectoryFile(file2, _odometryMsg);
            odometryMsgLastTime = _odometryMsg.header.stamp;
         }
      }
      rate.sleep();
   }
   
   if(save_trajectory_to_file()){
      file.close();
      file2.close();
   }
}

void LaserMapping::saveTrajectoryFile(std::fstream & file, const nav_msgs::Odometry & msg)
{

  file << msg.header.stamp << " " << msg.pose.pose.position.x 
                                  << " " << msg.pose.pose.position.y
                                  << " " << msg.pose.pose.position.z
                                  << " " << msg.pose.pose.orientation.x
                                  << " " << msg.pose.pose.orientation.y 
                                  << " " << msg.pose.pose.orientation.z
                                  << " " << msg.pose.pose.orientation.w
                                  << std::endl;
}

void LaserMapping::reset()
{
   _newLaserCloudCornerLast = false;
   _newLaserCloudSurfLast = false;
   _newLaserCloudFullRes = false;
   _newLaserOdometry = false;
}

bool LaserMapping::hasNewData()
{
   return _newLaserCloudCornerLast && _newLaserCloudSurfLast &&
      _newLaserCloudFullRes && _newLaserOdometry &&
      fabs((_timeLaserCloudCornerLast - _timeLaserOdometry).toSec()) < 0.005 &&
      fabs((_timeLaserCloudSurfLast - _timeLaserOdometry).toSec()) < 0.005 &&
      fabs((_timeLaserCloudFullRes - _timeLaserOdometry).toSec()) < 0.005;
}

void LaserMapping::process()
{
   if (!hasNewData())// waiting for new data to arrive...
      return;

   reset();// reset flags, etc.

   if (!BasicLaserMapping::process(fromROSTime(_timeLaserOdometry)))
      return;

   publishResult();
}

void LaserMapping::publishResult()
{
   // publish new map cloud according to the input output ratio
   if (hasCloudForVisualization()) // publish new map cloud
      publishCloudMsg(_pubLaserCloudSurround, laserCloudSurroundDS(), _timeLaserOdometry, "/camera_init");

   // publish transformed full resolution input cloud
   publishCloudMsg(_pubLaserCloudFullRes, laserCloud(), _timeLaserOdometry, "/camera_init");

   // publish last estimate from mapping with odometru and last odometry
   Eigen::Matrix4d lastMapPose = getLastPose();
   Eigen::Quaterniond lastMapQuat = Eigen::Quaterniond(lastMapPose.block<3, 3>(0, 0));

   Eigen::Matrix4d lastOdoPose = getLastOdometryPose();

   _odomAftMapped.header.stamp = _timeLaserOdometry;
   _odomAftMapped.pose.pose.orientation.x = lastMapQuat.x();
   _odomAftMapped.pose.pose.orientation.y = lastMapQuat.y();
   _odomAftMapped.pose.pose.orientation.z = lastMapQuat.z();
   _odomAftMapped.pose.pose.orientation.w = lastMapQuat.w();
   _odomAftMapped.pose.pose.position.x = lastMapPose(0,3);
   _odomAftMapped.pose.pose.position.y = lastMapPose(1,3);
   _odomAftMapped.pose.pose.position.z = lastMapPose(2,3);

   // TODO: Necessary?
   Eigen::Quaterniond lastOdoQuat(lastOdoPose.block<3, 3>(0, 0));
   lastOdoQuat.normalize();
   Eigen::Vector3d pry = lastOdoQuat.toRotationMatrix().eulerAngles(1, 0, 2);

   _odomAftMapped.twist.twist.angular.x = pry(1);
   _odomAftMapped.twist.twist.angular.y = pry(0);
   _odomAftMapped.twist.twist.angular.z = pry(2);
   _odomAftMapped.twist.twist.linear.x = lastOdoPose(0,3);
   _odomAftMapped.twist.twist.linear.y = lastOdoPose(1,3);
   _odomAftMapped.twist.twist.linear.z = lastOdoPose(2,3);
   _pubOdomAftMapped.publish(_odomAftMapped);

   _aftMappedTrans.stamp_ = _timeLaserOdometry;
   _aftMappedTrans.setRotation(tf::Quaternion(lastMapQuat.x(), lastMapQuat.y(), lastMapQuat.z(), lastMapQuat.w()));
   _aftMappedTrans.setOrigin(tf::Vector3(lastMapPose(0,3),
                                         lastMapPose(1,3),
                                         lastMapPose(2,3)));
   _tfBroadcaster.sendTransform(_aftMappedTrans);

   // Added - plane features cloud
   if (mapManager.isEnabled()){
   mapManager.putAllFeaturesIntoCloud();
   publishCloudMsg(_pubPlaneFeatures,  mapManager.planeFeaturesCloud(), _timeLaserOdometry, "/camera_init");
   mapManager.putBiggestFeaturesIntoCloud(30); // Gets only biggest Features into Cloud2, Arg: num of Features 
   publishCloudMsg(_pubPlaneFeatures2,  mapManager.planeFeaturesCloud2(), _timeLaserOdometry, "/camera_init");
   // Debugging points
   publishCloudMsg(_pubDebuggingPoints,  mapManager.debuggingPoints(), _timeLaserOdometry, "/camera_init");
   // My optimalization ppoints
   publishCloudMsg(_pubPointsUsedForOptimalization,  mapManager.pointsUsedForOptimalization(), _timeLaserOdometry, "/camera_init");
    // Oryginal Optim points
    publishCloudMsg(_pubOryginalOptimalizationPoints,  mapManager.oryginalOptimalizationPoints(), _timeLaserOdometry, "/camera_init");
   }

}

} // end namespace loam
