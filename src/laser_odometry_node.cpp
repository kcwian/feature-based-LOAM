#include <ros/ros.h>
#include "loam_velodyne/LaserOdometry.h"


/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "laserOdometry");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  loam::LaserOdometry laserOdom(0.1);

  if (laserOdom.setup(node, privateNode)) {
    // initialization successful
    laserOdom.spin();
  }

  
  std::cout << std::endl << std::endl << "<------------------Odometry Execution Time---------------->" << std::endl;
 
  std::cout << "Odometry Process Time:       " << double( laserOdom.totalTimeOdometryProcess() / CLOCKS_PER_SEC ) << "  Avg: " << double(laserOdom.totalTimeOdometryProcess() / CLOCKS_PER_SEC / 
                                                              laserOdom.numOfCallsOdometryProcess()) << std::endl; 

  std::cout << "  Corner For Loop Time:        " <<   double(laserOdom.totalTimeOdometryConditionsCorners() / CLOCKS_PER_SEC) << " Avg: " << double(laserOdom.totalTimeOdometryConditionsCorners() / CLOCKS_PER_SEC / 
                                                              laserOdom.numOfCallsOdometryConditionsCorners()) << std::endl;
                                                                                                                      
  std::cout << "    KdSearch Corner Time:        " << double( laserOdom.totalTimeOdometryKdSearchCorners() / CLOCKS_PER_SEC ) << " Avg: " << double(laserOdom.totalTimeOdometryKdSearchCorners() / CLOCKS_PER_SEC / 
                                                              laserOdom.numOfCallsOdometryKdSearchCorners()) << std::endl;

  std::cout << "  Surface For Loop Time:       " <<   double(laserOdom.totalTimeOdometryConditionsSurfaces() / CLOCKS_PER_SEC) << " Avg: " << double(laserOdom.totalTimeOdometryConditionsSurfaces() / CLOCKS_PER_SEC / 
                                                              laserOdom.numOfCallsOdometryConditionsSurfaces()) << std::endl;    
                                                              
  std::cout << "    KdSearch Surfaces Time:      " << double( laserOdom.totalTimeOdometryKdSearchSurfaces() / CLOCKS_PER_SEC ) << " Avg: " << double(laserOdom.totalTimeOdometryKdSearchSurfaces() / CLOCKS_PER_SEC / 
                                                              laserOdom.numOfCallsOdometryKdSearchSurfaces()) << std::endl;                                                                                                                                                                                                                                                                                            

  std::cout << "  Optimization Time:           " <<   double(laserOdom.totalTimeOdometryOptimalization() / CLOCKS_PER_SEC) << " Avg: " << double(laserOdom.totalTimeOdometryOptimalization() / CLOCKS_PER_SEC / 
                                                              laserOdom.numOfCallsOdometryOptimalization()) << std::endl;  
                                                               
  std::cout << "  End Calculations Time:       " <<   double(laserOdom.totalTimeOdometryEndCalculations() / CLOCKS_PER_SEC) << " Avg: " << double(laserOdom.totalTimeOdometryEndCalculations() / CLOCKS_PER_SEC / 
                                                              laserOdom.numOfCallsOdometryEndCalculations()) << std::endl;    

  std::cout << std::endl << std::endl;

  return 0;
}
