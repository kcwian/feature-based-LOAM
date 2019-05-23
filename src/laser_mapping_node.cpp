#include <ros/ros.h>
#include "loam_velodyne/LaserMapping.h"
#include "loam_velodyne/MapManager.h"

/** Main node entry point. */
//void readValidationMessageFile(nav_msgs::Odometry &readMessage);
//void compareOdometryMessage(nav_msgs::Odometry & msg1, nav_msgs::Odometry & msg2);


int main(int argc, char **argv) {
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle node;
    ros::NodeHandle privateNode("~");

    bool status = ros::ok();


     // std::string path = ros::package::getPath("loam_velodyne") + "/pointEqTime.txt";
    // std::fstream file;
    // file.open(path, std::fstream::out); // Creates empty file
    // file.close();
    // file.open(path, std::fstream::app); // Opens file in append mode

    // PlaneFeature pF;

    // for (int i = 0; i < 10000; i++)
    // {

    //     Eigen::Vector4d point(i, i, i, i);
    //     pF.addPoint(point);
    //     if (i > 5)
    //     {
    //         clock_t t1 = clock();
    //         pF.calcPlaneEq();
    //         clock_t t2 = clock();
    //         file << " PointNum: " << pF.pointsNum() << " Delta Time: " << double(t2 - t1) << " Time: " << t2 << std::endl;
    //     }
    // }

    // file.close();

    loam::LaserMapping laserMapping(0.1);

    if (laserMapping.setup(node, privateNode)) {
        laserMapping.spin();
    }


    std::cout << std::endl << std::endl << "<------------------Mapping Execution Time---------------->" << std::endl;

    std::cout << "Mapping Process Time:       "
              << double(laserMapping.timeMeasurement.totalTimeMappingProcess() / CLOCKS_PER_SEC) << "  Avg: "
              << double(laserMapping.timeMeasurement.totalTimeMappingProcess() / CLOCKS_PER_SEC /
                        laserMapping.timeMeasurement.numOfCallsMappingProcess()) << std::endl;

    std::cout << "  Start Calculations Time:     "
              << double(laserMapping.timeMeasurement.totalTimeMappingStartCalculations() / CLOCKS_PER_SEC) << " Avg: "
              << double(laserMapping.timeMeasurement.totalTimeMappingStartCalculations() / CLOCKS_PER_SEC /
                        laserMapping.timeMeasurement.numOfCallsMappingStartCalculations()) << std::endl;

    std::cout << "  Set KDTree Input Time:       "
              << double(laserMapping.timeMeasurement.totalTimeMappingSetInputCloud() / CLOCKS_PER_SEC) << " Avg: "
              << double(laserMapping.timeMeasurement.totalTimeMappingSetInputCloud() / CLOCKS_PER_SEC /
                        laserMapping.timeMeasurement.numOfCallsMappingSetInputCloud()) << std::endl;

    std::cout << "  Corner For Loop Time:        "
              << double(laserMapping.timeMeasurement.totalTimeMappingConditionsCorners() / CLOCKS_PER_SEC) << " Avg: "
              << double(laserMapping.timeMeasurement.totalTimeMappingConditionsCorners() / CLOCKS_PER_SEC /
                        laserMapping.timeMeasurement.numOfCallsMappingConditionsCorners()) << std::endl;

    std::cout << "    KdSearch Corner Time:        "
              << double(laserMapping.timeMeasurement.totalTimeMappingKdSearchCorners() / CLOCKS_PER_SEC) << " Avg: "
              << double(laserMapping.timeMeasurement.totalTimeMappingKdSearchCorners() / CLOCKS_PER_SEC /
                        laserMapping.timeMeasurement.numOfCallsMappingKdSearchCorners()) << std::endl;

    std::cout << "  Surface For Loop Time:       "
              << double(laserMapping.timeMeasurement.totalTimeMappingConditionsSurfaces() / CLOCKS_PER_SEC) << " Avg: "
              << double(laserMapping.timeMeasurement.totalTimeMappingConditionsSurfaces() / CLOCKS_PER_SEC /
                        laserMapping.timeMeasurement.numOfCallsMappingConditionsSurfaces()) << std::endl;

    std::cout << "    KdSearch Surfaces Time:      "
              << double(laserMapping.timeMeasurement.totalTimeMappingKdSearchSurfaces() / CLOCKS_PER_SEC) << " Avg: "
              << double(laserMapping.timeMeasurement.totalTimeMappingKdSearchSurfaces() / CLOCKS_PER_SEC /
                        laserMapping.timeMeasurement.numOfCallsMappingKdSearchSurfaces()) << std::endl;

    std::cout << "  Optimization Time:           "
              << double(laserMapping.timeMeasurement.totalTimeMappingOptimalization() / CLOCKS_PER_SEC) << " Avg: "
              << double(laserMapping.timeMeasurement.totalTimeMappingOptimalization() / CLOCKS_PER_SEC /
                        laserMapping.timeMeasurement.numOfCallsMappingOptimalization()) << std::endl;

    std::cout << "  End Calculations Time:       "
              << double(laserMapping.timeMeasurement.totalTimeMappingEndCalculations() / CLOCKS_PER_SEC) << " Avg: "
              << double(laserMapping.timeMeasurement.totalTimeMappingEndCalculations() / CLOCKS_PER_SEC /
                        laserMapping.timeMeasurement.numOfCallsMappingEndCalculations()) << std::endl;


    std::cout << std::endl << std::endl << "<------------------Features Found---------------->" << std::endl;

    std::cout << "Plane Features Number:       " << laserMapping.mapManager.planeFeaturesNum() << std::endl;

    laserMapping.mapManager.writeStatsToFile(ros::package::getPath("loam_velodyne") + "/features_statistics.txt");



    return 0;
}