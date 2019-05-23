add_library(loam
            ScanRegistration.cpp
            BasicScanRegistration.cpp
            MultiScanRegistration.cpp
            Optimization.cpp
            LaserOdometry.cpp
            BasicLaserOdometry.cpp
            LaserMapping.cpp
            BasicLaserMapping.cpp
            TransformMaintenance.cpp
            BasicTransformMaintenance.cpp
        FeatureUtils.cpp
        MapManager.cpp)
target_link_libraries(loam ${catkin_LIBRARIES} ${PCL_LIBRARIES}) ## ${OpenCV_LIBS}) #### TODO: FOR TESTS!
