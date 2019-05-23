//
// Created by mnowicki on 05.12.18.
//

#include "loam_velodyne/MapManager.h"
#include <chrono>

bool enableModifications = false; // Global variable for modifications in other files (creating and publishing Depth Image, removing points based on depth map)

MapManager::MapManager() : _enabled(true),                                             // Enables / Disables MapManager
                           _optimizationEnabled(true),                                  // Enables Mapping Optimization based on planes
                           _featureValidationType(0),                                  // 0 - Check based on planarity, 1 - Check based on curvature, 2 - no Validation
                           _mergingType(0),                                            // 0 - Merging based on plane-plane angle, cross-equations and pt-pt dst, 1 - Mergin based on std deviation and mean
                           _minDistancePointToPlaneFeature(0.5),            // 0.5 - malo drzew             // Min distance between new point and plane (equation) of feature
                           _minDistancePointToClosestPointFromPlaneFeature(0.5),  // 0.5 - malo drzew, OK|| 1 - dryfuje na prawo na początku || 2 - duzo pkt na asfalcie i drzew   // Biggger the value, more asphalt point added to optimization but also more trees  // Min distance between new point and closest point from feature
                           _minNumOfPointsForPlaneEqUpdate(5),                      // Min number of points to calculate plane equation - Min value 3
                           _maxNumOfPointsForPlaneEqUpdate(40),                    // Must change also Variable in PlaneFeature Class  // Max number of points to calculate plane equation
                           _maxPlaneDifferenceForMerge(10),                         // Max difference in angles between planes to merge them
                           _minPlanePointsForVoxelGrid(25),
                           _planeFeaturesCloud(new pcl::PointCloud<pcl::PointXYZI>()), //
                           _planeFeaturesCloud2(new pcl::PointCloud<pcl::PointXYZI>()), //
                           _debuggingPoints(new pcl::PointCloud<pcl::PointXYZI>()),
                           _pointsUsedForOptimalization(new pcl::PointCloud<pcl::PointXYZI>()),
                           _oryginalOptimalizationPoints(new pcl::PointCloud<pcl::PointXYZI>()), 
                           _featuresPerScan(0),

                           _matchingMinDstPtToPl(0.2), // 0.2 - Too small? 0.3 - 1.5
                           _matchingMinDstPtToPt(1),
                           _deleting1PtNum(10),                                // Size of feature that will be deleted at once 
                           _deleting2PtNum(25),                                // Size of feature that will be deleted after n-iterations
                           _deleting2Iterations(4)        
{
}

void MapManager::addEdgePoint(const pcl::PointXYZI &pi)
{
    for (unsigned int i = 0; i < _edgeFeatures.size(); i++)
    {
        int l;
    }
}


void MapManager::addPlanePoint(Eigen::Vector4d &point)
{
    double minFoundDistance = _minDistancePointToPlaneFeature;
    double minFoundDistanceIndex = -1; // Index of cloeset plane feature
    for (unsigned int i = 0; i < _planeFeatures.size(); i++)
    {

        if (_planeFeatures[i].hasEquation() == true) // Min. 5 Points in Feature
        {
            double distance = fabs(_planeFeatures[i].planeEq().head<3>().dot(point.head<3>()) + _planeFeatures[i].planeEq()(3)); // Calculates distance between point and plane
                                                                                                                                 //    std::cout << "Distance: " << distance << std::endl;
            if (distance < minFoundDistance)
            {
                minFoundDistance = distance;
                minFoundDistanceIndex = i;
            }
        }
        else
        {
            for (int j = 0; j < _planeFeatures[i].pointsNum(); j++)
            {
                double distance = FeatureUtils::computeDistanceBetweenPoints(_planeFeatures[i].getPoint(j), point); // Calculates distance between two points
                if (distance < minFoundDistance)
                {
                    minFoundDistance = distance; // Distance point - point
                    minFoundDistanceIndex = i;   // Index of feature containing closest point
                }
            }
        }
    }

    if (minFoundDistance < _minDistancePointToPlaneFeature) // Same value as minFoundDistance when initalized
    {
        point(3) = minFoundDistanceIndex; // Set intensity as index of feature for Rviz visualization
        _planeFeatures[minFoundDistanceIndex].addPoint(point);
    }
    else // Creates new Plane Feature
    {
        PlaneFeature newPlaneFeature = PlaneFeature();
        // add PlaneFeature ID
        newPlaneFeature.addPoint(point);
        point(3) = _planeFeatures.size(); // Set intensity as index of new feature for Rviz visualization
        _planeFeatures.push_back(newPlaneFeature);
    }

    pcl::PointXYZI pclPoint;
    pclPoint.x = point(0);
    pclPoint.y = point(1);
    pclPoint.z = point(2);
    pclPoint.intensity = point(3);
    _planeFeaturesCloud->push_back(pclPoint);
}

void MapManager::addPlanePoint2(Eigen::Vector4d &point)
{

     auto t1 = std::chrono::high_resolution_clock::now();
    std::vector<std::pair<int, double>> vector_featureIndex_pointDistance;
    //ToDo Check if next point also belongs to the same feature - for improving timing and reducing iterations?
    //ToDo Iterate from the end of feature points instead of beginning
    for (unsigned int i = 0; i < _planeFeatures.size(); i++)
    {
         
        if (_planeFeatures[i].hasEquation() == true && _planeFeatures[i].toDelete() == false) // Min. 5 Points in Feature
        {
            double distance = fabs(_planeFeatures[i].planeEq().head<3>().dot(point.head<3>()) + _planeFeatures[i].planeEq()(3)); // Calculates distance between point and plane
                                                                                                                                 // std::cout << " i: " << i;                                                                                                                          // std::cout << " 1: " << distance;
            if (distance < _minDistancePointToPlaneFeature)                                                                      // Adds only plane feature with more than 5 points and with distance  point-plane less than _minDistancePointToPlaneFeature
            {
                std::pair<int, double> new_featureIndex_pointDistance;
                new_featureIndex_pointDistance.first = i;
                new_featureIndex_pointDistance.second = distance;
                vector_featureIndex_pointDistance.push_back(new_featureIndex_pointDistance);
            }
        }
    }

    auto t2 = std::chrono::high_resolution_clock::now();
    // std::cout << "size: " << vector_featureIndex_pointDistance.size() << std::endl;
    std::sort(vector_featureIndex_pointDistance.begin(), vector_featureIndex_pointDistance.end(), // Sort features from smallest to biggest distance to point
              [](std::pair<int, double> &i, std::pair<int, double> &j) -> bool { return i.second < j.second; });

    auto t3 = std::chrono::high_resolution_clock::now();
    
    
    double minFoundPointToPointDistance = 999, minFoundPointToPointDistanceFeatureIndex = -1;
    // Check smallest point-point distance from 3 closest point-plane distance feature
    // ToAdd: if found point that meet criterias then break loop   
    double minDistances[3] = {999, 999, 999};
    for (int i = 0; i < vector_featureIndex_pointDistance.size(); i++)
    {
        
        if (i > 2)
            break;
        // std::cout << vector_featureIndex_pointDistance[i].second << std::endl;
        int featureIndex = vector_featureIndex_pointDistance[i].first;
        for (int j = 0; j < _planeFeatures[featureIndex].pointsNum(); j++)      // Takes a lot of time for big features
        {   
            // if (j > 100)  // Point limit  ??
            //  break;

            double distance = FeatureUtils::computeDistanceBetweenPoints(_planeFeatures[featureIndex].getPoint(j), point); // Calculates distance between two points
            if (distance < minDistances[i])
            {
                minDistances[i] = distance;
            }
            if (distance < minFoundPointToPointDistance)
            {
                minFoundPointToPointDistance = distance;                 // Distance point - point
                minFoundPointToPointDistanceFeatureIndex = featureIndex; // Index of feature containing closest point
                
            }
        }
    }

    std::vector<double> distancesVect (minDistances, minDistances+3);
     std::sort (distancesVect.begin(), distancesVect.end());


    auto t4 = std::chrono::high_resolution_clock::now();
    if (minFoundPointToPointDistance < _minDistancePointToClosestPointFromPlaneFeature) // If there exist point that meets both conditions (min Plane-Point distance and min Point-Point distance) then add him to this feature                                                                                       // Feature points with valid plane Equations have priority over closer Point-Point distance features without valid plane equation
    {
        if ((distancesVect[0] / distancesVect[1]) > 0.7)
            return;

        if (distancesVect[1] < _minDistancePointToClosestPointFromPlaneFeature)
            if (vector_featureIndex_pointDistance.size() >= 2)
                if ((vector_featureIndex_pointDistance[0].second / vector_featureIndex_pointDistance[1].second) > 0.7)
                    return;

        point(3) =_planeFeatures[minFoundPointToPointDistanceFeatureIndex].id(); //  = minFoundPointToPointDistanceFeatureIndex;                      // Set intensity as index of feature for Rviz visualization
        _planeFeatures[minFoundPointToPointDistanceFeatureIndex].addPoint(point); // Takes a lot of time for bigger features
         //std::cout << "Added point to existing feature" << std::endl;
    }
    

    auto t5 = std::chrono::high_resolution_clock::now();
    // else   // else changed to if for time measurement

     // Check smallest point-point distance from features without plane equation (less than 5 points)
    if (minFoundPointToPointDistance >= _minDistancePointToClosestPointFromPlaneFeature)
    {
        for (unsigned int i = 0; i < _planeFeatures.size(); i++)
        {
            if ((_planeFeatures[i].hasEquation() == false))
            {
                for (int j = 0; j < _planeFeatures[i].pointsNum(); j++)
                {
                    double distance = FeatureUtils::computeDistanceBetweenPoints(_planeFeatures[i].getPoint(j), point); // Calculates distance between two points
                    if (distance < minFoundPointToPointDistance)
                    {
                        minFoundPointToPointDistance = distance;      // Distance point - point
                        minFoundPointToPointDistanceFeatureIndex = i; // Index of feature containing closest point
                    }
                }
            }
        }
        if (minFoundPointToPointDistance < _minDistancePointToClosestPointFromPlaneFeature) // Min point - point distance to add point to existing feature
        {
            point(3) =  _planeFeatures[minFoundPointToPointDistanceFeatureIndex].id(); // Set intensity as index of feature for Rviz visualization
            _planeFeatures[minFoundPointToPointDistanceFeatureIndex].addPoint(point);
           // std::cout << "Added point to existing feature" << std::endl;
        }
        else // Creates new Plane Feature
        {
           //   if (_planeFeatures.size() < 2 )
            {
            PlaneFeature newPlaneFeature = PlaneFeature();
            // add PlaneFeature ID
          //  std::cout << "New Feature Created" << std::endl;
            _featuresPerScan++;
            newPlaneFeature.addPoint(point);
            point(3) = newPlaneFeature.id(); // Set intensity as index of new feature for Rviz visualization
            _planeFeatures.push_back(newPlaneFeature);
            
            }
        }
    }

    auto t6 = std::chrono::high_resolution_clock::now();
    auto t7 = std::chrono::high_resolution_clock::now();

    static int cnt = 0;
    static double dt1 = 0, dt2 = 0, dt3 = 0, dt4 = 0, dt5 = 0, dt6 = 0;
    dt1 += std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count();
    dt2 += std::chrono::duration_cast<std::chrono::microseconds>(t3-t2).count();
    dt3 += std::chrono::duration_cast<std::chrono::microseconds>(t4-t3).count();
    dt4 += std::chrono::duration_cast<std::chrono::microseconds>(t5-t4).count();
    dt5 += std::chrono::duration_cast<std::chrono::microseconds>(t6-t5).count();
    dt6 += std::chrono::duration_cast<std::chrono::microseconds>(t7-t6).count();

    cnt++;

    if (cnt > 100000)
    {
        cnt = 0;
        std::cout << "T1: " << dt1 / 1000000 << " T2: " << dt2 / 1000000 << " T3: " << dt3 / 1000000 << " T4: " << dt4 / 1000000 << " T5: " << dt5 / 1000000 << " T6: " << dt6 / 1000000 << std::endl;
    }
}

bool MapManager::matchPointToPlane(Eigen::Vector4d &point, PlaneFeature & matchedPF)
{

    std::vector<std::pair<int, double>> vector_featureIndex_pointDistance;
    for (unsigned int i = 0; i < _planeFeatures.size(); i++)
    {       
        if (_planeFeatures[i].hasEquation() == true) // Min. 5 Points in Feature
        {
            double distance = fabs(_planeFeatures[i].planeEq().head<3>().dot(point.head<3>()) + _planeFeatures[i].planeEq()(3)); // Calculates distance between point and plane
                                                                                                                                 // std::cout << " i: " << i;                                                                                                                          // std::cout << " 1: " << distance;
            if (distance < _matchingMinDstPtToPl)                                                                      // Adds only plane feature with more than 5 points and with distance  point-plane less than _minDistancePointToPlaneFeature
            {
                std::pair<int, double> new_featureIndex_pointDistance;
                new_featureIndex_pointDistance.first = i;
                new_featureIndex_pointDistance.second = distance;
                vector_featureIndex_pointDistance.push_back(new_featureIndex_pointDistance);
            }
        }
    }
    

    std::sort(vector_featureIndex_pointDistance.begin(), vector_featureIndex_pointDistance.end(), // Sort features from smallest to biggest distance to point
              [](std::pair<int, double> &i, std::pair<int, double> &j) -> bool { return i.second < j.second; });

    

    double minFoundPointToPointDistance = 999, minFoundPointToPointDistanceFeatureIndex = -1;
    double minSecondFoundDistance = 999, minSecondFoundDistanceIndex = -1;
    // Check smallest point-point distance from 3 closest point-plane distance feature
    // ToDo: if found point that meet criterias then break loop
    double minDistances[3] = {999, 999, 999};
    for (int i = 0; i < vector_featureIndex_pointDistance.size(); i++)
    {
        
        if (i > 2)
            break;
        int featureIndex = vector_featureIndex_pointDistance[i].first;
        for (int j = 0; j < _planeFeatures[featureIndex].pointsNum(); j++)
        {   

            double distance = FeatureUtils::computeDistanceBetweenPoints(_planeFeatures[featureIndex].getPoint(j), point); // Calculates distance between two points
            if (distance < minDistances[i])
            {
                minDistances[i]= distance;
            }
            if (distance < minFoundPointToPointDistance)
            {
                minFoundPointToPointDistance = distance;                 // Distance point - point
                minFoundPointToPointDistanceFeatureIndex = featureIndex; // Index of feature containing closest point
            }
        }
    }
    std::vector<double> distancesVect (minDistances, minDistances+3);
   std::sort (distancesVect.begin(), distancesVect.end());
      

    if (minFoundPointToPointDistance < _matchingMinDstPtToPt) // If there exist point that meets both conditions (min Plane-Point distance and min Point-Point distance) then add him to this feature                                                                                       // Feature points with valid plane Equations have priority over closer Point-Point distance features without valid plane equation
    {
         if ( (distancesVect[0] / distancesVect[1]) > 0.7){
         //  std::cout << "Two close matches: " << minDistances[0] << "  and  "  << minDistances[1] << " and: " << minDistances[2] << std::endl;
          return false;
             }
        matchedPF = _planeFeatures[minFoundPointToPointDistanceFeatureIndex];
       
        return true;
    }
    else
    {
        return false;
    }
}

void MapManager::putAllFeaturesIntoCloud()
{
    Eigen::Vector4d point;
    pcl::PointXYZI pclPoint;
    _planeFeaturesCloud->clear();
    for (unsigned int i=0; i < _planeFeatures.size(); i++)
    {
        for (unsigned int j=0; j < _planeFeatures[i].pointsNum(); j++)
        {
            point = _planeFeatures[i].getPoint(j);
        
        pclPoint.x = point(0);
        pclPoint.y = point(1);
        pclPoint.z = point(2);
        pclPoint.intensity = _planeFeatures[i].id();
        _planeFeaturesCloud->push_back(pclPoint);
        }
    }

}

void MapManager::putBiggestFeaturesIntoCloud(int numOfFeatures)
{
    // Adds only selected features to cloud to visualize in Rviz

    // Should be called only before publishing topic
    std::vector<std::pair<int, double>> vect_sort_pointsNum;
    for (int i = 0; i < _planeFeatures.size(); i++)
    {
        std::pair<int, double> index_pointsNum;
        index_pointsNum.first = i;
        index_pointsNum.second = _planeFeatures[i].pointsNum();
        vect_sort_pointsNum.push_back(index_pointsNum);
    }

    std::sort(vect_sort_pointsNum.begin(), vect_sort_pointsNum.end(),
              [](std::pair<int, double> &i, std::pair<int, double> &j) -> bool { return i.second > j.second; });
    //
    _planeFeaturesCloud2->clear(); // Clears the cloud - std::alloc error without it
    for (int i = 0; i < vect_sort_pointsNum.size(); i++)
    {
        if (i >= numOfFeatures)
            break;
        //    std::cout << planeFeaturesCopy[i].pointsNum() << std::endl;

        int indexOfFeature = vect_sort_pointsNum[i].first;
        for (int j = 0; j < vect_sort_pointsNum[i].second; j++)
        {
            Eigen::Vector4d pTmp = _planeFeatures[indexOfFeature].getPoint(j);
            pcl::PointXYZI pclPoint2;
            pclPoint2.x = pTmp(0);
            pclPoint2.y = pTmp(1);
            pclPoint2.z = pTmp(2);
            pclPoint2.intensity = i;
            _planeFeaturesCloud2->push_back(pclPoint2);
        }
    }
}

void MapManager::addDebuggingPoint(pcl::PointXYZI pt)
{
    _debuggingPoints -> push_back(pt);
}

void MapManager::addPointsUsedForOptimalization(pcl::PointXYZI pt)
{
    _pointsUsedForOptimalization->push_back(pt);
}

  void MapManager::addOryginalOptimalizationPoints(pcl::PointXYZI pt)
  {
      _oryginalOptimalizationPoints->push_back(pt);
  }

void MapManager::updatePlaneFeatures()
{

    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
    downSizeFilter.setLeafSize(0.4, 0.4, 0.4);
    for (unsigned int i = 0; i < _planeFeatures.size(); i++)
    {
       
        if(_planeFeatures[i].toDelete() == true) {   // Features with 5 points that were not planar
            continue;
        }

        if (_planeFeatures[i].hasEquation() == false) // Check if feature has equation, if not it will be deleted
        {
            _planeFeatures[i].setToDelete(true);
            continue;
        }

        if (_planeFeatures[i].getChanged())
        {
                
            if (_planeFeatures[i].pointsNum() > _minPlanePointsForVoxelGrid)
            {
                //  std::cout << "Before: " << _planeFeatures[i].pointsNum();
                _planeFeatures[i].applyVoxelGrid(downSizeFilter);
                //  std::cout << "  After:  " << _planeFeatures[i].pointsNum() << std::endl;
            }
         
            if (_featureValidationType == 0)
            {
                  // Limits planeEq calculations when new points where added to the feature
                if (_planeFeatures[i].pointsUsedForEqCalc() < _maxNumOfPointsForPlaneEqUpdate)   // Calculate PlaneEq until used 40 points
                {                                                                 
                    _planeFeatures[i].calcPlaneEq(_maxNumOfPointsForPlaneEqUpdate); // Calculate PlaneEq if feature changed until used 40 points
                }
                _planeFeatures[i].checkPlanarity(); // Check based on planarity - calculates distance of every point to plane
            }

            else if (_featureValidationType == 1)
            {
                _planeFeatures[i].calcPlaneEq(-1);  // Calculate Plane Eq from all points every time feature changed ?
                _planeFeatures[i].checkCurvature(); // Check based on curvature - calcPlaneEq must be called from all points before this call
            }
        }

        // Every feature bigger than _minPlanePointsForVoxelGrid wont pass
        if (_planeFeatures[i].filterApplied() == false) // Delete Features that were not filtered
        {
            if (_planeFeatures[i].pointsNum() < _deleting1PtNum) // With less than 10 points delete at once  ||| 6 - drifts down ||| 15 - No trees tops ||| 10 -OK more trees
            {
                if (_planeFeatures.size() > 10)
                _planeFeatures[i].setToDelete(true);
            }
            else if (_planeFeatures[i].pointsNum() < _deleting2PtNum) // With less than 25 points delete after 5 iteration   // ||| 25
            {
                _planeFeatures[i].increaseToDeleteCounter();
                if (_planeFeatures[i].getToDeleteCounter() > _deleting2Iterations) // After x iterations // 2 - deleted small cars, 0 -> 180 Features, 5 - > 1000 Features ||| 4
                {
                    if (_planeFeatures.size() > 10)
                    _planeFeatures[i].setToDelete(true);
                    // static int cnt = 0;
                    // cnt ++;
                    // std::cout << "deleted :  " << cnt << std::endl;
                }
            }
        }
    }

    deleteSelectedFeatures();
    if (_mergingType == 0)
        mergePlanes2(); //For merged features function calcPlaneEq and checkPlanarity() is called again
    else if (_mergingType == 1)
        mergePlanes3();

    statictics();
}

void MapManager::mergePlanes()  // Calculates angle between two planes and then distance point-point
{
    std::vector<int> featuresToRemove;
    int cnt = 0;
    for (int i = 0; i < _planeFeatures.size(); i++)
    {
        for (int j = i+1; j < _planeFeatures.size(); j++)
        {
            Eigen::Vector4d plane1 = _planeFeatures[i].planeEq(), plane2 = _planeFeatures[j].planeEq();
           
            if ( FeatureUtils::computeAngleBetweenPlanes(plane1,plane2) < _maxPlaneDifferenceForMerge )
            {
                double distance = 999;
                bool found = false;
                for (int k = 0; k < _planeFeatures[i].pointsNum(); k++)
                {
                    for (int l = 0; l < _planeFeatures[j].pointsNum(); l++)
                    {
                        distance = FeatureUtils::computeDistanceBetweenPoints(_planeFeatures[i].getPoint(k), _planeFeatures[j].getPoint(l));
                        if (distance < 1){
                            found = true;
                            break;
                        } 
                            
                    }
                    if (found == true)
                        break;
                }

                if (found == false)
                    continue;

                int fromIndex = -1, toIndex = -1;
                if (_planeFeatures[i].pointsNum() > _planeFeatures[j].pointsNum())
                {
                    fromIndex = j;
                    toIndex = i;
                }
                else
                {
                    fromIndex = i;
                    toIndex = j;
                }
                Eigen::Vector4d point;
                for (int k = 0; k < _planeFeatures[fromIndex].pointsNum(); k++)
                {
                    point = _planeFeatures[fromIndex].getPoint(k);
                    point(3) = _planeFeatures[toIndex].id();
                    _planeFeatures[toIndex].addPoint(point);
                }

                _planeFeatures.erase(_planeFeatures.begin() + fromIndex);
                j--;
                cnt++;
            }
        }
    }

    std::cout << "Merged: " << cnt << std::endl;
}

void MapManager::mergePlanes2() // Calculates angle between two planes and then check if points meets cross - equations, and  then calc distance point - point - eliminates problem of growing features upwards
{
    int cnt = 0;
    for (int i = 0; i < _planeFeatures.size(); i++)
    {
            for (int j = i + 1; j < _planeFeatures.size(); j++) {

                // We only merge plane features that were changed
                if (_planeFeatures[i].getChanged() || _planeFeatures[j].getChanged()){

                Eigen::Vector4d plane1 = _planeFeatures[i].planeEq(), plane2 = _planeFeatures[j].planeEq();

                if (FeatureUtils::computeAngleBetweenPlanes(plane1, plane2) < _maxPlaneDifferenceForMerge) {
                    bool merge = false;
                    double meanSqrErrorSelf1 = 0;
                    double meanSqrErrorCross12 = 0;
                    for (int k = 0; k < _planeFeatures[i].pointsNum(); k++) {
                        Eigen::Vector3d point = _planeFeatures[i].getPoint(k).head<3>();
                        double distanceSelf = plane1.head<3>().dot(point) + plane1(3);
                        double distanceCross = plane2.head<3>().dot(point) + plane2(3);

                        meanSqrErrorSelf1 += distanceSelf * distanceSelf;
                        meanSqrErrorCross12 += distanceCross * distanceCross;

                    }

                    meanSqrErrorSelf1 /= _planeFeatures[i].pointsNum();
                    meanSqrErrorCross12 /= _planeFeatures[i].pointsNum();

                    double meanSqrErrorSelf2 = 0;
                    double meanSqrErrorCross21 = 0;
                    for (int k = 0; k < _planeFeatures[j].pointsNum(); k++) {
                        Eigen::Vector3d point = _planeFeatures[j].getPoint(k).head<3>();;
                        double distanceSelf = plane2.head<3>().dot(point) + plane2(3);
                        double distanceCross = plane1.head<3>().dot(point) + plane1(3);

                        meanSqrErrorSelf2 += distanceSelf * distanceSelf;
                        meanSqrErrorCross21 += distanceCross * distanceCross;
                    }

                    meanSqrErrorSelf2 /= _planeFeatures[j].pointsNum();
                    meanSqrErrorCross21 /= _planeFeatures[j].pointsNum();

                    //   std::cout << "MSE_Self_12: " <<  meanSqrErrorSelf1 << " MSE_Cross_12: " <<  meanSqrErrorCross12
                    //         << " MSE_Self_21: " <<  meanSqrErrorSelf2 <<  " MSE_Cross_21: " <<  meanSqrErrorCross21  << std::endl;

                    
                    if (meanSqrErrorCross21 < 0.10 && meanSqrErrorCross12 < 0.10)
                    {

                         //ToDo: Algorithm that will quickly calculate closest point-point distance between points
                        double distance = 999;
                        for (int k = 0; k < _planeFeatures[i].pointsNum(); k++)
                        {
                            for (int l = 0; l < _planeFeatures[j].pointsNum(); l++)
                            {
                                distance = FeatureUtils::computeDistanceBetweenPoints(_planeFeatures[i].getPoint(k), _planeFeatures[j].getPoint(l));
                                if (distance < 1)
                                {
                                    merge = true;
                                    break;
                                }
                            }
                            if (merge == true)
                                break;
                        }
                    }

                    //                    if (meanSqrErrorCross12/meanSqrErrorSelf1 < 5 && meanSqrErrorCross21/meanSqrErrorSelf2 < 1)
                    //                        merge = true;

                    // Strategy similar to LOAM -> checking point from 1 to plane 2 and point from 2 to plane 1. If any point is further away than 15 cm then no merge
//                    bool merge = true;
//
//                    for (int k = 0; k < _planeFeatures[i].pointsNum(); k++) {
//                        Eigen::Vector3d point = _planeFeatures[i].getPoint(k).head<3>();
//                        double distanceCross = plane2.head<3>().dot(point) + plane2(3);
//                        if ( distanceCross > 0.15)
//                        {
//                            merge = false;
//                            break;
//                        }
//                    }
//
//                    if (merge) {
//                        for (int k = 0; k < _planeFeatures[j].pointsNum(); k++) {
//                            Eigen::Vector3d point = _planeFeatures[j].getPoint(k).head<3>();
//                            double distanceCross = plane1.head<3>().dot(point) + plane1(3);
//                            if ( distanceCross > 0.15)
//                            {
//                                merge = false;
//                                break;
//                            }
//                        }
//                    }


                    if (merge == false)
                    {
                         continue;
                    }

                    
                    
                    int fromIndex = -1, toIndex = -1;
                    if (_planeFeatures[i].pointsNum() > _planeFeatures[j].pointsNum()) {
                        fromIndex = j;
                        toIndex = i;
                    } else {
                        fromIndex = i;
                        toIndex = j;
                    }

                    bool mergeSuccess = false;
                    PlaneFeature mergedFeature = _planeFeatures[toIndex];
                    mergedFeature.setToDelete(false);
                   //std::cout << "s1  " << size1 << " size2  " <<  size2 << std::endl;

                    Eigen::Vector4d point;
                    for (int k = 0; k < _planeFeatures[fromIndex].pointsNum(); k++) {
                        point = _planeFeatures[fromIndex].getPoint(k);
                        point(3) = mergedFeature.id();
                        mergedFeature.addPoint(point);
                    }
                    //ToDo if Planarity < 0.8 after merge then dont merge, isMature = true?
                    
                    if (_featureValidationType == 0)
                    {
                        mergedFeature.calcPlaneEq(_maxNumOfPointsForPlaneEqUpdate); // Calculate Plane Eq from limited num of points
                        unsigned int pointsBefore = mergedFeature.pointsNum();
                        mergedFeature.checkPlanarity();
                        unsigned int pointsAfter = mergedFeature.pointsNum();
                       
                        // Dont merge if newly added points would be deleted anyway
                        if (pointsBefore - pointsAfter > _planeFeatures[fromIndex].pointsNum() * 0.3){ // If more than 30% of smaller feature size points would be deleted then dont merge
                            mergedFeature.setToDelete(true);
                            //std::cout << "Would delete: "  << (float)(pointsBefore - pointsAfter)/_planeFeatures[fromIndex].pointsNum() << " % of points" << std::endl;
                        }
                    }
                    else if (_featureValidationType == 1)
                    {
                        mergedFeature.calcPlaneEq(-1); // Calculate Plane Eq from all points
                        mergedFeature.checkCurvature();
                    }
                   
                    if (mergedFeature.toDelete() == true ) // Don't merge if Planarity/Curvature failed and feature is to be deleted
                    {
                    }
                    else // Merge 
                    {   
                         _planeFeatures[toIndex] = mergedFeature;
                         _planeFeatures.erase(_planeFeatures.begin() + fromIndex);
                         j--;
                         cnt++;
                    }          
                }
                }
            }

            _planeFeatures[i].setChanged(false); //
    }

  //  std::cout << "Merged: " << cnt << std::endl;

}

void MapManager::mergePlanes3() // Mergin based on mean and covariance
{

    int cnt = 0;
    for (int i = 0; i < _planeFeatures.size(); i++)
    {
        for (int j = i + 1; j < _planeFeatures.size(); j++)
        {

            // We only merge plane features that were changed
            if (_planeFeatures[i].getChanged() || _planeFeatures[j].getChanged())
            {

                Eigen::Vector4d plane1 = _planeFeatures[i].planeEq(), plane2 = _planeFeatures[j].planeEq();

                if (FeatureUtils::computeAngleBetweenPlanes(plane1, plane2) < _maxPlaneDifferenceForMerge)
                {
                    bool merge = false;

                    Eigen::Matrix3d cov1 =  _planeFeatures[i].covariance();
                    Eigen::Matrix3d cov2 =  _planeFeatures[j].covariance();
                    Eigen::Vector4d diff_mean = _planeFeatures[i].mean() - _planeFeatures[j].mean();
                    
                    // https://stats.stackexchange.com/questions/103800/calculate-probability-area-under-the-overlapping-area-of-two-normal-distributi
                    float u1[3], u2[3], o1[3], o2[3], nom[3], denom[3], c[3], f[3];
                    
                    for (int k=0; k<3; k++)
                    {
                       u1[k] = _planeFeatures[i].mean()(k);
                       u2[k] = _planeFeatures[j].mean()(k);
                       o1[k] = sqrt(fabs(cov1(k, k)));
                       o2[k] = sqrt(fabs(cov2(k, k)));
                       nom[k] = u2[k] * o1[k] * o1[k] - o2[k] * (u1[k] * o2[k] + o1[k] * sqrt((u1[k] - u2[k]) * (u1[k] - u2[k]) + 2 * (o1[k] * o1[k] - o2[k] * o2[k]) * log(o1[k] / o2[k])));
                       denom[k] = o1[k] * o1[k] - o2[k] * o2[k];
                       c[k] = nom[k] / denom[k];
                       f[k] = ( 1 / ( o1[k] * sqrt(2*M_PI) ) ) * exp( -0.5 * ((c[k] - u1[k]) / o1[k]) * ((c[k] - u1[k]) / o1[k]) );
                    }
                    if (f[0] > 0.008 &&  f[1] > 0.008  && f[2] > 0.008)
                    {
                        merge = true;
                    }  
                    //float pdf_gaussian = ( 1 / ( s * sqrt(2*M_PI) ) ) * exp( -0.5 * pow( (x-m)/s, 2.0 ) );

                   // std::cout << " f0:  " << f[0] << " f1:  " << f[1] << " f2:  " << f[2] << "  f0*f1*f2:  " << f[0] * f[1] * f[2] << std::endl;

                    if (merge == true)
                    {

                        int fromIndex = -1, toIndex = -1;
                        if (_planeFeatures[i].pointsNum() > _planeFeatures[j].pointsNum())
                        {
                            fromIndex = j;
                            toIndex = i;
                        }
                        else
                        {
                            fromIndex = i;
                            toIndex = j;
                        }
                        bool mergeSuccess = false;
                        PlaneFeature mergedFeature = _planeFeatures[toIndex];
                        mergedFeature.setToDelete(false);
                        //std::cout << "s1  " << size1 << " size2  " <<  size2 << std::endl;

                        Eigen::Vector4d point;
                        for (int k = 0; k < _planeFeatures[fromIndex].pointsNum(); k++)
                        {
                            point = _planeFeatures[fromIndex].getPoint(k);
                            point(3) = mergedFeature.id();
                            mergedFeature.addPoint(point);
                        }
                        //ToDo if Planarity < 0.8 after merge then dont merge, isMature = true?
                        if (_featureValidationType == 0)
                        {
                            mergedFeature.calcPlaneEq(_maxNumOfPointsForPlaneEqUpdate); // Calculate Plane Eq from limited num of points
                            mergedFeature.checkPlanarity();
                        }
                        else if (_featureValidationType == 1)
                        {
                            mergedFeature.calcPlaneEq(-1); // Calculate Plane Eq from all points
                            mergedFeature.checkCurvature();
                        }
                        if (mergedFeature.toDelete() == true) // Don't merge if Planarity/Curvature failed and feature is to be deleted
                        {
                            //  std::cout << "Merge would fail" << std::endl;
                        }
                        else // Merge
                        {
                            _planeFeatures[toIndex] = mergedFeature;
                            _planeFeatures.erase(_planeFeatures.begin() + fromIndex);
                            j--;
                            cnt++;
                        }
                    }
                }
            }
        }

        _planeFeatures[i].setChanged(false); //
    }

    //  std::cout << "Merged:  " << cnt << std::endl;
}

void MapManager::statictics()
{
    unsigned int sumOfPoints = 0;
    unsigned int featuresNum = _planeFeatures.size();
    unsigned int fwlt10p = 0;
    unsigned int fwlt5p = 0;
    unsigned int fnv = 0;
    unsigned int maxPointsFeature = 0, isPlanarNum = 0;
    int maxPointFeatureIndex = -1;
    double maxPFPlan = 0;
    unsigned int toDelete500 = 0;

    for (unsigned int i = 0; i < _planeFeatures.size(); i++)
    {
        sumOfPoints += _planeFeatures[i].pointsNum();

        if (_planeFeatures[i].pointsNum() < 10)
            fwlt10p++;

        if (_planeFeatures[i].pointsNum() < 5)
            fwlt5p++;

        if (_planeFeatures[i].hasEquation() == false)
            fnv++;

        if (_planeFeatures[i].pointsNum() > maxPointsFeature){
            maxPointsFeature = _planeFeatures[i].pointsNum();
            maxPointFeatureIndex = i;
        }

        if (_planeFeatures[i].isPlanar())
            isPlanarNum++;

        if (_planeFeatures[i].toDelete() == true && _planeFeatures[i].pointsNum() > 500)
            toDelete500++;
    }

    static int cnt = 0;
    cnt++;
    if (cnt > 10)
    {
        cnt = 0;
        std::cout << "SumPoints:  " << sumOfPoints << "  FeaturesNum: "  << featuresNum
                  //          << "  Features With Less than 10p: " << fwlt10p
                  //          << "  Features With Less than 5p: " << fwlt5p << " Max Point Feature: " << maxPointsFeature
                  << std::endl;
        std::cout << "Planar: " << isPlanarNum << " NotPlanar After Merge: " << featuresNum - isPlanarNum << std::endl;

        if (toDelete500 > 0 )
        std::cout << "To Delete (After Merges) bigger than 500 " << toDelete500 << std::endl;

        std::cout << "Feat Per Scan: " << _featuresPerScan/11 << std::endl;
        _featuresPerScan = 0;

        //if(maxPointFeatureIndex >= 0)
        // maxPFPlan = _planeFeatures[maxPointFeatureIndex].checkPlanarity();
        //Eigen::Vector4d planeZX; planeZX(0) = 0; planeZX(1) = 1; planeZX(2) = 0; planeZX(3) = 0;
        //std::cout << "Biggest Feature Eq  " << FeatureUtils::computeAngleBetweenPlanes(_planeFeatures[maxPointFeatureIndex].planeEq(), planeZX)  << std::endl;
    }
}



void MapManager::writeStatsToFile(std::string path)
{
     std::fstream file;
     file.open(path, std::fstream::out); // Creates empty file
     file.close();
     file.open(path, std::fstream::app); // Opens file in append mode

     PlaneFeature pF;

     for (unsigned int i = 0; i < _planeFeatures.size(); i++)
     {
            _planeFeatures[i].checkPlanarity();
            _planeFeatures[i].calcPlaneEq(_maxNumOfPointsForPlaneEqUpdate);
            file << "Id: " << _planeFeatures[i].id()
                 << " Size: " << _planeFeatures[i].pointsNum() 
                //<< " Mean: " << _planeFeatures[i].mean() 
               // << " Covariance: " << _planeFeatures[i].covariance() 
                << " IsPlanar: " << _planeFeatures[i].isPlanar() 
                << " Planarity: " << _planeFeatures[i].planarity() 
                << " Curvature: " << _planeFeatures[i].curvature()
                << " PointsForEq: " << _planeFeatures[i].pointsUsedForEqCalc() << std::endl;        
     }

     file.close();
}

void MapManager::deleteSelectedFeatures()
{
    _planeFeatures.erase(std::remove_if(_planeFeatures.begin(), _planeFeatures.end(), [](PlaneFeature & feature) -> bool { return feature.toDelete();} ), _planeFeatures.end());
}

int PlaneFeature::planeFeaturesObjectCount = 0;
// Plane Feature class constructor
PlaneFeature::PlaneFeature() : _numOfPointsForFirstPlaneCalc(5),
                               _maxNumOfPointsForPlaneEqUpdatePF(40), // Same as in Map Manager class
                               _planarityCheckDistance(0.2), 
                               _planarityThreshold(0.8),                          
                               _planeEq(0, 0, 0, 0),
                               _pointsUsedForEqCalc(0),
                               _hasEquation(false),
                               _changed(false),
                               _filterApplied(false),
                               _toDelete(false),
                               _isPlanar(false),
                               _isMature(false),
                               _mean(0,0,0,0),
                               _curvature(0),
                               _toDeleteCounter(0),
                               _planarity(0)
{
    _id = planeFeaturesObjectCount++;
}

void PlaneFeature::addPoint(const Eigen::Vector4d &point)
{
    _points.push_back(point);
    _changed = true;

    if (_points.size() == _numOfPointsForFirstPlaneCalc) // Exist 5 points for plane equation calculation
    {
        calcPlaneEq(_maxNumOfPointsForPlaneEqUpdatePF); 

       // if (_featureValidationType == 0)                          
          //  checkPlanarity();                                                   // Features with 5 points that are not planar will be deleted

       // else if (_featureValidationType == 1)
          //  checkCurvature();                                                   // Features with 5 points that are not planar will be deleted

        if (_isPlanar == false)
        {
        }
    }
}

void PlaneFeature::calcPlaneEq(int pointsLimit) // 
{
    // Limits points used for eq calc after merge
    float step  = 1.0;
    int pointsSize = _points.size();

    if (pointsSize < 3) 
        return;

    if (pointsLimit > 0)    // if pointsLimit <= 0 then use all points
    {
        if (pointsSize > pointsLimit)
        {
            step = (float)pointsSize / pointsLimit;
            pointsSize = pointsSize / step;
        }
    }

    // Chooses only pointsLimit points or less for calculations
    Eigen::MatrixXd points = Eigen::MatrixXd::Zero(4,pointsSize);
    for (int i = 0, j = 0; i < pointsSize; i++, j+=step)
    {
        points.col(i).head<3>() = _points[j].head<3>(3);
        if (j > _points.size())
            std::cout << "Indexes Error - CalcPlanEQ, Should never be displayed" << std::endl;
    }
    
    FeatureUtils::computePlaneEq(points, _planeEq, _mean, _covariance, _curvature);
    _hasEquation = true;
    _pointsUsedForEqCalc = pointsSize;
    
   // if (curvature > 0.00015) // https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=6948422&tag=1
   //   return false;
}

void PlaneFeature::checkPlanarity()
{
        
    bool isPlanar = true;
    unsigned int notPlanarPointsCnt = 0;
    
    std::vector<int> pointsToDeleteIdx;
    for (int i = 0; i < _points.size(); i++) {
        double distance = fabs(_planeEq.head<3>().dot(_points[i].head<3>()) + _planeEq(3));
        if (distance > _planarityCheckDistance){
            notPlanarPointsCnt++;     
            pointsToDeleteIdx.push_back(i);
        }
    }

    if ( (_points.size() - pointsToDeleteIdx.size()) < _numOfPointsForFirstPlaneCalc ) // If feature would be smaller than 5 points then delete it
    {
        // ToDo improve deleting too small features
        isPlanar = false;
        _toDelete = true;
        _planarity = 0;
        return;
    }
    
    double planarity = double((_points.size() - notPlanarPointsCnt)) / (_points.size()); // Percentage of co-planar points

    if (planarity > 0.95 )  // Do nothing
    {
        
    }
    else if (planarity > _planarityThreshold)   // Improve planarity by removing points
    {
            for (int i = pointsToDeleteIdx.size() - 1; i >= 0; i--)
            {
                _points[pointsToDeleteIdx[i]] = _points.back();
                _points.pop_back();
            }
            // ToDo Call only if some points were removed
            calcPlaneEq(_maxNumOfPointsForPlaneEqUpdatePF);
    }
    else // Not co-planar feature will be deleted
    {
         if (_points.size() > 400)
         {
          //   std::cout << "Deleted: " << _points.size() << "  Planarity:  " << planarity << std::endl;
         }
         isPlanar = false;
        _toDelete = true;
    }

    _isPlanar = isPlanar;
    _planarity = planarity;
}

void PlaneFeature::checkCurvature()
{
    // Calculates PlaneEq from all points and then checks curvature
    
    if( _curvature > 0.00015)  // https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=6948422&tag=1
        _toDelete  = true;
}



void PlaneFeature::applyVoxelGrid(pcl::VoxelGrid<pcl::PointXYZI> & downSizeFilter)
{
    _filterApplied = true;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZI>() );
    pointCloud->points.resize(_points.size());
    for (unsigned int i=0; i < _points.size(); i++)
    {
       pointCloud->points[i].x = _points[i](0);
        pointCloud->points[i].y = _points[i](1);
         pointCloud->points[i].z = _points[i](2);
    }
    downSizeFilter.setInputCloud(pointCloud);
    downSizeFilter.filter(*pointCloud);
   
    _points.resize(pointCloud->points.size());
    for (unsigned int i=0; i <_points.size(); i++)
    {
       _points[i](0) = pointCloud->points[i].x;
        _points[i](1) = pointCloud->points[i].y ;
         _points[i](2) =  pointCloud->points[i].z;
      
    }
}

EdgeFeature::EdgeFeature() : _edgeEq(0, 0, 0)
{
}