#ifndef LOAM_VELODYNE_MAP_MANAGER_H
#define LOAM_VELODYNE_MAP_MANAGER_H

#include <utility>
#include <vector>
#include <algorithm>
#include <fstream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/StdVector>
#include "FeatureUtils.h"

extern bool enableModifications;

class PlaneFeature;
class EdgeFeature;

class MapManager
{

public:
  MapManager();

  int planeFeaturesNum() { return _planeFeatures.size(); }      // Returns number of Plane Features in Map 

  auto &planeFeaturesCloud() { return *_planeFeaturesCloud; }   // All features cloud

  auto &planeFeaturesCloud2() { return *_planeFeaturesCloud2; } // Biggest features cloud

  auto &debuggingPoints() { return *_debuggingPoints; } // Debugging points cloud

  auto &pointsUsedForOptimalization() { return *_pointsUsedForOptimalization; } // Debugging points cloud

  auto &oryginalOptimalizationPoints() { return *_oryginalOptimalizationPoints; } // Debugging points cloud

  bool isEnabled() { return _enabled; }                         // Returns true if MapManager should be used

  bool isOptimizationEnabled() { return _optimizationEnabled; }

  void addEdgePoint(const pcl::PointXYZI &pi);                  // Adds Edge Point

  void addPlanePoint(Eigen::Vector4d &point);                   // Adds plane point v1 - calculates only distance Point-Feature

  void addPlanePoint2(Eigen::Vector4d &point);                  // Adds plane point v2 - calculates Point-Feature and Point-Point distance

  void putBiggestFeaturesIntoCloud(int numOfFeatures);          // Takes biggest features and puts them into _planeFeaturesCloud2

  void putAllFeaturesIntoCloud();

  void updatePlaneFeatures();                                   // Recalculates equation and applies Voxel Grid

  void mergePlanes();

  void mergePlanes2();

  void mergePlanes3();

  void statictics();

  void deleteSelectedFeatures();

  void writeStatsToFile(std::string path);

  bool matchPointToPlane(Eigen::Vector4d &point, PlaneFeature & matchedPF);

  void addDebuggingPoint(pcl::PointXYZI);

  void addPointsUsedForOptimalization(pcl::PointXYZI);

  void addOryginalOptimalizationPoints(pcl::PointXYZI);

  void setMinDistancePointToPlaneFeature(double val) { _minDistancePointToPlaneFeature = val; }
  void setMinDistancePointToClosestPointFromPlaneFeature(double val) { _minDistancePointToClosestPointFromPlaneFeature = val; }
  void setMatchingMinDstPtToPl(double val) { _matchingMinDstPtToPl = val; }
  void setMatchingMinDstPtToPt(double val) { _matchingMinDstPtToPt = val; }
  void setDeleting1PtNum(int val) { _deleting1PtNum = val; }
  void setDeleting2PtNum(int val) { _deleting2PtNum = val; }
  void setDeleting2Iterations(int val) { _deleting2Iterations = val; }

  unsigned int _featuresPerScan;

private: // Later change to private
  std::vector<PlaneFeature> _planeFeatures;                     // Vector of all planeFeatures
  std::vector<PlaneFeature> _edgeFeatures;                      // Vector of all edgeFeatures
  pcl::PointCloud<pcl::PointXYZI>::Ptr _planeFeaturesCloud;     //< Created only for Rviz
  pcl::PointCloud<pcl::PointXYZI>::Ptr _planeFeaturesCloud2;    //< Biggest plane Features Created only for Rviz
  pcl::PointCloud<pcl::PointXYZI>::Ptr _debuggingPoints;        //< Points that are far away from original system but found by this
  pcl::PointCloud<pcl::PointXYZI>::Ptr _pointsUsedForOptimalization; 
  pcl::PointCloud<pcl::PointXYZI>::Ptr _oryginalOptimalizationPoints; 

private:
  bool _enabled;
  bool _optimizationEnabled;

  // Parameters
  int _featureValidationType;
  int _mergingType; 
  double _minDistancePointToPlaneFeature;                 // Min distance between new point and plane (equation) of feature
  double _minDistancePointToClosestPointFromPlaneFeature; // Min distance between new point and closest point from feature
  int _minNumOfPointsForPlaneEqUpdate;                    // Min number of points to calculate plane equation
  int _maxNumOfPointsForPlaneEqUpdate;                    // Max number of points to calculate plane equation
  double _maxPlaneDifferenceForMerge;
  double _minPlanePointsForVoxelGrid;

  double _matchingMinDstPtToPl;
  double _matchingMinDstPtToPt;
  int _deleting1PtNum;  
  int _deleting2PtNum;
  int _deleting2Iterations;
};

class PlaneFeature
{

  static int planeFeaturesObjectCount;

public:
  PlaneFeature();
  void addPoint(const Eigen::Vector4d &point);
  void applyVoxelGrid(pcl::VoxelGrid<pcl::PointXYZI> & downSizeFilter);
  

  int id() { return _id; }                             // Returns id of feature, id is equal to index in _planeFeature vector
  bool hasEquation() { return _hasEquation; }                    // Returns true if Feature has calculated equation
  bool filterApplied() { return _filterApplied; }                    // Returns true if Feature has calculated equation
  bool getChanged() { return _changed; }
  void setChanged(bool val) { _changed = val; }
  unsigned int pointsUsedForEqCalc() { return _pointsUsedForEqCalc; }
  void setToDelete(bool val) { _toDelete = val; }
  unsigned long int pointsNum() { return _points.size(); } // Returns number of points in feature // _points.size()
  bool toDelete() { return _toDelete; }
  Eigen::Vector4d planeEq() { return _planeEq; }       // Returns plane equation
  Eigen::Vector4d getPoint(int idx) { return _points[idx]; } // Returns reference to idx'th point
  void calcPlaneEq(int pointsLimit);                                  // Calculates Plane equation using maximally pointsLimit points. If pointsLimit <= 0 then use all points
  void checkPlanarity();
  void checkCurvature();
  void increaseToDeleteCounter() { _toDeleteCounter++; }
  void resetToDeleteCounter() { _toDeleteCounter = 0; }
  int getToDeleteCounter() {return _toDeleteCounter; }
  Eigen::Vector4d mean() {return _mean;}
   Eigen::Matrix3d covariance() {return _covariance; }
  double curvature() {return _curvature; }
  double planarity() {return _planarity; }


  bool isPlanar() {
    return _isPlanar;
  }

private:
  bool _hasEquation;                                                                     // valid - feature has calculated equation
  bool _changed;
  bool _filterApplied;
  int _id;                                                                         // Id of feature - sequenced numbers
  unsigned long int _pointsNum;                                                    // Number of points in feature
  unsigned int _pointsUsedForEqCalc;
  Eigen::Vector4d _mean;
  Eigen::Matrix3d _covariance;
  double _curvature;
  double _planarity;
                                                 
  Eigen::Vector4d _planeEq;                                                        // Plane equation
  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> _points; // Points of feature  // 4th-Intensity for rviz // Changed for Vector3d - Vector4d requires Eigen aligned allocator

  bool _isPlanar; // it meets our planarity critiria
  bool _isMature;
  bool _toDelete;
  int _toDeleteCounter;

  // Parameters
  int _numOfPointsForFirstPlaneCalc; // Min number of points to calculate plane equation
  int _maxNumOfPointsForPlaneEqUpdatePF; // Max number of points used in calcPlaneEq function
  double _planarityCheckDistance;      // Max distance Point - Plane for plane to be planar
  double _planarityThreshold; // Threshold of planarity for delete

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class EdgeFeature
{
  EdgeFeature();
  Eigen::Vector3d _edgeEq;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> _pointCloud;
};
 
#endif