//
// Created by mnowicki on 05.12.18.
//

#ifndef LOAM_VELODYNE_TIMEMEASUREMENT_H
#define LOAM_VELODYNE_TIMEMEASUREMENT_H

#include <iostream>

class TimeMeasurement {
public:
    TimeMeasurement () :    _numOfCallsMappingProcess(0),
                            _totalTimeMappingProcess(0),
                            _numOfCallsMappingOptimalization(0),
                            _totalTimeMappingOptimalization(0),
                            _numOfCallsMappingEndCalculations(0),
                            _totalTimeMappingEndCalculations(0),
                            _numOfCallsMappingStartCalculations(0),
                            _totalTimeMappingStartCalculations(0),
                            _numOfCallsMappingConditionsEdges(0),
                            _totalTimeMappingConditionsEdges(0),
                            _numOfCallsMappingConditionsSurfaces(0),
                            _totalTimeMappingConditionsSurfaces(0),
                            _numOfCallsMappingKdSearchEdges(0),
                            _totalTimeMappingKdSearchEdges(0),
                            _numOfCallsMappingKdSearchSurfaces(0),
                            _totalTimeMappingKdSearchSurfaces(0),
                            _numOfCallsMappingSetInputCloud(0),
                            _totalTimeMappingSetInputCloud(0) {

    }

    size_t numOfCallsMappingProcess() { return _numOfCallsMappingProcess; }
    double totalTimeMappingProcess() { return _totalTimeMappingProcess; }

    size_t numOfCallsMappingOptimalization() { return _numOfCallsMappingOptimalization; }
    double totalTimeMappingOptimalization() { return _totalTimeMappingOptimalization; }

    size_t numOfCallsMappingEndCalculations() { return _numOfCallsMappingEndCalculations; }
    double totalTimeMappingEndCalculations() { return _totalTimeMappingEndCalculations; }

    size_t numOfCallsMappingStartCalculations() { return _numOfCallsMappingStartCalculations; }
    double totalTimeMappingStartCalculations() { return _totalTimeMappingStartCalculations; }

    size_t numOfCallsMappingConditionsCorners() { return _numOfCallsMappingConditionsEdges; }
    double totalTimeMappingConditionsCorners() { return _totalTimeMappingConditionsEdges; }

    size_t numOfCallsMappingConditionsSurfaces() { return _numOfCallsMappingConditionsSurfaces; }
    double totalTimeMappingConditionsSurfaces() { return _totalTimeMappingConditionsSurfaces; }

    size_t numOfCallsMappingKdSearchCorners() { return _numOfCallsMappingKdSearchEdges; }
    double totalTimeMappingKdSearchCorners() { return _totalTimeMappingKdSearchEdges; }

    size_t numOfCallsMappingKdSearchSurfaces() { return _numOfCallsMappingKdSearchSurfaces; }
    double totalTimeMappingKdSearchSurfaces() { return _totalTimeMappingKdSearchSurfaces; }

    size_t numOfCallsMappingSetInputCloud() { return _numOfCallsMappingSetInputCloud; }
    double totalTimeMappingSetInputCloud() { return _totalTimeMappingSetInputCloud; }

public:
    size_t _numOfCallsMappingProcess; ///< Number of calls of process()
    double _totalTimeMappingProcess;  ///< Total Time spent on process()

    size_t _numOfCallsMappingConditionsEdges; ///< Number of calls of conditions calc
    double _totalTimeMappingConditionsEdges;  ///< Total Time spent on conditions calc

    size_t _numOfCallsMappingConditionsSurfaces; ///< Number of calls of conditions calc
    double _totalTimeMappingConditionsSurfaces;  ///< Total Time spent on conditions calc

    size_t _numOfCallsMappingOptimalization; ///< Number of calls of optimalization
    double _totalTimeMappingOptimalization;  ///< Total Time spent on optimalization

    size_t _numOfCallsMappingEndCalculations; ///< Number of calls of end calculations
    double _totalTimeMappingEndCalculations;  ///< Total Time spent on end calculations

    size_t _numOfCallsMappingStartCalculations; ///< Number of calls of start calculations
    double _totalTimeMappingStartCalculations;  ///< Total Time spent on start calculations

    size_t _numOfCallsMappingKdSearchEdges; ///< Number of calls of KdSearch Corners
    double _totalTimeMappingKdSearchEdges; ///< Total Time spent on KdSearch Corners

    size_t _numOfCallsMappingKdSearchSurfaces; ///< Number of calls of KdSearch Surfaces
    double _totalTimeMappingKdSearchSurfaces; ///< Total Time spent on KdSearch Surfaces

    size_t _numOfCallsMappingSetInputCloud; ///< Number of calls of setting KdSearch  input
    double _totalTimeMappingSetInputCloud; ///< Total Time spent on setting KdSearch input
};

#endif //LOAM_VELODYNE_TIMEMEASUREMENT_H
