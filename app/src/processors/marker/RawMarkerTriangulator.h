#pragma once

#include <vector>

#include "src/model/camera/Camera.h"

#include "src/model/DataModel.h"

#include "src/processors/Processor.h"

#include "src/util/math/Vec4.h"

#define MAX_INTERSECTION_DISTANCE 0.001
#define MAX_CLUSTER_DISTANCE 0.005
#define MIN_RAY_INTERSECTIONS 2

/**
 * Generates 3D markers using multiple 2D camera images.
 */
class RawMarkerTriangulator : public Processor {
    struct Intersection {
        ImageMarkerId i;
        ImageMarkerId j;
        Vec4f point;

        Intersection(ImageMarkerId i, ImageMarkerId j, Vec4f point)
            : i(i)
            , j(j)
            , point(point)
        {}
    };

    struct Cluster {
        std::vector<Vec4f> points;
        std::unordered_set<ImageMarkerId> markers;

        float score;
    };

    struct Ray {
        Vec4d origin;
        Vec4d direction;

        ImageMarkerId imageMarker;

        Ray(Vec4d origin, Vec4d direction, ImageMarkerId imageMarker)
            : origin(origin)
            , direction(direction)
            , imageMarker(imageMarker)
        {}
    };

    /**
     * Returns all Rays in world space of all cameras.
     */
    static std::vector<Ray> getAllRays(std::map<int, Camera>& cameras, std::vector<CameraFrame>& frames);

    /**
     * Returns all intersection of the given rays (which are <Position, Direction> pairs).
     */
    static std::vector<Intersection> getIntersections(std::vector<Ray>& allRays);

    /**
     * Generate clusters of intersections.
     */
    static std::vector<Cluster> generateClusters(std::vector<Intersection>& intersections);


public:
    /**
     * Marker triangulator.
     */
    RawMarkerTriangulator(DataModel& model)
        : Processor(model){}

    /**
     * Reads the 2D markers and generates 3D markers from it.
     */
    virtual void process(TickData& tick) override;

    /**
     * Triangulates the given cameras & markerPakets, and calculates statistics
     * if markerGTPos + medianGroundTruthDistance or medianInnerClusterDistance
     * or meanClusterNum are given.
     */
    static std::vector<RawMarker> generateUnlabeled3DMarkers(
        DataModel& model,
        TickData& tick,
        float* medianInnerClusterDistance = nullptr,
        float* meanClusterNum = nullptr,
        float* medianGroundTruthDistance = nullptr
    );

    /**
     * Median distance within a cluster. Can be a hint for precision of marker2D creation.
     */
    float medianInnerClusterDistance = 0;

    /**
     * Mean number of clusters.
     */
    float meanClusterNum = 0;

    /**
     * Median distance to ground truth coordinates (Prone to poor registration).
     */
    float medianGroundTruthDistance = 0;
};
