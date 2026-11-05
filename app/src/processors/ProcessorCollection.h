#pragma once

#include <thread>
#include "src/util/Semaphore.h"

#include "src/model/DataModel.h"

#include "src/processors/calibration/CameraCalibrator.h"
#include "src/processors/input/CameraDataProvider.h"
#include "src/processors/input/TickProvider.h"
#include "src/processors/marker/RawMarkerTriangulator.h"
#include "src/processors/marker/MarkerPersistor.h"
#include "src/processors/rigid/RigidBodyDetector.h"
#include "src/processors/rigid/GroundTruthEvaluator.h"
#include "src/processors/skeleton/SkeletalFitter.h"

using namespace std::chrono;

struct ProcessorCollection {
    DataModel& model;

    /**
     * Calibrates the cameras (intrinsically & extrinsically)
     */
    std::shared_ptr<TickProvider> tickProvider;

    /**
     * Calibrates the cameras (intrinsically & extrinsically)
     */
    std::shared_ptr<CameraCalibrator> cameraCalibrator;

    /**
     * Provides 2D marker pakets from cameras.
     */
    std::shared_ptr<CameraDataProvider> cameraDataProvider;

    /**
     * Generates 3D markers from 2D markers of multiple images.
     */
    std::shared_ptr<RawMarkerTriangulator> markerTriangulator;

    /**
     * Assign and tracks ids of Marker3D
     */
    std::shared_ptr<MarkerPersistor> markerPersistor;

    /**
     * Detects a rigid body from a cloud of markers using a simple brute force approach
     */
    std::shared_ptr<RigidBodyDetector> rigidBodyDetector;
    std::shared_ptr<GroundTruthEvaluator> groundTruthEvaluator;

    /**
     * Fits skeletons into the found markers.
     */
    std::shared_ptr<SkeletalFitter> skeletalFitter;

    /**
     * The processing rate (ticks per second) at which the pipeline should run.
     */
    float requestedProcessingRate = 120.;

    /**
     * The delay with which the frames should be processed.
     */
    float processingDelay = 0.1;

    /**
     * The actual maintained measured processing rate.
     */
    float measuredProcessingRate = 0.;

    /**
     * Should the network reinitialized the next frame?
     */
    bool reinitNetworkNextFrame = false;

    std::thread processingThread;
    bool isProcessingThreadRunning;

    Semaphore nextMarker2DPaketsAvailable;
    Semaphore markersCopiedSemaphore;

    /**
     * @brief ProcessorCollection
     * @param model
     */
    ProcessorCollection(DataModel& model);
    ~ProcessorCollection();

    std::map<std::string, high_resolution_clock::time_point> startTimeMeasureMap;
    std::map<std::string, float> timeMeasureMap;

    inline void startTimeMeasure(std::string str);

    inline void endTimeMeasure(std::string str);
    std::map<std::string, float> getTimeMeasuresInMilliSec();

private:
    void reinitNetwork(DataModel& model);
};
