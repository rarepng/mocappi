#include "src/processors/ProcessorCollection.h"

#include "src/util/network/CameraNetworkUtils.h"

ProcessorCollection::ProcessorCollection(DataModel& model)
    : model(model)
    , markersCopiedSemaphore(1){
    tickProvider = std::make_shared<TickProvider>(model);
    cameraCalibrator = std::make_shared<CameraCalibrator>(model);
    cameraDataProvider = std::make_shared<CameraDataProvider>(model);
    markerTriangulator = std::make_shared<RawMarkerTriangulator>(model);
    markerPersistor = std::make_shared<MarkerPersistor>(model);
    rigidBodyDetector = std::make_shared<RigidBodyDetector>(model);
    groundTruthEvaluator = std::make_shared<GroundTruthEvaluator>(model);
    skeletalFitter = std::make_shared<SkeletalFitter>(model);

    // Processes the next marker pakets when available in a separate thread
    // (so that the udp receiving thread doesn't have to wait):
    processingThread = std::thread([this, &model](){
        isProcessingThreadRunning = true;

        auto lastProcessingStartTime = high_resolution_clock::now();
        auto measuredProcessingStartTime = steady_clock::now();

        while(isProcessingThreadRunning){
            // Wait until it's time to process new frame:
            {
                duration<double> deltaTime;
                do {
                    deltaTime = high_resolution_clock::now() - lastProcessingStartTime;
                }
                while(deltaTime.count() < (1 / requestedProcessingRate));
                lastProcessingStartTime = high_resolution_clock::now();
            }

            // Measure real processing rate:
            {
                auto now = steady_clock::now();
                measuredProcessingRate = (1.0/duration<double>(now - measuredProcessingStartTime).count());// * 0.05 + measuredProcessingRate * 0.95;
                measuredProcessingStartTime = now;
            }

            // Create a new model and update it at the end so the view doesn't read
            // models that are only particially updated:
            TickData tick = tickProvider->generateTick(cameraDataProvider->getHostTimestamp() - processingDelay);

            // Processes the marker provider:
            startTimeMeasure("Marker provider");
            cameraDataProvider->process(tick);
            endTimeMeasure("Marker provider");

            // Processes the camera calibrator
            startTimeMeasure("Camera Calibration");
            cameraCalibrator->process(tick);
            endTimeMeasure("Camera Calibration");

            // Process the input:
            startTimeMeasure("Marker Triangulation");
            markerTriangulator->process(tick);
            endTimeMeasure("Marker Triangulation");

            // Assign labels:
            startTimeMeasure("Marker Labeling");
            markerPersistor->process(tick);
            endTimeMeasure("Marker Labeling");

            // Rigid Body Detector:
            startTimeMeasure("Rigid Body Detector");
            rigidBodyDetector->process(tick);
            endTimeMeasure("Rigid Body Detector");

            groundTruthEvaluator->process(tick);

            startTimeMeasure("Skeletal Fitter");
            skeletalFitter->process(tick);
            endTimeMeasure("Skeletal Fitter");

            // Update the current model:
            model.lastTick = tick;

            // Sleep (less cpu time when waiting active):
           // std::this_thread::sleep_for(milliseconds(1));

            if(reinitNetworkNextFrame){
                reinitNetwork(model);
                reinitNetworkNextFrame = false;
            }
        }
    });
}

ProcessorCollection::~ProcessorCollection(){
    isProcessingThreadRunning = false;
    nextMarker2DPaketsAvailable.release();
    markersCopiedSemaphore.release();
    if(processingThread.joinable())
        processingThread.join();
}

void ProcessorCollection::startTimeMeasure(std::string str){
    auto time = high_resolution_clock::now();
    startTimeMeasureMap[str] = time;
}

void ProcessorCollection::endTimeMeasure(std::string str){
    auto time = high_resolution_clock::now();

    float duration = duration_cast<microseconds>(time - startTimeMeasureMap[str]).count() / 1000.f;
    //if(timeMeasureMap.count(str) == 0){
        timeMeasureMap[str] = duration;
    //} else {
        timeMeasureMap[str] = duration * 0.05f + timeMeasureMap[str] * 0.95f;
    //}
}

std::map<std::string, float> ProcessorCollection::getTimeMeasuresInMilliSec(){
    return timeMeasureMap;
}

void ProcessorCollection::reinitNetwork(DataModel& model){
    cameraDataProvider->updateBroadcastAddrNextSend = true;

    // Scan again for camera devices:
    CameraNetworkUtils::scanRemoteCams();
};
