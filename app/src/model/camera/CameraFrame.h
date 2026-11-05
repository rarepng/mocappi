#pragma once

#include <nlohmann/json.hpp>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "src/model/primitive/ImageMarker.h"

/**
 * Represents data from a single camera frame.
 */
struct CameraFrame {
    /**
     * Is this camera frame valid
     */
    bool valid;

    /**
     * Unique camera id
     */
    int uniqueCamID;

    /**
     * All the markers which were found in this frame by the camera.
     */
    std::vector<ImageMarker> markers;

    /**
     * The synchronized timestamp of the host when the image was recorded.
     */
    double timestamp;

    /**
     * Creates an empty invalid frame.
     */
    CameraFrame()
        : valid(false), uniqueCamID(-1), timestamp(0.0)
    {}

    /**
     * Creates an interpolated frame, which is between frame a and b and has the
     * given desired timestamp. Only markers, for which correspondences were found,
     * are considered and inserted in this interpolated frame.
     *
     * Assumes that a.hostTimestamp <= desiredTimestamp <= b.hostTimestamp!
     */
    CameraFrame(CameraFrame& a, CameraFrame& b, double desiredTimestamp, bool estimateCorrespondences = true)
        : valid(false), uniqueCamID(-1), timestamp(0.0){
        if(a.timestamp > desiredTimestamp){
            std::cout << "WARNING: Frame-Interp-Constructor: a.hostTimestamp is bigger that the desired timestamp!" << std::endl;
            return;
        }

        if(b.timestamp < desiredTimestamp){
            std::cout << "WARNING: Frame-Interp-Constructor: b.hostTimestamp is smaller that the desired timestamp!" << std::endl;
            return;
        }

        if(a.uniqueCamID != b.uniqueCamID){
            std::cout << "WARNING: Frame-Interp-Constructor: Unique Cam IDs differ!!" << std::endl;
        }

        if(!a.valid || !b.valid){
            std::cout << "WARNING: Frame-Interp-Constructor: Invalid frames given!!" << std::endl;
        }

        // Set unique cam id:
        uniqueCamID = a.uniqueCamID;

        // Set timestamp:
        timestamp = desiredTimestamp;

        // Calculate alpha for interpolation of both frames:
        double alpha = (desiredTimestamp - a.timestamp) / (b.timestamp - a.timestamp);

        // Correspondences between both frames:
        std::vector<int> correspondences;

        // If correspondences should be estimated, do that (real camera data). Otherwise,
        // simply assume that markers with same id are the same (ground truth data):
        if(estimateCorrespondences){
            correspondences = CameraFrame::estimateCorrespondences(a, b);
        } else {
            // Simply create an identity map:
            for(unsigned int i=0; i < a.markers.size();++i){
                correspondences.push_back(i);
            }
        }

        // Iterate over all correspondences:
        for(unsigned int aIdx=0; aIdx < correspondences.size(); ++aIdx){
            int bIdx = correspondences[aIdx];

            // Only consider valid correspondences:
            if(bIdx >= 0){
                ImageMarker interpMarker;
                interpMarker.x = float((b.markers[bIdx].x - a.markers[aIdx].x) * alpha + a.markers[aIdx].x);
                interpMarker.y = float((b.markers[bIdx].y - a.markers[aIdx].y) * alpha + a.markers[aIdx].y);
                interpMarker.flowX = b.markers[bIdx].x - a.markers[aIdx].x;
                interpMarker.flowY = b.markers[bIdx].y - a.markers[aIdx].y;


                {
                    // === Fix rolling shutter ===
                    // When looking in the driver (https://github.com/raspberrypi/linux/blob/rpi-6.1.y/drivers/media/i2c/imx708.c)
                    // at mode 1536x864, we get a line length of 5216, and 566400000 pixels / seconds.
                    // So for a whole image, we need 7.96ms for read out.
                    //
                    // The sensor is 4608x2592, and 2x2-binned mode is used, 3072x1728 is the section/cutout which is halved.
                    // Assume scaling from 1536x864 -> 1280x720. Assume that top and bottom sensor rows are ignored, and that
                    // the 3072x1728 (2x2 bins) needs 7.96ms for readout.

                    // The flow returns the movement of the marker between frame a and b. To caluclate the part of the flow
                    // to which the marker has to be moved back, we have to calculate the difference of the frame timestamps
                    // and determine the part:

                    double flowTimePart = 0.00796 / (b.timestamp - a.timestamp);

                    // Now, we have to move the marker depending on the flow and it's flowTimePart and the height:
                    interpMarker.x -= float(interpMarker.flowX * flowTimePart * (interpMarker.y / 720));
                    interpMarker.y -= float(interpMarker.flowY * flowTimePart * (interpMarker.y / 720));
                }

                interpMarker.pixels = float((b.markers[bIdx].pixels - a.markers[aIdx].pixels) * alpha + a.markers[aIdx].pixels);
                interpMarker.intensity = float((b.markers[bIdx].intensity - a.markers[aIdx].intensity) * alpha + a.markers[aIdx].intensity);
                markers.push_back(interpMarker);
            }
        }

        // Set valid:
        valid = true;
    }

    /**
     * Removes all points by the given camera mask.
     */
    void filterByCameraMask(std::vector<ImageMarker>& cameraMask){
        markers.erase(std::remove_if(markers.begin(), markers.end(), [&cameraMask](ImageMarker& marker){
                for(const ImageMarker& maskMarker : cameraMask){
                    float dX = maskMarker.x - marker.x;
                    float dY = maskMarker.y - marker.y;
                    float dist = dX * dX + dY * dY;

                    if(dist < maskMarker.pixels){
                        return true;
                    }
                }
                return false;
            }
        ), markers.end());
    }


    /**
     * Estimates the flow for the markers for the given frame **b**, by using the
     * markers in frame a.
     */
    static void estimateFlow(CameraFrame& a, CameraFrame& b){
        std::vector<int> correspondences = estimateCorrespondences(a, b);

        // Iterate over all correspondences and calculate the flow / delta:
        for(int aIdx=0; aIdx < correspondences.size(); ++aIdx){
            int bIdx = correspondences[aIdx];

            // If there is a correspondence, estimate flow:
            if(bIdx >= 0){
                ImageMarker& markerA = a.markers[aIdx];
                ImageMarker& markerB = b.markers[bIdx];

                markerB.flowX = markerB.x - markerA.x;
                markerB.flowY = markerB.y - markerA.y;

                //std::cout << "Flow: " << markerB.flowX << " x " << markerB.flowY << std::endl;
            }
        }
    }

    /**
     * Estimates the correspondences to map the markers in frame a to the markers
     * in frame b.
     *
     * Note: This function uses the flow, so it's assumed that frame a was recorded
     * earlier than frame b.
     *
     * E.g. if result[0] is 4, than marker 0 in frame a is marker 4 in frame b.
     *
     * @return
     */
    static std::vector<int> estimateCorrespondences(CameraFrame& a, CameraFrame& b){
        if(a.markers.size() == 0)
            return std::vector<int>();

        std::vector<double> nearestDistanceBToA(b.markers.size(), 100000000);
        std::vector<int> nearestDistanceBToAIndex(b.markers.size(), -1);

        for(unsigned int aIdx = 0; aIdx < a.markers.size(); ++aIdx){
            ImageMarker& markerA = a.markers[aIdx];

            // The nearest marker b to the current
            int nearestMarkerBIdx = -1;
            double nearestSqrDist = 100000000;

            // Search the nearest markerB;
            for (unsigned int bIdx = 0; bIdx < b.markers.size(); ++bIdx) {
                ImageMarker markerB = b.markers[bIdx];
                double dX = (markerA.x + markerA.flowX) - markerB.x;
                double dY = (markerA.y + markerA.flowY) - markerB.y;
                double sqrDist = dX * dX + dY * dY;

                if(sqrDist < nearestSqrDist){
                    nearestSqrDist = sqrDist;
                    nearestMarkerBIdx = bIdx;
                }
            }

            if(nearestMarkerBIdx >= 0 && nearestDistanceBToA[nearestMarkerBIdx] > nearestSqrDist){
                nearestDistanceBToA[nearestMarkerBIdx] = nearestSqrDist;
                nearestDistanceBToAIndex[nearestMarkerBIdx] = aIdx;
            }
        }

        std::vector<int> result(a.markers.size(), -1);
        for (unsigned int bIdx = 0; bIdx < nearestDistanceBToAIndex.size(); ++bIdx) {
            int aIdx = nearestDistanceBToAIndex[bIdx];

            if(aIdx >= 0)
                result[aIdx] = bIdx;
        }

        return result;
    }
};
