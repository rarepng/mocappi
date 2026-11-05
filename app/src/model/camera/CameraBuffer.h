#pragma once

#include "src/model/camera/CameraFrame.h"

#include "src/util/Semaphore.h"

#include <list>

/**
 * Manages a set of the last N recorded frames for a camera.
 *
 * This set is automatically ordered by the timestamp.
 */
struct CameraBuffer {
    /**
     * Maximum number of frames allowed to be in this list.
     */
    unsigned int maxFrames;

    /**
     * Semaphore to avoid concurrency problems when accessing
     * or inserting frames.
     */
    Semaphore semaphore;

    /**
     * Raw frames.
     */
    std::list<CameraFrame> frames;

    /**
     * Creates a frame set.
     */
    CameraBuffer(unsigned int maxFrames)
        : maxFrames(maxFrames)
        , semaphore(1)
    {}

    /**
     * Inserts the given raw frame at the correct place in this
     * list, sorted by the hostTimestamp.
     *
     * If the raw frameset would exceed the maximum number of frames,
     * the oldest frame (with the lowest timestamp) is removed.
     */
    void insertFrame(CameraFrame& frame){
        // We use the reverse iterator (rbegin, rend), because we
        // normally expect the timestamp to be bigger, so using this
        // direction is usually much faster:
        for (auto rit = frames.rbegin(); rit != frames.rend(); ++rit) {

            // As soon as the host timestamp of the current item is smaller,
            // we put the new frame behind it:
            if (rit->timestamp < frame.timestamp) {
                // Calculate the flow based on the previous frame (this is
                // not completely save, because it doesn't consider missing
                // frames):
                CameraFrame::estimateFlow(*rit, frame);

                // Insert the frame after the 'rit'-frame:
                frames.insert(rit.base(), frame);

                // If the list size is now bigger, we have to remove the
                // oldest item:
                if(frames.size() > maxFrames)
                    frames.pop_front();

                return;
            }
        }

        // If the frame was not inserted, we have to add it to the front of
        // the list (but only if the list is not already full):
        if(frames.size() < maxFrames){
            frames.push_front(frame);
        }
    }

    /**
     * Returns an interpolated frame for the given timestamp. If no interpolated
     * frame can be created, an invalid InterpFrame will be returned.
     */
    CameraFrame getInterpFrameAt(double desiredTimestamp){
        // The frame that is newer as the current found one:
        CameraFrame* newerFrame = nullptr;

        // We expect the desired interpolated frame to be very up-to-date in our
        // scenario, so we search from the end for performance reasons:
        for (auto rit = frames.rbegin(); rit != frames.rend(); ++rit) {

            // If the timestamp of the current frame is smaller or equals the
            // searched one, we have to interpolate it with the newer frame:
            if (rit->timestamp < desiredTimestamp) {
                // If there is no newer frame, there is no data that is up-to-date
                // enough to create the InterpFrame and we return an invalid one:
                if(newerFrame == nullptr)
                    return CameraFrame();

                // Return a blended frame of the current (older) frame and the newer
                // one:
                return CameraFrame(*rit, *newerFrame, desiredTimestamp);
            }

            // Temporally store the newer frame:
            newerFrame = &(*rit);
        }

        // If no host timestamp was smaller than the desired timestamp, all frames
        // lie behind the desired timestamp and we cannot interpolate one:
        return CameraFrame();
    }

    // returns ground truth as (camera ID 999) blended with the 2 frames around it
    std::vector<Vec4f> getGroundTruthAt(double desiredTimestamp) const {
        const CameraFrame* newerFrame = nullptr;
        for (auto rit = frames.rbegin(); rit != frames.rend(); ++rit) {
            if (rit->timestamp < desiredTimestamp) {
                if (newerFrame == nullptr || newerFrame->markers.size() != rit->markers.size())
                    return {};
                const double alpha = (desiredTimestamp - rit->timestamp) / (newerFrame->timestamp - rit->timestamp);
                std::vector<Vec4f> positions;
                positions.reserve(rit->markers.size());
                for (size_t i = 0; i < rit->markers.size(); ++i) {
                    const ImageMarker& a = rit->markers[i];
                    const ImageMarker& b = newerFrame->markers[i];
                    positions.push_back(Vec4f(float(a.x + (b.x - a.x) * alpha),
                                              float(a.y + (b.y - a.y) * alpha),
                                              float(a.pixels + (b.pixels - a.pixels) * alpha), 1.0f));
                }
                return positions;
            }
            newerFrame = &(*rit);
        }
        return {};
    }
};