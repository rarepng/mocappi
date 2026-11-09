#pragma once

#include <cstdio>
#include <fstream>

#include "src/processors/Processor.h"

/**
 * Writes the triangulated (unlabelled) 3D markers of every tick to
 * cam_markers.csv in the working directory, one row per tick:
 * Time, Count, then x, y and z of each marker in metres with 7 decimals
 **/
class MarkerLogger final : public Processor {
public:
    MarkerLogger(DataModel& model)
        : Processor(model) {}

    bool enabled = true;

    void process(TickData& tick) override {
        if(!enabled)
            return;

        if(!file.is_open()){
            file.open("cam_markers.csv");
            file << "Time,Count,x y z per marker (m)\n";
        }

        char value[64];
        std::snprintf(value, sizeof(value), "%.6f,%zu", tick.timestamp, tick.markers.size());
        file << value;
        for(const RawMarker& marker : tick.markers){
            std::snprintf(value, sizeof(value), ",%.7f,%.7f,%.7f", marker.position.x, marker.position.y, marker.position.z);
            file << value;
        }
        file << "\n";
    }

private:
    std::ofstream file;
};
