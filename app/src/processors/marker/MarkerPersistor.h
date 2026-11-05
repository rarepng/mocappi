#pragma once

#include <algorithm>
#include <vector>
#include "src/model/DataModel.h"
#include "src/util/math/Vec4.h"

#include "src/processors/Processor.h"

class MarkerPersistor : public Processor{

public:
    MarkerPersistor(DataModel& model)
        : Processor(model){}

    float associationRadius = 0.01f; // in meters

    int selectionHoldTicks = 12; // 12 ticks = ~100 ms at 120hz

    virtual void process(TickData& tick) override {
        std::vector<Marker> markers;
        markers.reserve(tick.markers.size());
        for(RawMarker& m : tick.markers){
            markers.push_back(m);
            markers.back().obsoleteCounter = 0;
            markers.back().isSelected = false;
        }

        std::vector<Track> sources;
        sources.reserve(model.markers.size() + heldSelections.size());
        for(size_t p = 0; p < model.markers.size(); ++p){
            const Vec4f velocity = p < lastVelocities.size() ? lastVelocities[p] : Vec4f(0.f, 0.f, 0.f, 0.f);
            sources.push_back({model.markers[p].position, velocity, model.markers[p].isSelected, 0});
        }
        sources.insert(sources.end(), heldSelections.begin(), heldSelections.end());
        heldSelections.clear();

        struct Pair { float distance; size_t source; size_t current; };
        std::vector<Pair> pairs;
        for(size_t s = 0; s < sources.size(); ++s){
            const Vec4f predicted = predict(sources[s]);
            for(size_t c = 0; c < markers.size(); ++c){
                const float distance = predicted.distanceTo(markers[c].position);
                if(distance <= associationRadius)
                    pairs.push_back({distance, s, c});
            }
        }
        std::sort(pairs.begin(), pairs.end(), [](const Pair& a, const Pair& b){ return a.distance < b.distance; });

        lastVelocities.assign(markers.size(), Vec4f(0.f, 0.f, 0.f, 0.f));
        std::vector<bool> sourceUsed(sources.size(), false);
        std::vector<bool> currentUsed(markers.size(), false);
        for(const Pair& pair : pairs){
            if(sourceUsed[pair.source] || currentUsed[pair.current])
                continue;
            sourceUsed[pair.source] = true;
            currentUsed[pair.current] = true;
            markers[pair.current].isSelected = sources[pair.source].selected;

            const Vec4f& from = sources[pair.source].position;
            const Vec4f& to = markers[pair.current].position;
            const float steps = float(sources[pair.source].missedTicks + 1);
            lastVelocities[pair.current] = Vec4f((to.x - from.x) / steps, (to.y - from.y) / steps, (to.z - from.z) / steps, 0.f);
        }

        for(size_t s = 0; s < sources.size(); ++s){
            if(!sourceUsed[s] && sources[s].selected && sources[s].missedTicks < selectionHoldTicks)
                heldSelections.push_back({sources[s].position, sources[s].velocity, true, sources[s].missedTicks + 1});
        }

        model.markers = markers;
    }

private:
    struct Track {
        Vec4f position;
        Vec4f velocity;
        bool selected;
        int missedTicks;
    };

    static Vec4f predict(const Track& track){
        const float steps = float(track.missedTicks + 1);
        return Vec4f(track.position.x + track.velocity.x * steps,
                     track.position.y + track.velocity.y * steps,
                     track.position.z + track.velocity.z * steps,
                     track.position.w);
    }

    std::vector<Track> heldSelections;
    std::vector<Vec4f> lastVelocities;
};
