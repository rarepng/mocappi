#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <fstream>
#include <numbers>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "src/processors/Processor.h"

/**
 * Compares every tracked rigid body with the ground truth of the current tick
 * (camera ID 999)
 * writes one row per body.
 * and tick to cam_groundTruthErrors.csv in the working directory. 
 * only bodies with exactly as many markers as the ground truth are evaluated.
 * the tip is the body origin.
 **/
class GroundTruthEvaluator : public Processor {
public:
    GroundTruthEvaluator(DataModel& model)
        : Processor(model) {}

    void process(TickData& tick) override {
        const size_t count = tick.markersGTPos.size();

        for(auto it = history.begin(); it != history.end();){
            const bool active = std::find(tick.allActiveRigidBodies.begin(), tick.allActiveRigidBodies.end(), it->first) != tick.allActiveRigidBodies.end();
            it = active ? std::next(it) : history.erase(it);
        }
        for(RigidBody* rb : tick.allActiveRigidBodies){
            rb->groundTruthTipError = -1.f;
            if(count < 3 || rb->markers.size() != count){
                rb->groundTruthTipErrorP99 = -1.f;
                history.erase(rb);
            }
        }

        if(count < 3)
            return;

        for(RigidBody* rb : tick.allActiveRigidBodies){
            if(rb->markers.size() != count)
                continue;

            Eigen::Matrix3Xd body(3, count), truth(3, count);
            for(size_t i = 0; i < count; ++i){
                const Vec4f& m = rb->markers[i].position;
                const Vec4f& g = tick.markersGTPos[i];
                body.col(i) = Eigen::Vector3d(m.x, m.y, m.z);
                truth.col(i) = Eigen::Vector3d(g.x, g.y, g.z);
            }
            const Eigen::Matrix4d truePose = Eigen::umeyama(body, truth, false);

            if(!file.is_open()){
                file.open("cam_groundTruthErrors.csv");
                file << "Time,Found,TipError_mm,CentroidError_mm,RotationError_deg,Inliers,WrongIdentity,RigidBody\n";
            }

            char row[160];
            if(!rb->bodyFound){
                std::snprintf(row, sizeof(row), "%.6f,0,,,,0,0,", tick.timestamp);
                file << row << rb->label << "\n";
                continue;
            }

            const Eigen::Matrix4d estimatedPose = Mat4f::toEigen(rb->pose).cast<double>();
            const Eigen::Matrix3d estimatedRotation = estimatedPose.block<3, 3>(0, 0);
            const Eigen::Matrix3d trueRotation = truePose.block<3, 3>(0, 0);
            const Eigen::Vector3d translationDelta = estimatedPose.block<3, 1>(0, 3) - truePose.block<3, 1>(0, 3);
            const Eigen::Vector3d centroid = body.rowwise().mean();

            const double tipError = translationDelta.norm();
            const double centroidError = ((estimatedRotation - trueRotation) * centroid + translationDelta).norm();

            const Eigen::Matrix3d delta = estimatedRotation.transpose() * trueRotation;
            const double sine = 0.5 * Eigen::Vector3d(delta(2, 1) - delta(1, 2), delta(0, 2) - delta(2, 0), delta(1, 0) - delta(0, 1)).norm();
            const double rotationError = std::atan2(sine, (delta.trace() - 1.0) / 2.0) * 180.0 / std::numbers::pi;

            bool wrongIdentity = false;
            for(size_t i = 0; i < count && !wrongIdentity; ++i){
                const Eigen::Vector3d estimated = estimatedRotation * body.col(i) + estimatedPose.block<3, 1>(0, 3);
                Eigen::Index nearest = 0;
                (truth.colwise() - estimated).colwise().squaredNorm().minCoeff(&nearest);
                wrongIdentity = size_t(nearest) != i;
            }

            std::deque<float>& recent = history[rb];
            recent.push_back(float(tipError * 1000.0));
            if(recent.size() > 1200)
                recent.pop_front();
            std::vector<float> sorted(recent.begin(), recent.end());
            const size_t rank = (sorted.size() * 99 + 99) / 100;
            std::nth_element(sorted.begin(), sorted.begin() + (rank - 1), sorted.end());
            rb->groundTruthTipError = float(tipError * 1000.0);
            rb->groundTruthTipErrorP99 = sorted[rank - 1];

            std::snprintf(row, sizeof(row), "%.6f,1,%.4f,%.4f,%.4f,%d,%d,", tick.timestamp,
                          tipError * 1000.0, centroidError * 1000.0, rotationError, rb->inlierCount, wrongIdentity ? 1 : 0);
            file << row << rb->label << "\n";
        }
    }

private:
    std::ofstream file;
    std::unordered_map<RigidBody*, std::deque<float>> history;
};
