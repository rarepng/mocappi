#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "src/model/DataModel.h"
#include "src/processors/Processor.h"
#include "src/util/math/Utils.h"

/**
 * Simple rigid-body detector for an unlabeled point cloud.
 */

class RigidBodyDetector : public Processor {
public:
    // Maximum edge-length error (in meter).
    float pairTolerance = 0.015f;

    // Maximum distance (in meter).
    float inlierRadius = 0.02f;

    // Final pose acceptance criteria.
    int minimumInliers = 4;
    float maximumRms = 0.01f;
    float minimumSecondEigenvalueRatio = 0.01f;

    int minimumTrackingInliers = 3;

    float maximumLenientResidual = 0.001f;

    // Maximum allowed edge-length error between every pair of final inliers (meter):
    float finalPairTolerance = 0.002f;

    // Reject very small / almost collinear seed triangles:
    float minimumTripletArea = 0.0001f;

    // Keep only the geometrically strongest model triplets. Set <= 0 for all.
    int maximumModelTriplets = 64;

    // Recompute correspondences and pose this many times after a hypothesis.
    int refinementIterations = 2;

private:
    struct BodyState {
        Mat4f lastPose;
        bool foundLastFrame = false;
    };

    struct Match {
        int modelMarkerId = -1;
        int candidateId = -1;
        Vec4f source;
        Vec4f target;
        float residual = std::numeric_limits<float>::infinity();
    };

    struct Fit {
        Mat4f pose;
        std::vector<Match> inliers;
        float rms = std::numeric_limits<float>::infinity();
        float observability = 0.f;
        bool valid = false;
    };

    struct ModelTriplet {
        std::array<int, 3> markerIds;
        std::array<float, 3> distances; // 01, 02, 12
        float area = 0.f;
    };

    struct PairCandidate {
        int modelMarkerId = -1;
        int candidateId = -1;
        float residual = std::numeric_limits<float>::infinity();
    };

    std::unordered_map<RigidBody*, BodyState> states;
    std::unordered_map<RigidBody*, std::vector<ModelTriplet>> tripletCache;
    uint64_t frameNumber = 0;

    static Eigen::Vector3d toEigen(const Vec4f& point) {
        return Eigen::Vector3d(point.x, point.y, point.z);
    }

    static float tripletArea(const Vec4f& a, const Vec4f& b, const Vec4f& c) {
        const Eigen::Vector3d ab = toEigen(b) - toEigen(a);
        const Eigen::Vector3d ac = toEigen(c) - toEigen(a);
        return float(ab.cross(ac).norm());
    }

    static bool rigidTransform(const std::vector<Match>& matches, Mat4f& result) {
        if(matches.size() < 3)
            return false;

        Eigen::Vector3d sourceCenter = Eigen::Vector3d::Zero();
        Eigen::Vector3d targetCenter = Eigen::Vector3d::Zero();
        for(const Match& match : matches){
            sourceCenter += toEigen(match.source);
            targetCenter += toEigen(match.target);
        }
        sourceCenter /= double(matches.size());
        targetCenter /= double(matches.size());

        Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
        for(const Match& match : matches){
            const Eigen::Vector3d source = toEigen(match.source);
            const Eigen::Vector3d target = toEigen(match.target);
            covariance += (source - sourceCenter) * (target - targetCenter).transpose();
        }

        Eigen::JacobiSVD<Eigen::Matrix3d> svd(
            covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);

        if(svd.info() != Eigen::Success)
            return false;

        Eigen::Matrix3d rotation = svd.matrixV() * svd.matrixU().transpose();
        if(rotation.determinant() < 0.0){
            Eigen::Matrix3d v = svd.matrixV();
            v.col(2) *= -1.0;
            rotation = v * svd.matrixU().transpose();
        }

        const Eigen::Vector3d translation = targetCenter - rotation * sourceCenter;
        Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
        matrix.block<3,3>(0,0) = rotation.cast<float>();
        matrix.block<3,1>(0,3) = translation.cast<float>();

        if(!matrix.allFinite())
            return false;

        result = Mat4f::fromEigen(matrix);
        return result.isFinite();
    }

    static float observability(const std::vector<Match>& matches) {
        if(matches.size() < 3)
            return 0.f;

        Eigen::Vector3d center = Eigen::Vector3d::Zero();

        for(const Match& match : matches)
            center += toEigen(match.source);

        center /= double(matches.size());

        Eigen::Matrix3d scatter = Eigen::Matrix3d::Zero();
        for(const Match& match : matches){
            const Eigen::Vector3d delta = toEigen(match.source) - center;
            scatter += delta * delta.transpose();
        }

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(scatter);

        if(solver.info() != Eigen::Success)
            return 0.f;

        const Eigen::Vector3d values = solver.eigenvalues();

        return float(values[1] / std::max(values[2], 1e-12));
    }

    static float poseRms(const Mat4f& pose, const std::vector<Match>& matches) {
        if(matches.empty())
            return std::numeric_limits<float>::infinity();

        double squaredError = 0.0;
        for(const Match& match : matches){
            const float residual = (pose * match.source).distanceTo(match.target);
            squaredError += double(residual) * residual;
        }

        return float(std::sqrt(squaredError / matches.size()));
    }

    float edgeError(const Match& a, const Match& b) const {
        return std::abs(a.target.distanceTo(b.target) - a.source.distanceTo(b.source));
    }

    bool allEdgesCompatible(const std::vector<Match>& matches) const {
        for(size_t i = 0; i < matches.size(); ++i){
            for(size_t j = i + 1; j < matches.size(); ++j){
                if(edgeError(matches[i], matches[j]) > finalPairTolerance)
                    return false;
            }
        }
        return true;
    }

    int worstInlierByEdgeConsistency(const std::vector<Match>& matches) const {
        if(matches.size() < 2) return -1;

        std::vector<int> badEdgeCount(matches.size(), 0);
        std::vector<float> excessError(matches.size(), 0.f);

        for(size_t i = 0; i < matches.size(); ++i){
            for(size_t j = i + 1; j < matches.size(); ++j){
                const float error = edgeError(matches[i], matches[j]);

                if(error <= finalPairTolerance)
                    continue;

                const float excess = error - finalPairTolerance;
                ++badEdgeCount[i];
                ++badEdgeCount[j];
                excessError[i] += excess;
                excessError[j] += excess;
            }
        }

        int worst = -1;
        for(size_t i = 0; i < matches.size(); ++i){
            if(badEdgeCount[i] == 0) continue;
            if(worst < 0
                || badEdgeCount[i] > badEdgeCount[worst]
                || (badEdgeCount[i] == badEdgeCount[worst]
                    && excessError[i] > excessError[worst])) {
                worst = int(i);
            }
        }
        return worst;
    }

    bool pruneAndRefit(std::vector<Match>& matches, Mat4f& pose, size_t minimum) const {
        if(matches.size() < minimum || observability(matches) < minimumSecondEigenvalueRatio
            || !rigidTransform(matches, pose))
            return false;

        while(!allEdgesCompatible(matches)){
            if(matches.size() <= minimum)
                return false;

            const int worst = worstInlierByEdgeConsistency(matches);

            if(worst < 0)
                return false;

            matches.erase(matches.begin() + worst);

            if(matches.size() < minimum || observability(matches) < minimumSecondEigenvalueRatio
                || !rigidTransform(matches, pose))
                return false;
        }
        return true;
    }

    int requiredInliers(const RigidBody* rb) const {
        return std::max(3, std::min(int(rb->markers.size()), std::max(3, minimumInliers)));
    }

    std::vector<ModelTriplet> buildModelTriplets(RigidBody* rb) const {
        std::vector<ModelTriplet> result;
        const int count = int(rb->markers.size());

        for(int a = 0; a < count; ++a){
            for(int b = a + 1; b < count; ++b){
                for(int c = b + 1; c < count; ++c){
                    const Vec4f& pa = rb->markers[a].position;
                    const Vec4f& pb = rb->markers[b].position;
                    const Vec4f& pc = rb->markers[c].position;
                    const float area = tripletArea(pa, pb, pc);

                    if(area < minimumTripletArea)
                        continue;

                    ModelTriplet triplet;
                    triplet.markerIds = {{a, b, c}};
                    triplet.distances = {{pa.distanceTo(pb), pa.distanceTo(pc), pb.distanceTo(pc)
                    }};

                    triplet.area = area;
                    result.push_back(triplet);
                }
            }
        }

        std::sort(result.begin(), result.end(), [](const ModelTriplet& lhs, const ModelTriplet& rhs) {
            return lhs.area > rhs.area;
        });

        if(maximumModelTriplets > 0 && int(result.size()) > maximumModelTriplets)
            result.resize(size_t(maximumModelTriplets));

        return result;
    }

    const std::vector<ModelTriplet>& modelTriplets(RigidBody* rb) {
        std::unordered_map<RigidBody*, std::vector<ModelTriplet>>::iterator it = tripletCache.find(rb);

        if(it != tripletCache.end())
            return it->second;

        return tripletCache.insert(std::make_pair(rb, buildModelTriplets(rb))).first->second;
    }

    std::vector<Match> assignInliers(RigidBody* rb, const Mat4f& pose, const std::vector<int>& candidates) const {
        std::vector<PairCandidate> possible;

        possible.reserve(rb->markers.size() * candidates.size());

        for(int modelMarkerId = 0; modelMarkerId < rb->markers.size(); ++modelMarkerId) {
            Vec4f expected = pose * rb->markers[modelMarkerId].position;

            for(int candidateId : candidates){
                const Vec4f& observed = model.markers[candidateId].position;
                const float residual = expected.distanceTo(observed);
                if(residual <= inlierRadius){
                    PairCandidate pair;
                    pair.modelMarkerId = modelMarkerId;
                    pair.candidateId = candidateId;
                    pair.residual = residual;
                    possible.push_back(pair);
                }
            }
        }

        std::sort(possible.begin(), possible.end(), [](const PairCandidate& lhs, const PairCandidate& rhs) {
            return lhs.residual < rhs.residual;
        });

        std::vector<bool> modelUsed(rb->markers.size(), false);
        std::vector<bool> candidateUsed(model.markers.size(), false);
        std::vector<Match> matches;

        matches.reserve(std::min(rb->markers.size(), candidates.size()));

        for(const PairCandidate& pair : possible){
            if(modelUsed[pair.modelMarkerId] || candidateUsed[pair.candidateId])
                continue;

            modelUsed[pair.modelMarkerId] = true;
            candidateUsed[pair.candidateId] = true;

            Match match;
            match.modelMarkerId = pair.modelMarkerId;
            match.candidateId = pair.candidateId;
            match.source = rb->markers[pair.modelMarkerId].position;
            match.target = model.markers[pair.candidateId].position;
            match.residual = pair.residual;
            matches.push_back(match);
        }
        return matches;
    }

    Fit evaluatePose(RigidBody* rb, const Mat4f& initialPose, const std::vector<int>& candidates, int minimumMatches) const {
        
        Fit result;

        const size_t minimum = size_t(std::max(3, minimumMatches));

        if(candidates.size() < minimum)
            return result;

        if(!initialPose.isFinite())
            return result;

        Mat4f pose = initialPose;
        std::vector<Match> matches;

        const int iterations = std::max(1, refinementIterations);

        for(int iteration = 0; iteration < iterations; ++iteration){
            matches = assignInliers(rb, pose, candidates);

            if(matches.size() < minimum)
                return Fit{};

            if(observability(matches) < minimumSecondEigenvalueRatio)
                return Fit{};

            if(!rigidTransform(matches, pose))
                return Fit{};
        }

        matches = assignInliers(rb, pose, candidates);
        if(matches.size() < minimum)
            return Fit{};

        result.observability = observability(matches);
        if(result.observability < minimumSecondEigenvalueRatio)
            return Fit{};

        if(!rigidTransform(matches, result.pose)) return Fit{};

        matches = assignInliers(rb, result.pose, candidates);
        if(matches.size() < minimum)
            return Fit{};

        if(!pruneAndRefit(matches, result.pose, minimum))
            return Fit{};

        result.observability = observability(matches);
        if(result.observability < minimumSecondEigenvalueRatio)
            return Fit{};

        result.rms = poseRms(result.pose, matches);
        if(!std::isfinite(result.rms) || result.rms > maximumRms)
            return Fit{};

        result.inliers = std::move(matches);
        result.valid = true;
        return result;
    }

    static bool betterFit(const Fit& candidate, const Fit& current) {
        if(!candidate.valid)
            return false;

        if(!current.valid)
            return true;

        if(candidate.inliers.size() != current.inliers.size())
            return candidate.inliers.size() > current.inliers.size();

        if(candidate.rms != current.rms)
            return candidate.rms < current.rms;

        return candidate.observability > current.observability;
    }

    bool compatibleTriplet(const ModelTriplet& modelTriplet, const std::array<Vec4f, 3>& observed, const std::array<int, 3>& permutation) const {
        const Vec4f& a = observed[permutation[0]];
        const Vec4f& b = observed[permutation[1]];
        const Vec4f& c = observed[permutation[2]];

        return std::abs(a.distanceTo(b) - modelTriplet.distances[0]) <= pairTolerance
               && std::abs(a.distanceTo(c) - modelTriplet.distances[1]) <= pairTolerance
               && std::abs(b.distanceTo(c) - modelTriplet.distances[2]) <= pairTolerance;
    }

    Fit globalTripletSearch(RigidBody* rb, const std::vector<int>& candidates) {
        Fit best;
        const std::vector<ModelTriplet>& triplets = modelTriplets(rb);

        if(triplets.empty() || candidates.size() < 3)
            return best;

        static const std::array<std::array<int, 3>, 6> permutations = {{
            {{0, 1, 2}}, {{0, 2, 1}}, {{1, 0, 2}},
            {{1, 2, 0}}, {{2, 0, 1}}, {{2, 1, 0}}
        }};

        for(size_t oa = 0; oa + 2 < candidates.size(); ++oa){
            for(size_t ob = oa + 1; ob + 1 < candidates.size(); ++ob){
                for(size_t oc = ob + 1; oc < candidates.size(); ++oc){

                    const std::array<int, 3> observedIds = {{
                        candidates[oa], candidates[ob], candidates[oc]
                    }};

                    const std::array<Vec4f, 3> observed = {{
                        model.markers[observedIds[0]].position,
                        model.markers[observedIds[1]].position,
                        model.markers[observedIds[2]].position
                    }};

                    if(tripletArea(observed[0], observed[1], observed[2]) < minimumTripletArea) continue;

                    for(const ModelTriplet& modelTriplet : triplets){
                        for(const std::array<int, 3>& permutation : permutations){
                            if(!compatibleTriplet(modelTriplet, observed, permutation))
                                continue;

                            std::vector<Match> seed(3);
                            for(int i = 0; i < 3; ++i){
                                seed[i].modelMarkerId = modelTriplet.markerIds[i];
                                seed[i].candidateId = observedIds[permutation[i]];
                                seed[i].source = rb->markers[seed[i].modelMarkerId].position;
                                seed[i].target = model.markers[seed[i].candidateId].position;
                            }

                            Mat4f hypothesis;
                            if(!rigidTransform(seed, hypothesis))
                                continue;

                            Fit fit = evaluatePose(rb, hypothesis, candidates, requiredInliers(rb));

                            if(betterFit(fit, best))
                                best = std::move(fit);

                            if(best.valid && best.inliers.size() == rb->markers.size())
                                return best;
                        }
                    }
                }
            }
        }

        return best;
    }


    void removeInactiveStates(const TickData& tick) {
        for(std::unordered_map<RigidBody*, BodyState>::iterator it = states.begin(); it != states.end(); ) {

            auto fnd = std::find(tick.allActiveRigidBodies.begin(), tick.allActiveRigidBodies.end(), it->first);
            if(fnd == tick.allActiveRigidBodies.end()) {
                tripletCache.erase(it->first);
                it = states.erase(it);
            } else {
                ++it;
            }
        }
    }

    bool tightenLenientFit(Fit& fit, size_t minimum) const {
        while(true){
            size_t worst = 0;
            float worstResidual = 0.f;
            for(size_t i = 0; i < fit.inliers.size(); ++i){
                const float residual = (fit.pose * fit.inliers[i].source).distanceTo(fit.inliers[i].target);
                if(residual > worstResidual){ worstResidual = residual; worst = i; }
            }
            if(worstResidual <= maximumLenientResidual)
                return true;
            if(fit.inliers.size() <= minimum)
                return false;
            fit.inliers.erase(fit.inliers.begin() + worst);
            fit.observability = observability(fit.inliers);
            if(fit.observability < minimumSecondEigenvalueRatio || !rigidTransform(fit.inliers, fit.pose))
                return false;
            fit.rms = poseRms(fit.pose, fit.inliers);
        }
    }

    void updateBody(RigidBody* rb, std::vector<bool>& usedCandidates) {
        std::vector<int> candidates;
        candidates.reserve(model.markers.size());

        for(size_t i = 0; i < model.markers.size(); ++i){
            if(!usedCandidates[i] && model.markers[i].obsoleteCounter == 0)
                candidates.push_back(i);
        }

        BodyState& state = states[rb];
        Fit fit;

        if(state.foundLastFrame){
            const int trackingInliers = std::min(requiredInliers(rb), std::max(3, minimumTrackingInliers));
            fit = evaluatePose(rb, state.lastPose, candidates, trackingInliers);

            if(fit.valid && trackingInliers < requiredInliers(rb) && !tightenLenientFit(fit, size_t(trackingInliers)))
                fit = Fit{};
        }

        if(!fit.valid){
            fit = globalTripletSearch(rb, candidates);
        }

        if(!fit.valid){
            state.foundLastFrame = false;
            rb->bodyFound = false;
            ++rb->notSeenSince;
            rb->inlierCount = 0;

            return;
        }

        rb->pose = fit.pose;
        rb->bodyFound = true;
        rb->notSeenSince = 0;
        rb->inlierCount = int(fit.inliers.size());
        state.lastPose = fit.pose;
        state.foundLastFrame = true;

        for(const Match& match : fit.inliers){
            if(match.candidateId >= 0 && match.candidateId < int(usedCandidates.size())) {
                usedCandidates[match.candidateId] = true;
            }
        }
    }

public:
    explicit RigidBodyDetector(DataModel& model)
        : Processor(model) {}

    ~RigidBodyDetector(){
        if(rbFileStream.is_open()){
            rbFileStream.close();
        }
    }

    bool shouldWriteRBs = false;
    std::ofstream rbFileStream;

    void invalidateRigidBody(RigidBody* rb) {
        tripletCache.erase(rb);
        states.erase(rb);
    }

    void process(TickData& tick) override {
        ++frameNumber;
        removeInactiveStates(tick);

        std::vector<RigidBody*> bodies = tick.allActiveRigidBodies;
        std::stable_sort(bodies.begin(), bodies.end(), [this](RigidBody* lhs, RigidBody* rhs) {
            const std::unordered_map<RigidBody*, BodyState>::const_iterator lhsIt = states.find(lhs);
            const std::unordered_map<RigidBody*, BodyState>::const_iterator rhsIt = states.find(rhs);
            const bool lhsTracked = lhsIt != states.end() && lhsIt->second.foundLastFrame;
            const bool rhsTracked = rhsIt != states.end() && rhsIt->second.foundLastFrame;
            return lhsTracked && !rhsTracked;
        });

        std::vector<bool> usedCandidates(model.markers.size(), false);
        for(RigidBody* rb : bodies){
            updateBody(rb, usedCandidates);

            if(rb->bodyFound && !shouldWriteRBs){
                shouldWriteRBs = true;

                if(!rbFileStream.is_open()){
                    rbFileStream.open("cam_rigidBodies.csv");
                    rbFileStream << "Time,RigidBody,Found,Position,Quaternion" << "\n";
                }
            }

            if(shouldWriteRBs){
                rbFileStream << tick.timestamp << "," << rb->label << "," << rb->bodyFound << "," << rb->pose.getPosition() << "," << rb->pose.getQuaternion() << "\n";
            }
        }
    }
};
