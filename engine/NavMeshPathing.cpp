#include "NavMesh.h"
#include "NavMeshInternal.h"
#include "NavPriorDB.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <queue>
#include <unordered_map>

using namespace navmesh_internal;

namespace {

double ComputePortalAwareStepCost(int prevPoly,
                                  int currentPoly,
                                  int neighborPoly,
                                  int goalPoly,
                                  const std::vector<Vec2>& centroids,
                                  int clearanceBucket) {
    double stepCost = Distance(centroids[currentPoly], centroids[neighborPoly]);
    const float priorBonus =
        NavPriorDB::Instance().GetTransitionBonus(goalPoly, currentPoly, neighborPoly, clearanceBucket);
    const float portalHint =
        NavPriorDB::Instance().GetPortalLengthHint(currentPoly, neighborPoly, clearanceBucket);

    stepCost -= static_cast<double>(priorBonus);
    if (portalHint > 0.0f) {
        stepCost += 20.0 / static_cast<double>(portalHint);
    }

    if (prevPoly >= 0 && prevPoly != currentPoly) {
        const double angle = AngleBetweenDirections(centroids[prevPoly],
                                                    centroids[currentPoly],
                                                    centroids[neighborPoly]);
        stepCost += angle * 55.0;
    }

    return std::max(1.0, stepCost);
}

} // namespace

std::vector<int> NavMesh::FindPath(int startIndex, int goalIndex, int clearanceBucket) {
    std::vector<int> result;

    if (startIndex < 0 || goalIndex < 0 ||
        startIndex >= static_cast<int>(polygons.size()) ||
        goalIndex >= static_cast<int>(polygons.size()) ||
        !IsPolygonEnabled(startIndex) || !IsPolygonEnabled(goalIndex)) {
        return result;
    }

    if (startIndex == goalIndex) {
        result.push_back(startIndex);
        return result;
    }

    const int quantized = NavMesh::QuantizeClearanceBucket(clearanceBucket);
    std::shared_ptr<const std::vector<BucketView>> bucketSnapshot;
    const BucketView* bucketView = (quantized > 0) ? GetBucketView(quantized, bucketSnapshot) : nullptr;

    const std::vector<Vec2>& centroids = polygonCentroids;

    auto heuristic = [&](int idx) -> double {
        return Distance(centroids[idx], centroids[goalIndex]);
    };

    std::priority_queue<SearchNode, std::vector<SearchNode>, std::greater<SearchNode>> open;
    std::unordered_map<PolyState, double, PolyStateHash> gScore;
    std::unordered_map<PolyState, PolyState, PolyStateHash> cameFrom;

    const PolyState startState{-1, startIndex};
    gScore[startState] = 0.0;
    open.push({startState, 0.0, heuristic(startIndex)});

    PolyState goalState;
    bool found = false;

    while (!open.empty()) {
        const SearchNode current = open.top();
        open.pop();

        auto scoreIt = gScore.find(current.state);
        if (scoreIt == gScore.end() || current.g > scoreIt->second) {
            continue;
        }

        if (current.state.curPoly == goalIndex) {
            goalState = current.state;
            found = true;
            break;
        }

        const std::vector<int>* neighborList = nullptr;
        if (bucketView &&
            current.state.curPoly >= 0 &&
            current.state.curPoly < static_cast<int>(bucketView->traversableNeighbors.size())) {
            neighborList = &bucketView->traversableNeighbors[current.state.curPoly];
        }

        if (neighborList) {
            for (int neighbor : *neighborList) {
                const double edgeCost = ComputePortalAwareStepCost(
                    current.state.prevPoly,
                    current.state.curPoly,
                    neighbor,
                    goalIndex,
                    centroids,
                    quantized
                );

                const double tentativeG = current.g + edgeCost;
                const PolyState nextState{current.state.curPoly, neighbor};

                auto nextIt = gScore.find(nextState);
                if (nextIt == gScore.end() || tentativeG < nextIt->second) {
                    gScore[nextState] = tentativeG;
                    cameFrom[nextState] = current.state;
                    open.push({nextState, tentativeG, tentativeG + heuristic(neighbor)});
                }
            }
            continue;
        }

        for (int neighbor : polygons[current.state.curPoly].neighborIndices) {
            if (neighbor < 0 || neighbor >= static_cast<int>(polygons.size()) ||
                !IsPolygonEnabled(neighbor)) {
                continue;
            }

            if (quantized > 0) {
                Vec2 sharedA{};
                Vec2 sharedB{};
                if (!TryGetSharedPortal(current.state.curPoly, neighbor, sharedA, sharedB)) {
                    continue;
                }

                if (!IsPortalTraversableForClearance(sharedA, sharedB, quantized)) {
                    continue;
                }
            }

            const double edgeCost = ComputePortalAwareStepCost(
                current.state.prevPoly,
                current.state.curPoly,
                neighbor,
                goalIndex,
                centroids,
                quantized
            );

            const double tentativeG = current.g + edgeCost;
            const PolyState nextState{current.state.curPoly, neighbor};

            auto nextIt = gScore.find(nextState);
            if (nextIt == gScore.end() || tentativeG < nextIt->second) {
                gScore[nextState] = tentativeG;
                cameFrom[nextState] = current.state;
                open.push({nextState, tentativeG, tentativeG + heuristic(neighbor)});
            }
        }
    }

    if (!found) {
        return result;
    }

    std::vector<int> reversed;
    PolyState cur = goalState;
    reversed.push_back(cur.curPoly);

    while (!(cur.prevPoly == -1 && cur.curPoly == startIndex)) {
        auto it = cameFrom.find(cur);
        if (it == cameFrom.end()) {
            break;
        }
        cur = it->second;
        reversed.push_back(cur.curPoly);
    }

    std::reverse(reversed.begin(), reversed.end());

    for (int poly : reversed) {
        if (result.empty() || result.back() != poly) {
            result.push_back(poly);
        }
    }

    return result;
}

std::vector<Vec2> NavMesh::FunnelPath(const std::vector<int>& polyPath,
                                      const Vec2& startPt,
                                      const Vec2& goalPt,
                                      int clearanceBucket) {
    if (polyPath.empty() || polygons.empty()) {
        return {};
    }

    const int quantized = NavMesh::QuantizeClearanceBucket(clearanceBucket);

    const Vec2 clampedStart = ClampToNavMesh(startPt, quantized);
    const Vec2 clampedGoal = ClampToNavMesh(goalPt, quantized);

    const std::vector<Portal> portals =
        BuildPortals(polyPath, polygons, clampedStart, clampedGoal);
    if (portals.empty()) {
        return {};
    }

    std::vector<Vec2> path;
    if (quantized > 0) {
        path = BuildClearanceAwareCorridorPath(portals, polyPath, quantized);
    } else {
        path = BuildSampledCorridorPath(portals, polyPath, polygons);
    }

    if (path.empty()) {
        return {};
    }

    if (quantized > 0) {
        for (size_t i = 1; i < path.size(); ++i) {
            if (!IsSegmentClearOfWalls(path[i - 1], path[i], quantized)) {
                return {};
            }
        }
    }

    if (IsPathInsideCorridor(path, polyPath, polygons)) {
        return path;
    }

    return {};
}

bool NavMesh::TryGetSharedPortal(int polyAIndex,
                                 int polyBIndex,
                                 Vec2& outA,
                                 Vec2& outB) const {
    auto it = sharedPortals.find(MakePortalKey(polyAIndex, polyBIndex));
    if (it == sharedPortals.end()) {
        return false;
    }
    outA = it->second.first;
    outB = it->second.second;
    return true;
}

bool NavMesh::IsPortalTraversableForClearance(const Vec2& a,
                                              const Vec2& b,
                                              int clearanceBucket) const {
    const int quantized = NavMesh::QuantizeClearanceBucket(clearanceBucket);
    if (quantized <= 0) {
        return true;
    }

    const Portal portal{a, b};
    const Portal inset = InsetPortal(portal, static_cast<double>(quantized));

    const double dx = static_cast<double>(inset.right.x - inset.left.x);
    const double dy = static_cast<double>(inset.right.y - inset.left.y);
    const double len = std::sqrt(dx * dx + dy * dy);

    if (len <= 1.0) {
        return IsPointClearOfWalls(inset.left, quantized);
    }

    static const double TS[] = {0.0, 0.25, 0.5, 0.75, 1.0};
    for (double t : TS) {
        const Vec2 p = {
            static_cast<int>(std::lround(inset.left.x + dx * t)),
            static_cast<int>(std::lround(inset.left.y + dy * t))
        };

        if (IsPointClearOfWalls(p, quantized)) {
            return true;
        }
    }

    return false;
}

std::vector<Vec2> NavMesh::BuildClearanceAwareCorridorPath(
    const std::vector<Portal>& portals,
    const std::vector<int>& polyPath,
    int clearanceBucket) const {
    std::vector<Vec2> empty;
    if (portals.empty()) {
        return empty;
    }

    const int quantized = NavMesh::QuantizeClearanceBucket(clearanceBucket);
    std::shared_ptr<const std::vector<BucketView>> bucketSnapshot;
    const BucketView* bucketView = GetBucketView(quantized, bucketSnapshot);

    auto buildLayerSamples = [&](const Portal& portal, bool fixedPoint) -> std::vector<PortalSample> {
        std::vector<PortalSample> samples;

        if (fixedPoint || (portal.left.x == portal.right.x && portal.left.y == portal.right.y)) {
            if (quantized <= 0 || IsPointClearOfWalls(portal.left, quantized)) {
                samples.push_back({portal.left, 0.0});
            }
            return samples;
        }

        const Portal inset = InsetPortal(portal, static_cast<double>(quantized) + 4.0);
        const double dx = static_cast<double>(inset.right.x - inset.left.x);
        const double dy = static_cast<double>(inset.right.y - inset.left.y);
        const double len = std::sqrt(dx * dx + dy * dy);

        if (len < 6.0) {
            const Vec2 mid = PortalMidpoint(inset);
            if (IsPointClearOfWalls(mid, quantized)) {
                samples.push_back({mid, 0.0});
            }
            return samples;
        }

        static const double TS[] = {0.0, 0.25, 0.5, 0.75, 1.0};
        for (double t : TS) {
            const Vec2 p = {
                static_cast<int>(std::lround(inset.left.x + dx * t)),
                static_cast<int>(std::lround(inset.left.y + dy * t))
            };

            if (!IsPointClearOfWalls(p, quantized)) {
                continue;
            }

            bool duplicate = false;
            for (const auto& existing : samples) {
                if (existing.point.x == p.x && existing.point.y == p.y) {
                    duplicate = true;
                    break;
                }
            }
            if (duplicate) {
                continue;
            }

            const double midpointBias = std::abs(t - 0.5) * 8.0;
            samples.push_back({p, midpointBias});
        }

        return samples;
    };

    auto simplifyWithClearance = [&](const std::vector<Vec2>& path) -> std::vector<Vec2> {
        if (path.size() <= 2) {
            return path;
        }

        std::vector<Vec2> simplified;
        simplified.push_back(path.front());

        size_t i = 0;
        while (i < path.size() - 1) {
            size_t best = i + 1;

            for (size_t j = path.size() - 1; j > i + 1; --j) {
                if (!IsSegmentInsideCorridorSampled(simplified.back(), path[j], polyPath, polygons)) {
                    continue;
                }
                if (!IsSegmentClearOfWalls(simplified.back(), path[j], quantized)) {
                    continue;
                }

                best = j;
                break;
            }

            simplified.push_back(path[best]);
            i = best;
        }

        return simplified;
    };

    std::vector<std::vector<PortalSample>> layers;
    layers.reserve(portals.size());

    for (size_t i = 0; i < portals.size(); ++i) {
        const bool fixedPoint = (i == 0 || i == portals.size() - 1);

        if (!fixedPoint &&
            bucketView &&
            i - 1 < polyPath.size()) {
            const uint64_t key = MakePortalKey(polyPath[i - 1], polyPath[i]);
            auto it = bucketView->portalData.find(key);
            if (it != bucketView->portalData.end() && it->second.valid && !it->second.samples.empty()) {
                std::vector<PortalSample> precomputed;
                precomputed.reserve(it->second.samples.size());
                for (const auto& sample : it->second.samples) {
                    precomputed.push_back({sample.point, sample.landingPenalty});
                }
                layers.push_back(std::move(precomputed));
                continue;
            }
        }

        layers.push_back(buildLayerSamples(portals[i], fixedPoint));
        if (layers.back().empty()) {
            return empty;
        }
    }

    const int layerCount = static_cast<int>(layers.size());
    std::vector<std::vector<double>> best(layerCount);
    std::vector<std::vector<int>> cameLayer(layerCount);
    std::vector<std::vector<int>> cameSample(layerCount);

    for (int i = 0; i < layerCount; ++i) {
        best[i].assign(layers[i].size(), std::numeric_limits<double>::max());
        cameLayer[i].assign(layers[i].size(), -1);
        cameSample[i].assign(layers[i].size(), -1);
    }

    std::priority_queue<CorridorNode, std::vector<CorridorNode>, std::greater<CorridorNode>> open;
    best[0][0] = 0.0;
    open.push({0, 0, 0.0});

    while (!open.empty()) {
        const CorridorNode cur = open.top();
        open.pop();

        if (cur.cost > best[cur.layer][cur.sample]) {
            continue;
        }

        if (cur.layer == layerCount - 1) {
            std::vector<Vec2> raw;
            int layer = cur.layer;
            int sample = cur.sample;

            while (layer >= 0 && sample >= 0) {
                raw.push_back(layers[layer][sample].point);
                const int prevLayer = cameLayer[layer][sample];
                const int prevSample = cameSample[layer][sample];
                layer = prevLayer;
                sample = prevSample;
            }

            std::reverse(raw.begin(), raw.end());
            return simplifyWithClearance(raw);
        }

        const Vec2 fromPoint = layers[cur.layer][cur.sample].point;

        for (int nextLayer = cur.layer + 1; nextLayer < layerCount; ++nextLayer) {
            for (int nextSample = 0; nextSample < static_cast<int>(layers[nextLayer].size()); ++nextSample) {
                const Vec2 toPoint = layers[nextLayer][nextSample].point;

                if (!IsSegmentInsideCorridorSampled(fromPoint, toPoint, polyPath, polygons)) {
                    continue;
                }

                if (!IsSegmentClearOfWalls(fromPoint, toPoint, quantized)) {
                    continue;
                }

                double edgeCost = Distance(fromPoint, toPoint);
                edgeCost += layers[nextLayer][nextSample].landingPenalty;
                edgeCost -= static_cast<double>(nextLayer - cur.layer - 1) * 4.0;

                const double newCost = cur.cost + std::max(0.0, edgeCost);

                if (newCost < best[nextLayer][nextSample]) {
                    best[nextLayer][nextSample] = newCost;
                    cameLayer[nextLayer][nextSample] = cur.layer;
                    cameSample[nextLayer][nextSample] = cur.sample;
                    open.push({nextLayer, nextSample, newCost});
                }
            }
        }
    }

    return empty;
}
