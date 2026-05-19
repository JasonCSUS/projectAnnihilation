#include "NavigationTraining.h"
#include "NavMesh.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <set>
#include <unordered_map>

namespace {
std::vector<Vec2> BuildThreeAnchorsFromPortal(const Vec2& a, const Vec2& b, int clearanceBucket) {
    std::vector<Vec2> out;
    const double dx = static_cast<double>(b.x - a.x);
    const double dy = static_cast<double>(b.y - a.y);
    const double len = std::sqrt(dx * dx + dy * dy);
    if (len < 1.0) {
        out.push_back(a);
        return out;
    }

    const double inset = std::min(len * 0.25, static_cast<double>(clearanceBucket));
    const double ux = dx / len;
    const double uy = dy / len;
    const Vec2 left{static_cast<int>(std::lround(a.x + ux * inset)), static_cast<int>(std::lround(a.y + uy * inset))};
    const Vec2 right{static_cast<int>(std::lround(b.x - ux * inset)), static_cast<int>(std::lround(b.y - uy * inset))};
    const Vec2 mid{static_cast<int>(std::lround((left.x + right.x) * 0.5)), static_cast<int>(std::lround((left.y + right.y) * 0.5))};
    out = {left, mid, right};

    std::vector<Vec2> deduped;
    for (const auto& p : out) {
        bool dup = false;
        for (const auto& existing : deduped) {
            if (existing.x == p.x && existing.y == p.y) { dup = true; break; }
        }
        if (!dup) deduped.push_back(p);
    }
    return deduped;
}
}

void NavigationTraining::EnumerateRegionStateKeys(const std::string& regionLabel,
                                                  std::vector<uint64_t>& outStateKeys) const {
    outStateKeys.clear();

    std::unordered_map<std::string, bool> baseState;
    for (const auto& blocker : regionGraph.GetBlockers()) {
        baseState[blocker.toggleId] = true;
    }
    outStateKeys.push_back(regionGraph.ComputeStateKey(baseState));

    std::vector<std::string> roomSpawners;
    std::vector<std::string> roomBlockades;

    for (const auto& blocker : regionGraph.GetBlockers()) {
        if (blocker.owningRegion != regionLabel) continue;
        if (blocker.blockerType == "spawner") roomSpawners.push_back(blocker.toggleId);
        else if (blocker.blockerType == "blockade") roomBlockades.push_back(blocker.toggleId);
    }

    const int spawnerCount = static_cast<int>(roomSpawners.size());
    for (int mask = 1; mask < (1 << spawnerCount); ++mask) {
        std::unordered_map<std::string, bool> state = baseState;
        for (int i = 0; i < spawnerCount; ++i) {
            if ((mask >> i) & 1) state[roomSpawners[i]] = false;
        }
        outStateKeys.push_back(regionGraph.ComputeStateKey(state));
    }

    std::unordered_map<std::string, bool> roomClear = baseState;
    for (const std::string& toggleId : roomSpawners) roomClear[toggleId] = false;
    outStateKeys.push_back(regionGraph.ComputeStateKey(roomClear));

    const int blockadeCount = static_cast<int>(roomBlockades.size());
    for (int mask = 1; mask < (1 << blockadeCount); ++mask) {
        std::unordered_map<std::string, bool> state = roomClear;
        for (int i = 0; i < blockadeCount; ++i) {
            if ((mask >> i) & 1) state[roomBlockades[i]] = false;
        }
        outStateKeys.push_back(regionGraph.ComputeStateKey(state));
    }

    std::sort(outStateKeys.begin(), outStateKeys.end());
    outStateKeys.erase(std::unique(outStateKeys.begin(), outStateKeys.end()), outStateKeys.end());
}

void NavigationTraining::BuildExitAnchorsForRegion(const std::string& regionLabel,
                                                   int clearanceBucket,
                                                   std::vector<NavigationAnchor>& outAnchors) const {
    outAnchors.clear();
    const NavigationRegionNode* region = regionGraph.FindRegion(regionLabel);
    if (!region) return;

    std::set<std::string> seen;
    for (int polyId : region->polygonIds) {
        const NavPolygon* poly = NavMesh::Instance().GetPolygon(polyId);
        if (!poly) continue;
        for (int neighborPoly : poly->neighborIndices) {
            const std::string neighborRegion = regionGraph.GetPrimaryRegionForPoly(neighborPoly);
            if (neighborRegion.empty() || neighborRegion == regionLabel) continue;
            Vec2 a{}, b{};
            if (!NavMesh::Instance().GetSharedPortal(polyId, neighborPoly, a, b)) continue;

            std::string portalKey = neighborRegion + ":" +
                std::to_string(std::min(a.x, b.x)) + "," + std::to_string(std::min(a.y, b.y)) + ":" +
                std::to_string(std::max(a.x, b.x)) + "," + std::to_string(std::max(a.y, b.y));
            if (!seen.insert(portalKey).second) continue;

            auto samples = BuildThreeAnchorsFromPortal(a, b, clearanceBucket);
            for (size_t i = 0; i < samples.size(); ++i) {
                const Vec2 clamped = NavMesh::Instance().ClampToNavMesh(samples[i], clearanceBucket);
                const int anchorPoly = NavMesh::Instance().GetPolygonIndexAt(clamped.x, clamped.y, clearanceBucket);
                if (anchorPoly < 0) continue;
                NavigationAnchor anchor;
                anchor.anchorId = regionLabel + "->" + neighborRegion + "#" + std::to_string(seen.size()) + ":" + std::to_string(i);
                anchor.ownerRegionLabel = regionLabel;
                anchor.connectedRegionLabel = neighborRegion;
                anchor.clearanceBucket = clearanceBucket;
                anchor.anchorPoly = anchorPoly;
                anchor.point = clamped;
                outAnchors.push_back(std::move(anchor));
            }
        }
    }
}

bool NavigationTraining::BuildStartupCaches(const std::string& navJsonPath,
                                            const std::string& targetRegionLabel,
                                            const std::vector<int>& bucketRadii,
                                            const std::string& cacheFilePath) {
    (void)targetRegionLabel;
    if (!regionGraph.LoadFromNavJson(navJsonPath)) {
        std::cout << "NavigationTraining: failed loading region graph from " << navJsonPath << "\n";
        return false;
    }

    macroRoutes.Clear();
    stateCache.Clear();

    std::vector<uint64_t> allStateKeys;
    for (const auto& region : regionGraph.GetRegions()) {
        std::vector<uint64_t> stateKeys;
        EnumerateRegionStateKeys(region.label, stateKeys);
        allStateKeys.insert(allStateKeys.end(), stateKeys.begin(), stateKeys.end());
    }
    std::sort(allStateKeys.begin(), allStateKeys.end());
    allStateKeys.erase(std::unique(allStateKeys.begin(), allStateKeys.end()), allStateKeys.end());

    std::unordered_map<std::string, std::vector<NavigationAnchor>> anchorsByRegionBucket;
    std::vector<NavigationAnchor> allAnchors;
    for (const auto& region : regionGraph.GetRegions()) {
        for (int rawBucket : bucketRadii) {
            const int bucket = NavMesh::QuantizeClearanceBucket(rawBucket);
            std::vector<NavigationAnchor> anchors;
            BuildExitAnchorsForRegion(region.label, bucket, anchors);
            anchorsByRegionBucket[region.label + "#" + std::to_string(bucket)] = anchors;
            allAnchors.insert(allAnchors.end(), anchors.begin(), anchors.end());
        }
    }

    for (uint64_t stateKey : allStateKeys) {
        macroRoutes.BuildAllRoutesBetweenAnchors(allAnchors, bucketRadii, stateKey);
    }

    for (const auto& region : regionGraph.GetRegions()) {
        std::vector<uint64_t> stateKeys;
        EnumerateRegionStateKeys(region.label, stateKeys);
        for (uint64_t stateKey : stateKeys) {
            for (int rawBucket : bucketRadii) {
                const int bucket = NavMesh::QuantizeClearanceBucket(rawBucket);
                const auto& exitAnchors = anchorsByRegionBucket[region.label + "#" + std::to_string(bucket)];
                NavigationRegionStateSubCache subCache;
                if (localRoutes.BuildRegionStateCache(regionGraph, region.label, stateKey, bucket, exitAnchors, subCache)) {
                    stateCache.PutRegionStateCache(subCache);
                }
            }
        }
    }

    const bool saved = stateCache.SaveToFile(cacheFilePath,
                                             NavMesh::GetPathCacheNavVersion(),
                                             NavMesh::Instance().GetPathCacheMapVersion());
    if (saved) {
        std::cout << "NavigationTraining: built and saved "
                  << stateCache.GetSubCacheCount()
                  << " state subcaches to "
                  << cacheFilePath
                  << "\n";
    }
    return saved;
}

bool NavigationTraining::LoadStartupCaches(const std::string& navJsonPath,
                                           const std::string& targetRegionLabel,
                                           const std::vector<int>& bucketRadii,
                                           const std::string& cacheFilePath) {
    (void)targetRegionLabel; (void)bucketRadii;
    if (!regionGraph.LoadFromNavJson(navJsonPath)) return false;
    const bool loaded = stateCache.LoadFromFile(cacheFilePath,
                                                NavMesh::GetPathCacheNavVersion(),
                                                NavMesh::Instance().GetPathCacheMapVersion());
    if (loaded) {
        std::cout << "NavigationTraining: loaded " << stateCache.GetSubCacheCount()
                  << " state subcaches from " << cacheFilePath << "\n";
    }
    return loaded;
}

bool NavigationTraining::EnsureStartupCaches(const std::string& navJsonPath,
                                             const std::string& targetRegionLabel,
                                             const std::vector<int>& bucketRadii,
                                             const std::string& cacheFilePath) {
    if (LoadStartupCaches(navJsonPath, targetRegionLabel, bucketRadii, cacheFilePath)) {
        return true;
    }
    return BuildStartupCaches(navJsonPath, targetRegionLabel, bucketRadii, cacheFilePath);
}
