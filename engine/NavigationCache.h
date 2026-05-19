#ifndef NAVIGATIONCACHE_H
#define NAVIGATIONCACHE_H

#include "NavMesh.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

struct PathCacheKey {
    int startPoly = -1;
    int goalPoly = -1;
    int clearanceBucket = 0;
    uint64_t blockerRevision = 0;

    bool operator==(const PathCacheKey& other) const {
        return startPoly == other.startPoly &&
               goalPoly == other.goalPoly &&
               clearanceBucket == other.clearanceBucket &&
               blockerRevision == other.blockerRevision;
    }
};

struct PathCacheKeyHash {
    std::size_t operator()(const PathCacheKey& key) const;
};

struct CachedRoute {
    std::vector<Vec2> finalRoute;
    int startPointX = 0;
    int startPointY = 0;
    int goalPointX = 0;
    int goalPointY = 0;
    unsigned int hits = 0;
};

class NavigationCache {
public:
    bool LoadFromFile(const std::string& filePath,
                      int cacheFileVersion,
                      uint32_t navVersion,
                      uint64_t mapVersion);

    bool SaveToFile(const std::string& filePath,
                    int cacheFileVersion,
                    uint32_t navVersion,
                    uint64_t mapVersion) const;

    void Clear();

    bool IsDirty() const { return dirty; }
    void MarkClean() { dirty = false; }

    std::size_t GetRouteCount() const;

    bool TryGetExact(int startPoly,
                     int goalPoly,
                     int clearanceBucket,
                     uint64_t blockerRevision,
                     std::vector<Vec2>& outRoute,
                     Vec2* outStartPoint = nullptr,
                     Vec2* outGoalPoint = nullptr);

    void StoreExact(int startPoly,
                    int goalPoly,
                    int clearanceBucket,
                    uint64_t blockerRevision,
                    const std::vector<Vec2>& finalRoute,
                    const Vec2& startPoint,
                    const Vec2& goalPoint);

    bool TryGetNearbyIndexed(const Vec2& currentStart,
                             int goalPoly,
                             int clearanceBucket,
                             uint64_t blockerRevision,
                             std::vector<Vec2>& outRoute,
                             Vec2* outStartPoint,
                             Vec2* outGoalPoint,
                             float reuseDistance) const;

private:
    struct NearbyIndexKey {
        int goalPoly = -1;
        int clearanceBucket = 0;
        uint64_t blockerRevision = 0;

        bool operator==(const NearbyIndexKey& other) const {
            return goalPoly == other.goalPoly &&
                   clearanceBucket == other.clearanceBucket &&
                   blockerRevision == other.blockerRevision;
        }
    };

    struct NearbyIndexKeyHash {
        std::size_t operator()(const NearbyIndexKey& key) const;
    };

private:
    void RebuildIndexes();

private:
    std::unordered_map<PathCacheKey, CachedRoute, PathCacheKeyHash> routes;
    std::unordered_map<NearbyIndexKey, std::vector<PathCacheKey>, NearbyIndexKeyHash> nearbyIndex;
    bool dirty = false;
};

#endif
