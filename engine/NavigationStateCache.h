#ifndef NAVIGATIONSTATECACHE_H
#define NAVIGATIONSTATECACHE_H

#include "NavMesh.h"
#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

struct NavigationAnchor {
    std::string anchorId;
    std::string ownerRegionLabel;
    std::string connectedRegionLabel;
    int clearanceBucket = 0;
    int anchorPoly = -1;
    Vec2 point{};
};

struct NavigationSubCacheRoute {
    int startPoly = -1;
    int anchorPoly = -1;
    int goalPoly = -1;
    Vec2 anchorPoint{};
    std::string anchorId;
    float routeLength = 0.0f;
    std::vector<Vec2> finalRoute;
};

struct NavigationRegionStateSubCache {
    std::string regionLabel;
    uint64_t stateKey = 0;
    int clearanceBucket = 0;
    std::vector<NavigationAnchor> exitAnchors;
    std::unordered_map<int, std::vector<NavigationSubCacheRoute>> polyToExitRoutes;
    // Legacy/best-of fields kept so current runtime still compiles until path assembly is rewritten.
    std::unordered_map<int, NavigationSubCacheRoute> polyToMacroAttach;
    std::unordered_map<int, NavigationSubCacheRoute> polyToExitAttach;
};

class NavigationStateCache {
public:
    void Clear();
    bool SaveToFile(const std::string& filePath, uint32_t navVersion, uint64_t mapVersion) const;
    bool LoadFromFile(const std::string& filePath, uint32_t navVersion, uint64_t mapVersion);
    void PutRegionStateCache(const NavigationRegionStateSubCache& subCache);
    const NavigationRegionStateSubCache* FindRegionStateCache(const std::string& regionLabel,
                                                              uint64_t stateKey,
                                                              int clearanceBucket) const;
    std::size_t GetSubCacheCount() const;

private:
    std::string MakeKey(const std::string& regionLabel, uint64_t stateKey, int clearanceBucket) const;
    std::unordered_map<std::string, NavigationRegionStateSubCache> subCaches;
};

#endif
