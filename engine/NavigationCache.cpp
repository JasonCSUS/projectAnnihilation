#include "NavigationCache.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>

std::size_t PathCacheKeyHash::operator()(const PathCacheKey& key) const {
    const std::size_t h1 = std::hash<int>{}(key.startPoly);
    const std::size_t h2 = std::hash<int>{}(key.goalPoly);
    const std::size_t h3 = std::hash<int>{}(key.clearanceBucket);
    const std::size_t h4 = std::hash<uint64_t>{}(key.blockerRevision);
    return h1 ^ (h2 << 1) ^ (h3 << 2) ^ (h4 << 3);
}

std::size_t NavigationCache::NearbyIndexKeyHash::operator()(const NearbyIndexKey& key) const {
    const std::size_t h1 = std::hash<int>{}(key.goalPoly);
    const std::size_t h2 = std::hash<int>{}(key.clearanceBucket);
    const std::size_t h3 = std::hash<uint64_t>{}(key.blockerRevision);
    return h1 ^ (h2 << 1) ^ (h3 << 2);
}

namespace {
float Distance2D(const Vec2& a, const Vec2& b) {
    const float dx = static_cast<float>(a.x - b.x);
    const float dy = static_cast<float>(a.y - b.y);
    return std::sqrt(dx * dx + dy * dy);
}
}

void NavigationCache::Clear() {
    routes.clear();
    nearbyIndex.clear();
    dirty = false;
}

std::size_t NavigationCache::GetRouteCount() const {
    return routes.size();
}

void NavigationCache::RebuildIndexes() {
    nearbyIndex.clear();

    for (const auto& [key, route] : routes) {
        if (route.finalRoute.empty()) {
            continue;
        }

        NearbyIndexKey idx{key.goalPoly, key.clearanceBucket, key.blockerRevision};
        nearbyIndex[idx].push_back(key);
    }
}

bool NavigationCache::LoadFromFile(const std::string& filePath,
                                   int cacheFileVersion,
                                   uint32_t navVersion,
                                   uint64_t mapVersion) {
    std::ifstream in(filePath, std::ios::binary);
    if (!in) {
        std::cout << "NavigationCache: file not found: " << filePath << "\n";
        return false;
    }

    int version = 0;
    uint32_t fileNavVersion = 0;
    uint64_t fileMapVersion = 0;
    int entryCount = 0;

    in.read(reinterpret_cast<char*>(&version), sizeof(version));
    in.read(reinterpret_cast<char*>(&fileNavVersion), sizeof(fileNavVersion));
    in.read(reinterpret_cast<char*>(&fileMapVersion), sizeof(fileMapVersion));
    in.read(reinterpret_cast<char*>(&entryCount), sizeof(entryCount));

    if (!in || version != cacheFileVersion || entryCount < 0 ||
        fileNavVersion != navVersion || fileMapVersion != mapVersion) {
        std::cout << "NavigationCache: incompatible file " << filePath << "\n";
        return false;
    }

    routes.clear();

    for (int i = 0; i < entryCount; ++i) {
        PathCacheKey key;
        CachedRoute route;
        int pointCount = 0;

        in.read(reinterpret_cast<char*>(&key.startPoly), sizeof(key.startPoly));
        in.read(reinterpret_cast<char*>(&key.goalPoly), sizeof(key.goalPoly));
        in.read(reinterpret_cast<char*>(&key.clearanceBucket), sizeof(key.clearanceBucket));
        in.read(reinterpret_cast<char*>(&key.blockerRevision), sizeof(key.blockerRevision));

        in.read(reinterpret_cast<char*>(&route.startPointX), sizeof(route.startPointX));
        in.read(reinterpret_cast<char*>(&route.startPointY), sizeof(route.startPointY));
        in.read(reinterpret_cast<char*>(&route.goalPointX), sizeof(route.goalPointX));
        in.read(reinterpret_cast<char*>(&route.goalPointY), sizeof(route.goalPointY));

        in.read(reinterpret_cast<char*>(&pointCount), sizeof(pointCount));
        if (!in || pointCount <= 0) {
            std::cout << "NavigationCache: corrupted entry in " << filePath << "\n";
            return false;
        }

        route.finalRoute.resize(static_cast<std::size_t>(pointCount));
        for (int p = 0; p < pointCount; ++p) {
            in.read(reinterpret_cast<char*>(&route.finalRoute[p].x), sizeof(route.finalRoute[p].x));
            in.read(reinterpret_cast<char*>(&route.finalRoute[p].y), sizeof(route.finalRoute[p].y));
        }

        in.read(reinterpret_cast<char*>(&route.hits), sizeof(route.hits));

        if (!in) {
            std::cout << "NavigationCache: read failure in " << filePath << "\n";
            return false;
        }

        routes[key] = std::move(route);
    }

    RebuildIndexes();
    dirty = false;

    std::cout << "NavigationCache: loaded " << routes.size()
              << " routes from " << filePath << "\n";
    return true;
}

bool NavigationCache::SaveToFile(const std::string& filePath,
                                 int cacheFileVersion,
                                 uint32_t navVersion,
                                 uint64_t mapVersion) const {
    std::ofstream out(filePath, std::ios::binary | std::ios::trunc);
    if (!out) {
        std::cout << "NavigationCache: failed opening " << filePath << " for write\n";
        return false;
    }

    const int entryCount = static_cast<int>(routes.size());

    out.write(reinterpret_cast<const char*>(&cacheFileVersion), sizeof(cacheFileVersion));
    out.write(reinterpret_cast<const char*>(&navVersion), sizeof(navVersion));
    out.write(reinterpret_cast<const char*>(&mapVersion), sizeof(mapVersion));
    out.write(reinterpret_cast<const char*>(&entryCount), sizeof(entryCount));

    for (const auto& [key, route] : routes) {
        const int pointCount = static_cast<int>(route.finalRoute.size());

        out.write(reinterpret_cast<const char*>(&key.startPoly), sizeof(key.startPoly));
        out.write(reinterpret_cast<const char*>(&key.goalPoly), sizeof(key.goalPoly));
        out.write(reinterpret_cast<const char*>(&key.clearanceBucket), sizeof(key.clearanceBucket));
        out.write(reinterpret_cast<const char*>(&key.blockerRevision), sizeof(key.blockerRevision));

        out.write(reinterpret_cast<const char*>(&route.startPointX), sizeof(route.startPointX));
        out.write(reinterpret_cast<const char*>(&route.startPointY), sizeof(route.startPointY));
        out.write(reinterpret_cast<const char*>(&route.goalPointX), sizeof(route.goalPointX));
        out.write(reinterpret_cast<const char*>(&route.goalPointY), sizeof(route.goalPointY));

        out.write(reinterpret_cast<const char*>(&pointCount), sizeof(pointCount));
        for (const Vec2& point : route.finalRoute) {
            out.write(reinterpret_cast<const char*>(&point.x), sizeof(point.x));
            out.write(reinterpret_cast<const char*>(&point.y), sizeof(point.y));
        }

        out.write(reinterpret_cast<const char*>(&route.hits), sizeof(route.hits));
    }

    return static_cast<bool>(out);
}

bool NavigationCache::TryGetExact(int startPoly,
                                  int goalPoly,
                                  int clearanceBucket,
                                  uint64_t blockerRevision,
                                  std::vector<Vec2>& outRoute,
                                  Vec2* outStartPoint,
                                  Vec2* outGoalPoint) {
    const PathCacheKey key{
        startPoly,
        goalPoly,
        clearanceBucket,
        blockerRevision
    };

    auto it = routes.find(key);
    if (it == routes.end() || it->second.finalRoute.empty()) {
        return false;
    }

    outRoute = it->second.finalRoute;
    if (outStartPoint) {
        outStartPoint->x = it->second.startPointX;
        outStartPoint->y = it->second.startPointY;
    }
    if (outGoalPoint) {
        outGoalPoint->x = it->second.goalPointX;
        outGoalPoint->y = it->second.goalPointY;
    }

    it->second.hits += 1;
    return true;
}

void NavigationCache::StoreExact(int startPoly,
                                 int goalPoly,
                                 int clearanceBucket,
                                 uint64_t blockerRevision,
                                 const std::vector<Vec2>& finalRoute,
                                 const Vec2& startPoint,
                                 const Vec2& goalPoint) {
    if (finalRoute.empty()) {
        return;
    }

    const PathCacheKey key{
        startPoly,
        goalPoly,
        clearanceBucket,
        blockerRevision
    };

    CachedRoute& route = routes[key];
    route.finalRoute = finalRoute;
    route.startPointX = startPoint.x;
    route.startPointY = startPoint.y;
    route.goalPointX = goalPoint.x;
    route.goalPointY = goalPoint.y;
    route.hits = 0;

    RebuildIndexes();
    dirty = true;
}

bool NavigationCache::TryGetNearbyIndexed(const Vec2& currentStart,
                                          int goalPoly,
                                          int clearanceBucket,
                                          uint64_t blockerRevision,
                                          std::vector<Vec2>& outRoute,
                                          Vec2* outStartPoint,
                                          Vec2* outGoalPoint,
                                          float reuseDistance) const {
    const NearbyIndexKey idx{
        goalPoly,
        clearanceBucket,
        blockerRevision
    };

    auto itIndex = nearbyIndex.find(idx);
    if (itIndex == nearbyIndex.end()) {
        return false;
    }

    const CachedRoute* bestRoute = nullptr;
    float bestDist = std::numeric_limits<float>::max();
    unsigned int bestHits = 0;

    for (const PathCacheKey& key : itIndex->second) {
        auto itRoute = routes.find(key);
        if (itRoute == routes.end() || itRoute->second.finalRoute.empty()) {
            continue;
        }

        const Vec2 anchor{itRoute->second.startPointX, itRoute->second.startPointY};
        const float d = Distance2D(currentStart, anchor);
        if (d > reuseDistance) {
            continue;
        }

        if (!NavMesh::Instance().HasLineOfSight(currentStart, anchor, clearanceBucket)) {
            continue;
        }

        if (!bestRoute ||
            itRoute->second.hits > bestHits ||
            (itRoute->second.hits == bestHits && d < bestDist)) {
            bestRoute = &itRoute->second;
            bestDist = d;
            bestHits = itRoute->second.hits;
        }
    }

    if (!bestRoute) {
        return false;
    }

    outRoute = bestRoute->finalRoute;
    if (outStartPoint) {
        *outStartPoint = {bestRoute->startPointX, bestRoute->startPointY};
    }
    if (outGoalPoint) {
        *outGoalPoint = {bestRoute->goalPointX, bestRoute->goalPointY};
    }
    return true;
}
