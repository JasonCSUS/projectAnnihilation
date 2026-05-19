#include "NavigationStateCache.h"

#include <fstream>
#include <cmath>
#include <iostream>

namespace {
constexpr int STATE_CACHE_FILE_VERSION = 2;

float ComputeRouteLength(const std::vector<Vec2>& route) {
    float total = 0.0f;
    for (size_t i = 1; i < route.size(); ++i) {
        const float dx = static_cast<float>(route[i].x - route[i - 1].x);
        const float dy = static_cast<float>(route[i].y - route[i - 1].y);
        total += std::sqrt(dx * dx + dy * dy);
    }
    return total;
}
}

void NavigationStateCache::Clear() { subCaches.clear(); }

void NavigationStateCache::PutRegionStateCache(const NavigationRegionStateSubCache& subCache) {
    subCaches[MakeKey(subCache.regionLabel, subCache.stateKey, subCache.clearanceBucket)] = subCache;
}

const NavigationRegionStateSubCache* NavigationStateCache::FindRegionStateCache(const std::string& regionLabel,
                                                                                 uint64_t stateKey,
                                                                                 int clearanceBucket) const {
    auto it = subCaches.find(MakeKey(regionLabel, stateKey, clearanceBucket));
    if (it == subCaches.end()) return nullptr;
    return &it->second;
}

std::size_t NavigationStateCache::GetSubCacheCount() const { return subCaches.size(); }

std::string NavigationStateCache::MakeKey(const std::string& regionLabel, uint64_t stateKey, int clearanceBucket) const {
    return regionLabel + "#" + std::to_string(stateKey) + "#" + std::to_string(clearanceBucket);
}

bool NavigationStateCache::SaveToFile(const std::string& filePath, uint32_t navVersion, uint64_t mapVersion) const {
    std::ofstream out(filePath, std::ios::binary | std::ios::trunc);
    if (!out) return false;

    const int version = STATE_CACHE_FILE_VERSION;
    const int count = static_cast<int>(subCaches.size());
    out.write(reinterpret_cast<const char*>(&version), sizeof(version));
    out.write(reinterpret_cast<const char*>(&navVersion), sizeof(navVersion));
    out.write(reinterpret_cast<const char*>(&mapVersion), sizeof(mapVersion));
    out.write(reinterpret_cast<const char*>(&count), sizeof(count));

    auto writeString = [&](const std::string& s) {
        int len = static_cast<int>(s.size());
        out.write(reinterpret_cast<const char*>(&len), sizeof(len));
        if (len > 0) out.write(s.data(), len);
    };

    auto writeRoute = [&](const NavigationSubCacheRoute& route) {
        out.write(reinterpret_cast<const char*>(&route.startPoly), sizeof(route.startPoly));
        out.write(reinterpret_cast<const char*>(&route.anchorPoly), sizeof(route.anchorPoly));
        out.write(reinterpret_cast<const char*>(&route.goalPoly), sizeof(route.goalPoly));
        out.write(reinterpret_cast<const char*>(&route.anchorPoint.x), sizeof(route.anchorPoint.x));
        out.write(reinterpret_cast<const char*>(&route.anchorPoint.y), sizeof(route.anchorPoint.y));
        out.write(reinterpret_cast<const char*>(&route.routeLength), sizeof(route.routeLength));
        writeString(route.anchorId);
        int pointCount = static_cast<int>(route.finalRoute.size());
        out.write(reinterpret_cast<const char*>(&pointCount), sizeof(pointCount));
        for (const Vec2& point : route.finalRoute) {
            out.write(reinterpret_cast<const char*>(&point.x), sizeof(point.x));
            out.write(reinterpret_cast<const char*>(&point.y), sizeof(point.y));
        }
    };

    auto writeLegacyMap = [&](const std::unordered_map<int, NavigationSubCacheRoute>& m) {
        int size = static_cast<int>(m.size());
        out.write(reinterpret_cast<const char*>(&size), sizeof(size));
        for (const auto& [polyId, route] : m) {
            out.write(reinterpret_cast<const char*>(&polyId), sizeof(polyId));
            writeRoute(route);
        }
    };

    for (const auto& [_, cache] : subCaches) {
        writeString(cache.regionLabel);
        out.write(reinterpret_cast<const char*>(&cache.stateKey), sizeof(cache.stateKey));
        out.write(reinterpret_cast<const char*>(&cache.clearanceBucket), sizeof(cache.clearanceBucket));

        int anchorCount = static_cast<int>(cache.exitAnchors.size());
        out.write(reinterpret_cast<const char*>(&anchorCount), sizeof(anchorCount));
        for (const auto& anchor : cache.exitAnchors) {
            writeString(anchor.anchorId);
            writeString(anchor.ownerRegionLabel);
            writeString(anchor.connectedRegionLabel);
            out.write(reinterpret_cast<const char*>(&anchor.clearanceBucket), sizeof(anchor.clearanceBucket));
            out.write(reinterpret_cast<const char*>(&anchor.anchorPoly), sizeof(anchor.anchorPoly));
            out.write(reinterpret_cast<const char*>(&anchor.point.x), sizeof(anchor.point.x));
            out.write(reinterpret_cast<const char*>(&anchor.point.y), sizeof(anchor.point.y));
        }

        int polyRouteCount = static_cast<int>(cache.polyToExitRoutes.size());
        out.write(reinterpret_cast<const char*>(&polyRouteCount), sizeof(polyRouteCount));
        for (const auto& [polyId, routes] : cache.polyToExitRoutes) {
            out.write(reinterpret_cast<const char*>(&polyId), sizeof(polyId));
            int routeCount = static_cast<int>(routes.size());
            out.write(reinterpret_cast<const char*>(&routeCount), sizeof(routeCount));
            for (const auto& route : routes) writeRoute(route);
        }

        writeLegacyMap(cache.polyToMacroAttach);
        writeLegacyMap(cache.polyToExitAttach);
    }

    return static_cast<bool>(out);
}

bool NavigationStateCache::LoadFromFile(const std::string& filePath,
                                        uint32_t navVersion,
                                        uint64_t mapVersion) {
    std::ifstream in(filePath, std::ios::binary);
    if (!in) return false;

    int version = 0, count = 0;
    uint32_t fileNavVersion = 0;
    uint64_t fileMapVersion = 0;
    in.read(reinterpret_cast<char*>(&version), sizeof(version));
    in.read(reinterpret_cast<char*>(&fileNavVersion), sizeof(fileNavVersion));
    in.read(reinterpret_cast<char*>(&fileMapVersion), sizeof(fileMapVersion));
    in.read(reinterpret_cast<char*>(&count), sizeof(count));

    if (!in || version != STATE_CACHE_FILE_VERSION || fileNavVersion != navVersion || fileMapVersion != mapVersion || count < 0) {
        return false;
    }

    auto readString = [&]() -> std::string {
        int len = 0;
        in.read(reinterpret_cast<char*>(&len), sizeof(len));
        if (len < 0) return {};
        std::string s(static_cast<size_t>(len), '\0');
        if (len > 0) in.read(s.data(), len);
        return s;
    };

    auto readRoute = [&]() -> NavigationSubCacheRoute {
        NavigationSubCacheRoute route;
        in.read(reinterpret_cast<char*>(&route.startPoly), sizeof(route.startPoly));
        in.read(reinterpret_cast<char*>(&route.anchorPoly), sizeof(route.anchorPoly));
        in.read(reinterpret_cast<char*>(&route.goalPoly), sizeof(route.goalPoly));
        in.read(reinterpret_cast<char*>(&route.anchorPoint.x), sizeof(route.anchorPoint.x));
        in.read(reinterpret_cast<char*>(&route.anchorPoint.y), sizeof(route.anchorPoint.y));
        in.read(reinterpret_cast<char*>(&route.routeLength), sizeof(route.routeLength));
        route.anchorId = readString();
        int pointCount = 0;
        in.read(reinterpret_cast<char*>(&pointCount), sizeof(pointCount));
        if (pointCount < 0) pointCount = 0;
        route.finalRoute.resize(static_cast<size_t>(pointCount));
        for (int p = 0; p < pointCount; ++p) {
            in.read(reinterpret_cast<char*>(&route.finalRoute[p].x), sizeof(route.finalRoute[p].x));
            in.read(reinterpret_cast<char*>(&route.finalRoute[p].y), sizeof(route.finalRoute[p].y));
        }
        if (route.routeLength <= 0.0f) route.routeLength = ComputeRouteLength(route.finalRoute);
        return route;
    };

    auto readLegacyMap = [&](std::unordered_map<int, NavigationSubCacheRoute>& m) -> bool {
        int size = 0;
        in.read(reinterpret_cast<char*>(&size), sizeof(size));
        if (size < 0) return false;
        for (int i = 0; i < size; ++i) {
            int polyId = -1;
            in.read(reinterpret_cast<char*>(&polyId), sizeof(polyId));
            m[polyId] = readRoute();
        }
        return true;
    };

    subCaches.clear();
    for (int i = 0; i < count; ++i) {
        NavigationRegionStateSubCache cache;
        cache.regionLabel = readString();
        in.read(reinterpret_cast<char*>(&cache.stateKey), sizeof(cache.stateKey));
        in.read(reinterpret_cast<char*>(&cache.clearanceBucket), sizeof(cache.clearanceBucket));

        int anchorCount = 0;
        in.read(reinterpret_cast<char*>(&anchorCount), sizeof(anchorCount));
        if (anchorCount < 0) return false;
        cache.exitAnchors.reserve(static_cast<size_t>(anchorCount));
        for (int a = 0; a < anchorCount; ++a) {
            NavigationAnchor anchor;
            anchor.anchorId = readString();
            anchor.ownerRegionLabel = readString();
            anchor.connectedRegionLabel = readString();
            in.read(reinterpret_cast<char*>(&anchor.clearanceBucket), sizeof(anchor.clearanceBucket));
            in.read(reinterpret_cast<char*>(&anchor.anchorPoly), sizeof(anchor.anchorPoly));
            in.read(reinterpret_cast<char*>(&anchor.point.x), sizeof(anchor.point.x));
            in.read(reinterpret_cast<char*>(&anchor.point.y), sizeof(anchor.point.y));
            cache.exitAnchors.push_back(std::move(anchor));
        }

        int polyRouteCount = 0;
        in.read(reinterpret_cast<char*>(&polyRouteCount), sizeof(polyRouteCount));
        if (polyRouteCount < 0) return false;
        for (int pr = 0; pr < polyRouteCount; ++pr) {
            int polyId = -1;
            int routeCount = 0;
            in.read(reinterpret_cast<char*>(&polyId), sizeof(polyId));
            in.read(reinterpret_cast<char*>(&routeCount), sizeof(routeCount));
            if (routeCount < 0) return false;
            auto& routes = cache.polyToExitRoutes[polyId];
            routes.reserve(static_cast<size_t>(routeCount));
            for (int r = 0; r < routeCount; ++r) routes.push_back(readRoute());
        }

        if (!readLegacyMap(cache.polyToMacroAttach)) return false;
        if (!readLegacyMap(cache.polyToExitAttach)) return false;

        subCaches[MakeKey(cache.regionLabel, cache.stateKey, cache.clearanceBucket)] = std::move(cache);
    }

    return static_cast<bool>(in);
}
