#include "NavigationLocalRoutes.h"

#include <algorithm>
#include <cmath>

namespace {
float RouteLength(const std::vector<Vec2>& route) {
    float total = 0.0f;
    for (size_t i = 1; i < route.size(); ++i) {
        const float dx = static_cast<float>(route[i].x - route[i - 1].x);
        const float dy = static_cast<float>(route[i].y - route[i - 1].y);
        total += std::sqrt(dx * dx + dy * dy);
    }
    return total;
}
}

bool NavigationLocalRoutes::BuildShortestAttachmentRoute(int startPoly,
                                                         const NavigationAnchor& anchor,
                                                         int clearanceBucket,
                                                         NavigationSubCacheRoute& outRoute) {
    if (startPoly < 0 || anchor.anchorPoly < 0) return false;

    const Vec2 startPoint = NavMesh::Instance().GetPolygonCentroid(startPoly);
    std::vector<int> polyPath = NavMesh::Instance().FindPath(startPoly, anchor.anchorPoly, clearanceBucket);
    if (polyPath.empty()) return false;

    std::vector<Vec2> finalRoute = NavMesh::Instance().FunnelPath(polyPath, startPoint, anchor.point, clearanceBucket);
    if (finalRoute.empty()) return false;

    outRoute.startPoly = startPoly;
    outRoute.anchorPoly = anchor.anchorPoly;
    outRoute.goalPoly = anchor.anchorPoly;
    outRoute.anchorPoint = anchor.point;
    outRoute.anchorId = anchor.anchorId;
    outRoute.finalRoute = std::move(finalRoute);
    outRoute.routeLength = RouteLength(outRoute.finalRoute);
    return true;
}

bool NavigationLocalRoutes::BuildRegionStateCache(const NavigationRegionGraph& regionGraph,
                                                  const std::string& regionLabel,
                                                  uint64_t stateKey,
                                                  int clearanceBucket,
                                                  const std::vector<NavigationAnchor>& exitAnchors,
                                                  NavigationRegionStateSubCache& outCache) {
    const NavigationRegionNode* region = regionGraph.FindRegion(regionLabel);
    if (!region) return false;

    outCache.regionLabel = regionLabel;
    outCache.stateKey = stateKey;
    outCache.clearanceBucket = clearanceBucket;
    outCache.exitAnchors = exitAnchors;
    outCache.polyToMacroAttach.clear();
    outCache.polyToExitAttach.clear();
    outCache.polyToExitRoutes.clear();

    for (int polyId : region->polygonIds) {
        std::vector<NavigationSubCacheRoute> builtRoutes;
        builtRoutes.reserve(exitAnchors.size());
        for (const auto& exitAnchor : exitAnchors) {
            if (exitAnchor.clearanceBucket != clearanceBucket) continue;
            NavigationSubCacheRoute route;
            if (BuildShortestAttachmentRoute(polyId, exitAnchor, clearanceBucket, route)) {
                builtRoutes.push_back(std::move(route));
            }
        }

        if (!builtRoutes.empty()) {
            std::sort(builtRoutes.begin(), builtRoutes.end(), [](const auto& a, const auto& b) {
                return a.routeLength < b.routeLength;
            });
            outCache.polyToExitAttach[polyId] = builtRoutes.front();
            outCache.polyToExitRoutes[polyId] = std::move(builtRoutes);
        }
    }

    return !outCache.polyToExitRoutes.empty();
}
