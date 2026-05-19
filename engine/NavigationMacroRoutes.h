#ifndef NAVIGATIONMACROROUTES_H
#define NAVIGATIONMACROROUTES_H

#include "NavMesh.h"
#include "NavigationRegionGraph.h"
#include "NavigationStateCache.h"
#include <string>
#include <vector>

struct NavigationMacroRoute {
    std::string sourceRegionLabel;
    std::string targetRegionLabel;
    uint64_t stateKey = 0;
    int clearanceBucket = 0;

    std::string sourceAnchorId;
    std::string targetAnchorId;
    int sourceAnchorPoly = -1;
    int targetAnchorPoly = -1;
    Vec2 sourceAnchorPoint{};
    Vec2 targetAnchorPoint{};

    // Legacy fields kept for current runtime compatibility.
    int anchorPoly = -1;
    Vec2 anchorPoint{};

    float routeLength = 0.0f;
    std::vector<Vec2> finalRoute;
};

class NavigationMacroRoutes {
public:
    void Clear();
    bool BuildAllRoutesBetweenAnchors(const std::vector<NavigationAnchor>& anchors,
                                      const std::vector<int>& bucketRadii,
                                      uint64_t stateKey);
    const NavigationMacroRoute* FindBestRoute(const std::string& sourceRegionLabel,
                                              const std::string& targetRegionLabel,
                                              uint64_t stateKey,
                                              int clearanceBucket,
                                              const Vec2& startPoint) const;
    const std::vector<NavigationMacroRoute>& GetRoutes() const { return routes; }

private:
    std::vector<NavigationMacroRoute> routes;
};

#endif
