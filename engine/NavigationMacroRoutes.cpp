#include "NavigationMacroRoutes.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
float DistSq(const Vec2& a, const Vec2& b) {
    const float dx = static_cast<float>(a.x - b.x);
    const float dy = static_cast<float>(a.y - b.y);
    return dx * dx + dy * dy;
}

float RouteLength(const std::vector<Vec2>& route) {
    float total = 0.0f;
    for (size_t i = 1; i < route.size(); ++i) {
        const float dx = static_cast<float>(route[i].x - route[i-1].x);
        const float dy = static_cast<float>(route[i].y - route[i-1].y);
        total += std::sqrt(dx*dx + dy*dy);
    }
    return total;
}
}

void NavigationMacroRoutes::Clear() { routes.clear(); }

bool NavigationMacroRoutes::BuildAllRoutesBetweenAnchors(const std::vector<NavigationAnchor>& anchors,
                                                         const std::vector<int>& bucketRadii,
                                                         uint64_t stateKey) {
    int built = 0;
    for (size_t i = 0; i < anchors.size(); ++i) {
        for (size_t j = i + 1; j < anchors.size(); ++j) {
            const auto& a = anchors[i];
            const auto& b = anchors[j];
            if (a.ownerRegionLabel == b.ownerRegionLabel) continue;
            for (int rawBucket : bucketRadii) {
                const int bucket = NavMesh::QuantizeClearanceBucket(rawBucket);
                if (a.clearanceBucket != bucket || b.clearanceBucket != bucket) continue;
                if (a.anchorPoly < 0 || b.anchorPoly < 0) continue;
                std::vector<int> polyPath = NavMesh::Instance().FindPath(a.anchorPoly, b.anchorPoly, bucket);
                if (polyPath.empty()) continue;
                std::vector<Vec2> finalRoute = NavMesh::Instance().FunnelPath(polyPath, a.point, b.point, bucket);
                if (finalRoute.empty()) continue;

                NavigationMacroRoute route;
                route.sourceRegionLabel = a.ownerRegionLabel;
                route.targetRegionLabel = b.ownerRegionLabel;
                route.stateKey = stateKey;
                route.clearanceBucket = bucket;
                route.sourceAnchorId = a.anchorId;
                route.targetAnchorId = b.anchorId;
                route.sourceAnchorPoly = a.anchorPoly;
                route.targetAnchorPoly = b.anchorPoly;
                route.sourceAnchorPoint = a.point;
                route.targetAnchorPoint = b.point;
                route.anchorPoly = a.anchorPoly;
                route.anchorPoint = a.point;
                route.finalRoute = std::move(finalRoute);
                route.routeLength = RouteLength(route.finalRoute);
                routes.push_back(std::move(route));
                ++built;
            }
        }
    }
    return built > 0;
}

const NavigationMacroRoute* NavigationMacroRoutes::FindBestRoute(const std::string& sourceRegionLabel,
                                                                 const std::string& targetRegionLabel,
                                                                 uint64_t stateKey,
                                                                 int clearanceBucket,
                                                                 const Vec2& startPoint) const {
    const NavigationMacroRoute* best = nullptr;
    float bestScore = std::numeric_limits<float>::max();
    for (const auto& route : routes) {
        if (route.stateKey != stateKey) continue;
        if (route.clearanceBucket != clearanceBucket) continue;

        bool forward = false;
        if (route.sourceRegionLabel == sourceRegionLabel && route.targetRegionLabel == targetRegionLabel) {
            forward = true;
        } else if (route.sourceRegionLabel == targetRegionLabel && route.targetRegionLabel == sourceRegionLabel) {
            forward = false;
        } else {
            continue;
        }

        const Vec2& startAnchor = forward ? route.sourceAnchorPoint : route.targetAnchorPoint;
        const float score = DistSq(startPoint, startAnchor) + route.routeLength * 0.25f;
        if (score < bestScore) {
            bestScore = score;
            best = &route;
        }
    }
    return best;
}
