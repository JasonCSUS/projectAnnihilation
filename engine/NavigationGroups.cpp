#include "NavigationGroups.h"

#include <cmath>
#include <limits>

namespace {
constexpr int GROUP_CELL_SIZE = 160;

float Distance2D(const Vec2& a, const Vec2& b) {
    const float dx = static_cast<float>(a.x - b.x);
    const float dy = static_cast<float>(a.y - b.y);
    return std::sqrt(dx * dx + dy * dy);
}
}

std::size_t NavigationGroups::GroupKeyHash::operator()(const GroupKey& key) const {
    std::size_t h = std::hash<int>{}(key.goalPoly);
    h ^= std::hash<int>{}(key.clearanceBucket) << 1;
    h ^= std::hash<uint64_t>{}(key.blockerRevision) << 2;
    h ^= std::hash<int>{}(key.cellX) << 3;
    h ^= std::hash<int>{}(key.cellY) << 4;
    return h;
}

int NavigationGroups::CellX(int x) {
    return static_cast<int>(std::floor(static_cast<float>(x) / static_cast<float>(GROUP_CELL_SIZE)));
}

int NavigationGroups::CellY(int y) {
    return static_cast<int>(std::floor(static_cast<float>(y) / static_cast<float>(GROUP_CELL_SIZE)));
}

void NavigationGroups::Clear() {
    groups.clear();
}

void NavigationGroups::RegisterSharedRoute(const Vec2& anchorStart,
                                           const Vec2& goalPoint,
                                           const std::vector<Vec2>& finalRoute,
                                           int goalPoly,
                                           int clearanceBucket,
                                           uint64_t blockerRevision) {
    if (finalRoute.empty()) {
        return;
    }

    GroupKey key;
    key.goalPoly = goalPoly;
    key.clearanceBucket = clearanceBucket;
    key.blockerRevision = blockerRevision;
    key.cellX = CellX(anchorStart.x);
    key.cellY = CellY(anchorStart.y);

    auto& bucket = groups[key];

    for (auto& route : bucket) {
        if (route.anchorStart.x == anchorStart.x &&
            route.anchorStart.y == anchorStart.y &&
            route.goalPoint.x == goalPoint.x &&
            route.goalPoint.y == goalPoint.y &&
            route.finalRoute == finalRoute) {
            route.score += 1;
            return;
        }
    }

    SharedRoute route;
    route.anchorStart = anchorStart;
    route.goalPoint = goalPoint;
    route.finalRoute = finalRoute;
    route.score = 1;
    bucket.push_back(std::move(route));
}

bool NavigationGroups::TryGetSharedRoute(const Vec2& currentStart,
                                         int goalPoly,
                                         int clearanceBucket,
                                         uint64_t blockerRevision,
                                         std::vector<Vec2>& outRoute,
                                         Vec2& outAnchorStart,
                                         Vec2& outGoalPoint,
                                         float reuseDistance) const {
    const int baseX = CellX(currentStart.x);
    const int baseY = CellY(currentStart.y);

    const SharedRoute* bestRoute = nullptr;
    float bestDist = std::numeric_limits<float>::max();
    unsigned int bestScore = 0;

    for (int oy = -1; oy <= 1; ++oy) {
        for (int ox = -1; ox <= 1; ++ox) {
            GroupKey key;
            key.goalPoly = goalPoly;
            key.clearanceBucket = clearanceBucket;
            key.blockerRevision = blockerRevision;
            key.cellX = baseX + ox;
            key.cellY = baseY + oy;

            auto it = groups.find(key);
            if (it == groups.end()) {
                continue;
            }

            for (const auto& route : it->second) {
                const float d = Distance2D(currentStart, route.anchorStart);
                if (d > reuseDistance) {
                    continue;
                }

                if (!NavMesh::Instance().HasLineOfSight(currentStart, route.anchorStart, clearanceBucket)) {
                    continue;
                }

                if (!bestRoute ||
                    route.score > bestScore ||
                    (route.score == bestScore && d < bestDist)) {
                    bestRoute = &route;
                    bestScore = route.score;
                    bestDist = d;
                }
            }
        }
    }

    if (!bestRoute) {
        return false;
    }

    outRoute = bestRoute->finalRoute;
    outAnchorStart = bestRoute->anchorStart;
    outGoalPoint = bestRoute->goalPoint;
    return true;
}
