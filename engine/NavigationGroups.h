#ifndef NAVIGATIONGROUPS_H
#define NAVIGATIONGROUPS_H

#include "NavMesh.h"

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

class NavigationGroups {
public:
    void Clear();

    void RegisterSharedRoute(const Vec2& anchorStart,
                             const Vec2& goalPoint,
                             const std::vector<Vec2>& finalRoute,
                             int goalPoly,
                             int clearanceBucket,
                             uint64_t blockerRevision);

    bool TryGetSharedRoute(const Vec2& currentStart,
                           int goalPoly,
                           int clearanceBucket,
                           uint64_t blockerRevision,
                           std::vector<Vec2>& outRoute,
                           Vec2& outAnchorStart,
                           Vec2& outGoalPoint,
                           float reuseDistance) const;

private:
    struct GroupKey {
        int goalPoly = -1;
        int clearanceBucket = 0;
        uint64_t blockerRevision = 0;
        int cellX = 0;
        int cellY = 0;

        bool operator==(const GroupKey& other) const {
            return goalPoly == other.goalPoly &&
                   clearanceBucket == other.clearanceBucket &&
                   blockerRevision == other.blockerRevision &&
                   cellX == other.cellX &&
                   cellY == other.cellY;
        }
    };

    struct GroupKeyHash {
        std::size_t operator()(const GroupKey& key) const;
    };

    struct SharedRoute {
        Vec2 anchorStart{};
        Vec2 goalPoint{};
        std::vector<Vec2> finalRoute;
        unsigned int score = 0;
    };

private:
    static int CellX(int x);
    static int CellY(int y);

private:
    std::unordered_map<GroupKey, std::vector<SharedRoute>, GroupKeyHash> groups;
};

#endif
