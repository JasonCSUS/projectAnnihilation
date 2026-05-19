#include "NavMesh.h"
#include "NavMeshInternal.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>
#include <vector>

using namespace navmesh_internal;

namespace {

constexpr float WALL_GRID_CELL = 96.0f;

long long WallGridKey(int cx, int cy) {
    return (static_cast<long long>(cx) << 32) ^ static_cast<unsigned int>(cy);
}

int WallCellCoord(float v) {
    return static_cast<int>(std::floor(v / WALL_GRID_CELL));
}

float Cross2D(float ax, float ay, float bx, float by) {
    return ax * by - ay * bx;
}

// True when segments [a,b] and [c,d] properly cross (parallel/collinear cases
// fall through to the 4 point-to-segment distances, which already return 0 for
// overlapping collinear segments).
bool SegmentsIntersect2D(const Vec2& a, const Vec2& b, const Vec2& c, const Vec2& d) {
    const float d1x = static_cast<float>(b.x - a.x);
    const float d1y = static_cast<float>(b.y - a.y);
    const float d2x = static_cast<float>(d.x - c.x);
    const float d2y = static_cast<float>(d.y - c.y);
    const float denom = Cross2D(d1x, d1y, d2x, d2y);
    if (std::abs(denom) < 0.0001f) return false;
    const float acx = static_cast<float>(c.x - a.x);
    const float acy = static_cast<float>(c.y - a.y);
    const float t = Cross2D(acx, acy, d2x, d2y) / denom;
    const float u = Cross2D(acx, acy, d1x, d1y) / denom;
    return t >= 0.0f && t <= 1.0f && u >= 0.0f && u <= 1.0f;
}

// Exact minimum distance between 2D segments [a,b] and [c,d].
// For non-intersecting segments the minimum is always the smallest of the 4
// point-to-segment distances (proved by the fact that in 2D the closest point
// pair always involves at least one segment endpoint when segments do not cross).
float SegmentToSegmentDist2D(const Vec2& a, const Vec2& b, const Vec2& c, const Vec2& d) {
    if (SegmentsIntersect2D(a, b, c, d)) return 0.0f;
    return std::min({
        DistancePointToSegment(c, d, a),
        DistancePointToSegment(c, d, b),
        DistancePointToSegment(a, b, c),
        DistancePointToSegment(a, b, d),
    });
}

} // namespace

// -------------------------------------------------------------------------
// Grid construction — called whenever boundaryWalls or activeDynamicWalls changes.
// -------------------------------------------------------------------------

void NavMesh::RebuildWallGrid() {
    wallGridWalls.clear();
    wallGridWalls.reserve(boundaryWalls.size() + activeDynamicWalls.size());
    wallGridWalls.insert(wallGridWalls.end(), boundaryWalls.begin(), boundaryWalls.end());
    wallGridWalls.insert(wallGridWalls.end(), activeDynamicWalls.begin(), activeDynamicWalls.end());

    wallGridCells.clear();

    for (int i = 0; i < static_cast<int>(wallGridWalls.size()); ++i) {
        const NavWallSegment& w = wallGridWalls[i];
        const float minX = static_cast<float>(std::min(w.a.x, w.b.x));
        const float maxX = static_cast<float>(std::max(w.a.x, w.b.x));
        const float minY = static_cast<float>(std::min(w.a.y, w.b.y));
        const float maxY = static_cast<float>(std::max(w.a.y, w.b.y));

        for (int cy = WallCellCoord(minY); cy <= WallCellCoord(maxY); ++cy) {
            for (int cx = WallCellCoord(minX); cx <= WallCellCoord(maxX); ++cx) {
                wallGridCells[WallGridKey(cx, cy)].push_back(i);
            }
        }
    }
}

// Returns the deduplicated indices of walls whose bounding cells overlap [minX,maxX]×[minY,maxY].
void NavMesh::GetWallCandidates(float minX, float minY, float maxX, float maxY,
                                std::vector<int>& out) const {
    std::unordered_set<int> seen;
    for (int cy = WallCellCoord(minY); cy <= WallCellCoord(maxY); ++cy) {
        for (int cx = WallCellCoord(minX); cx <= WallCellCoord(maxX); ++cx) {
            auto it = wallGridCells.find(WallGridKey(cx, cy));
            if (it == wallGridCells.end()) continue;
            for (int idx : it->second) {
                if (seen.insert(idx).second) {
                    out.push_back(idx);
                }
            }
        }
    }
}

// -------------------------------------------------------------------------
// Public / private collision functions — all now query the grid instead of
// iterating the full wall list.
// -------------------------------------------------------------------------

void NavMesh::GetAllActiveWalls(std::vector<NavWallSegment>& outWalls) const {
    outWalls.clear();
    outWalls.reserve(boundaryWalls.size() + activeDynamicWalls.size());
    outWalls.insert(outWalls.end(), boundaryWalls.begin(), boundaryWalls.end());
    outWalls.insert(outWalls.end(), activeDynamicWalls.begin(), activeDynamicWalls.end());
}

bool NavMesh::ResolveSoftCollision(const Vec2& center,
                                   float radius,
                                   float& outPushX,
                                   float& outPushY) const {
    outPushX = 0.0f;
    outPushY = 0.0f;

    if (radius <= 0.0f) {
        return false;
    }

    const float cx = static_cast<float>(center.x);
    const float cy = static_cast<float>(center.y);

    std::vector<int> candidates;
    GetWallCandidates(cx - radius, cy - radius, cx + radius, cy + radius, candidates);
    if (candidates.empty()) {
        return false;
    }

    bool hit = false;
    float accumX = 0.0f;
    float accumY = 0.0f;

    for (int idx : candidates) {
        const NavWallSegment& wall = wallGridWalls[idx];
        float closestX = 0.0f;
        float closestY = 0.0f;
        float t = 0.0f;

        ClosestPointOnSegment(wall.a, wall.b, cx, cy, closestX, closestY, t);

        float nx = cx - closestX;
        float ny = cy - closestY;
        const float distSq = nx * nx + ny * ny;

        if (distSq >= radius * radius) {
            continue;
        }

        const float dist = std::sqrt(std::max(0.0001f, distSq));
        float pushX = 0.0f;
        float pushY = 0.0f;

        if (wall.ownerPoly >= 0 && wall.ownerPoly < static_cast<int>(polygons.size())) {
            const NavPolygon& owner = polygons[wall.ownerPoly];

            int ownerEdgeIndex = -1;
            for (size_t i = 0; i < owner.vertices.size(); ++i) {
                const Vec2 a = owner.vertices[i];
                const Vec2 b = owner.vertices[(i + 1) % owner.vertices.size()];
                if (a.x == wall.a.x && a.y == wall.a.y &&
                    b.x == wall.b.x && b.y == wall.b.y) {
                    ownerEdgeIndex = static_cast<int>(i);
                    break;
                }
            }

            if (ownerEdgeIndex >= 0) {
                const Vec2 inward = ComputeInwardNormal(owner, ownerEdgeIndex);
                const float len = std::sqrt(static_cast<float>(inward.x * inward.x + inward.y * inward.y));
                if (len > 0.0001f) {
                    const float overlap = radius - dist;
                    pushX = (static_cast<float>(inward.x) / len) * overlap;
                    pushY = (static_cast<float>(inward.y) / len) * overlap;
                }
            }
        }

        if (std::abs(pushX) < 0.0001f && std::abs(pushY) < 0.0001f) {
            const float overlap = radius - dist;
            pushX = (nx / dist) * overlap;
            pushY = (ny / dist) * overlap;
        }

        accumX += pushX;
        accumY += pushY;
        hit = true;
    }

    outPushX = accumX;
    outPushY = accumY;
    return hit;
}

bool NavMesh::IsPointClearOfWalls(const Vec2& pt, int clearanceBucket) const {
    const int quantized = NavMesh::QuantizeClearanceBucket(clearanceBucket);
    if (quantized <= 0) {
        return true;
    }

    const float fq = static_cast<float>(quantized);
    const float px = static_cast<float>(pt.x);
    const float py = static_cast<float>(pt.y);

    std::vector<int> candidates;
    GetWallCandidates(px - fq, py - fq, px + fq, py + fq, candidates);

    for (int idx : candidates) {
        const NavWallSegment& wall = wallGridWalls[idx];
        if (DistancePointToSegment(wall.a, wall.b, pt) < fq) {
            return false;
        }
    }

    return true;
}

bool NavMesh::IsSegmentClearOfWalls(const Vec2& a,
                                    const Vec2& b,
                                    int clearanceBucket) const {
    const int quantized = NavMesh::QuantizeClearanceBucket(clearanceBucket);
    if (quantized <= 0) {
        return true;
    }

    const float fq = static_cast<float>(quantized);

    std::vector<int> candidates;
    GetWallCandidates(
        std::min(static_cast<float>(a.x), static_cast<float>(b.x)) - fq,
        std::min(static_cast<float>(a.y), static_cast<float>(b.y)) - fq,
        std::max(static_cast<float>(a.x), static_cast<float>(b.x)) + fq,
        std::max(static_cast<float>(a.y), static_cast<float>(b.y)) + fq,
        candidates);

    for (int idx : candidates) {
        if (SegmentToSegmentDist2D(a, b, wallGridWalls[idx].a, wallGridWalls[idx].b) < fq) {
            return false;
        }
    }
    return true;
}
