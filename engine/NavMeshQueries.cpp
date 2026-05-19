#include "NavMesh.h"
#include "NavMeshInternal.h"

#include <cmath>
#include <limits>

using namespace navmesh_internal;

bool NavMesh::IsPointWalkable(const Vec2& pt, int clearanceBucket) const {
    return GetContainingPolygonIndexAt(pt.x, pt.y, clearanceBucket) >= 0;
}

bool NavMesh::IsPolygonEnabled(int polyIndex) const {
    // Lock-free read: polygonEnabled (uint8_t) is only resized at load/blocker-toggle
    // time, never during normal gameplay where this is called per-polygon per-frame.
    return polyIndex >= 0 &&
           polyIndex < static_cast<int>(polygonEnabled.size()) &&
           polygonEnabled[polyIndex] != 0;
}

int NavMesh::GetPolygonIndexAt(int x, int y, int clearanceBucket) const {
    if (polygons.empty()) {
        return -1;
    }

    const int direct = GetContainingPolygonIndexAt(x, y, clearanceBucket);
    if (direct >= 0) {
        return direct;
    }

    const Vec2 point = {x, y};
    const int quantized = NavMesh::QuantizeClearanceBucket(clearanceBucket);

    double bestDist = std::numeric_limits<double>::max();
    int bestIndex = -1;

    for (size_t i = 0; i < polygons.size(); ++i) {
        if (!IsPolygonEnabled(static_cast<int>(i))) {
            continue;
        }

        Vec2 candidate = ClampPointToPolygon(point, polygons[i].vertices);

        if (quantized > 0) {
            if (!IsPointClearOfWalls(candidate, quantized)) {
                Vec2 safe = candidate;
                if (!FindNearbyClearPoint(candidate, quantized, safe)) {
                    continue;
                }
                candidate = safe;
            }

            if (!PointInPolygon(candidate, polygons[i].vertices)) {
                continue;
            }
        }

        const double d = DistanceSquared(point, candidate);
        if (d < bestDist) {
            bestDist = d;
            bestIndex = static_cast<int>(i);
        }
    }

    return bestIndex;
}

Vec2 NavMesh::ClampToNavMesh(const Vec2& pt, int clearanceBucket) {
    if (polygons.empty()) {
        return pt;
    }

    const int quantized = NavMesh::QuantizeClearanceBucket(clearanceBucket);

    const int containing = GetContainingPolygonIndexAt(pt.x, pt.y, quantized);
    if (containing >= 0) {
        return pt;
    }

    const int nearest = GetPolygonIndexAt(pt.x, pt.y, quantized);
    if (nearest < 0 || nearest >= static_cast<int>(polygons.size())) {
        return pt;
    }

    Vec2 base = ClampPointToPolygon(pt, polygons[nearest].vertices);

    if (quantized <= 0) {
        return base;
    }

    if (GetContainingPolygonIndexAt(base.x, base.y, quantized) >= 0 &&
        IsPointClearOfWalls(base, quantized)) {
        return base;
    }

    Vec2 safe = base;
    if (FindNearbyClearPoint(base, quantized, safe)) {
        return safe;
    }

    safe = pt;
    if (FindNearbyClearPoint(pt, quantized, safe)) {
        return safe;
    }

    return base;
}

int NavMesh::GetContainingPolygonIndexAt(int x, int y, int clearanceBucket) const {
    if (polygons.empty()) {
        return -1;
    }

    const Vec2 point = {x, y};
    const int quantized = NavMesh::QuantizeClearanceBucket(clearanceBucket);

    for (size_t i = 0; i < polygons.size(); ++i) {
        if (!IsPolygonEnabled(static_cast<int>(i))) continue;
        if (!PointInPolygon(point, polygons[i].vertices)) continue;
        if (quantized > 0 && !IsPointClearOfWalls(point, quantized)) continue;
        return static_cast<int>(i);
    }

    return -1;
}

bool NavMesh::FindNearbyClearPoint(const Vec2& desired,
                                   int clearanceBucket,
                                   Vec2& outPoint) const {
    const int quantized = NavMesh::QuantizeClearanceBucket(clearanceBucket);
    if (quantized <= 0) {
        outPoint = desired;
        return true;
    }

    if (GetContainingPolygonIndexAt(desired.x, desired.y, quantized) >= 0) {
        outPoint = desired;
        return true;
    }

    double bestDistSq = std::numeric_limits<double>::max();
    bool found = false;

    constexpr int SAMPLE_COUNT = 20;
    constexpr float STEP = 8.0f;
    constexpr int RINGS = 14;

    for (int ring = 1; ring <= RINGS; ++ring) {
        const float r = ring * STEP;
        for (int i = 0; i < SAMPLE_COUNT; ++i) {
            const double ang =
                (static_cast<double>(i) / static_cast<double>(SAMPLE_COUNT)) * 2.0 * M_PI;
            const Vec2 cand{
                static_cast<int>(std::lround(desired.x + std::cos(ang) * r)),
                static_cast<int>(std::lround(desired.y + std::sin(ang) * r))
            };

            if (GetContainingPolygonIndexAt(cand.x, cand.y, quantized) < 0) {
                continue;
            }

            const double d2 = DistanceSquared(desired, cand);
            if (d2 < bestDistSq) {
                bestDistSq = d2;
                outPoint = cand;
                found = true;
            }
        }

        if (found) {
            return true;
        }
    }

    return false;
}
