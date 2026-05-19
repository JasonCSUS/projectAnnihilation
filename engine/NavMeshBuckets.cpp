#include "NavMeshBuckets.h"
#include "NavMeshInternal.h"

#include <cmath>
#include <memory>
#include <unordered_set>

using namespace navmesh_internal;

namespace {
int QuantizeAgainstList(int radius, const std::vector<int>& buckets) {
    if (radius <= 0) {
        return 0;
    }

    if (buckets.empty()) {
        if (radius <= 20) return 20;
        if (radius <= 30) return 30;
        return 40;
    }

    for (int bucket : buckets) {
        if (radius <= bucket) {
            return bucket;
        }
    }
    return buckets.back();
}
}

const NavMesh::BucketView* NavMeshBuckets::GetBucketView(
    const NavMesh& navMesh,
    int clearanceBucket,
    std::shared_ptr<const std::vector<NavMesh::BucketView>>& outSnapshot) {
    std::lock_guard<std::mutex> lock(navMesh.bucketMutex);

    const int quantized = QuantizeAgainstList(clearanceBucket, navMesh.configuredBuckets);
    if (quantized <= 0) {
        outSnapshot.reset();
        return nullptr;
    }
    if (navMesh.bucketViewsRevision != navMesh.blockerRevision || !navMesh.bucketViewsSnapshot) {
        outSnapshot.reset();
        return nullptr;
    }

    outSnapshot = navMesh.bucketViewsSnapshot;
    for (const auto& view : *outSnapshot) {
        if (view.radius == quantized) {
            return &view;
        }
    }

    outSnapshot.reset();
    return nullptr;
}

void NavMeshBuckets::BuildRuntimeStateFromCurrentData(
    const NavMesh& navMesh,
    std::vector<uint8_t>& outPolygonEnabled,
    std::vector<NavWallSegment>& outActiveDynamicWalls,
    std::vector<NavMesh::BucketView>& outBucketViews) {
    std::vector<NavPolygon> localPolygons;
    std::vector<NavWallSegment> localBoundaryWalls;
    std::vector<NavWallSegment> localExportedBlockerWalls;
    std::vector<NavMesh::RuntimeBlocker> localRuntimeBlockers;
    std::vector<int> localBuckets;
    bool localHasExplicitWallEdges = false;

    {
        std::lock_guard<std::mutex> lock(navMesh.bucketMutex);
        localPolygons = navMesh.polygons;
        localBoundaryWalls = navMesh.boundaryWalls;
        localExportedBlockerWalls = navMesh.exportedBlockerWalls;
        localRuntimeBlockers = navMesh.runtimeBlockers;
        localHasExplicitWallEdges = navMesh.hasExplicitWallEdges;
        localBuckets = navMesh.configuredBuckets;
    }

    outPolygonEnabled.assign(localPolygons.size(), 1);
    outActiveDynamicWalls.clear();

    for (const auto& blocker : localRuntimeBlockers) {
        if (!blocker.enabled) {
            continue;
        }

        for (int cellId : blocker.cellIds) {
            if (cellId >= 0 && cellId < static_cast<int>(outPolygonEnabled.size())) {
                outPolygonEnabled[cellId] = 0;
            }
        }
    }

    if (localHasExplicitWallEdges) {
        for (const auto& wall : localExportedBlockerWalls) {
            if (wall.toggleId.empty()) {
                continue;
            }

            bool blockerEnabled = false;
            for (const auto& blocker : localRuntimeBlockers) {
                if (blocker.toggleId == wall.toggleId) {
                    blockerEnabled = blocker.enabled;
                    break;
                }
            }

            if (!blockerEnabled) {
                continue;
            }

            if (wall.ownerPoly >= 0 &&
                wall.ownerPoly < static_cast<int>(outPolygonEnabled.size()) &&
                !outPolygonEnabled[wall.ownerPoly]) {
                continue;
            }

            outActiveDynamicWalls.push_back(wall);
        }

        for (const auto& blocker : localRuntimeBlockers) {
            if (!blocker.enabled || !blocker.cellIds.empty()) {
                continue;
            }

            if (blocker.kind == "rect") {
                BuildRectWalls(blocker.x, blocker.y, blocker.w, blocker.h,
                                       -1, blocker.toggleId, outActiveDynamicWalls);
            } else if (blocker.kind == "circle") {
                BuildCircleWalls(blocker.x, blocker.y, blocker.w, blocker.h,
                                         -1, blocker.toggleId, outActiveDynamicWalls);
            }
        }
    } else {
        std::unordered_set<std::size_t> seen;

        auto makeWallHash = [](const Vec2& a, const Vec2& b, int owner) -> std::size_t {
            const EdgeKey k = MakeCanonicalEdgeKey(a, b);
            std::size_t h = EdgeKeyHash{}(k);
            h ^= (static_cast<std::size_t>(owner) << 1);
            return h;
        };

        for (size_t polyIndex = 0; polyIndex < localPolygons.size(); ++polyIndex) {
            if (!outPolygonEnabled[polyIndex]) {
                continue;
            }

            const auto& poly = localPolygons[polyIndex];
            for (size_t edgeIndex = 0; edgeIndex < poly.vertices.size(); ++edgeIndex) {
                const int neighbor =
                    (edgeIndex < poly.neighborIndices.size()) ? poly.neighborIndices[edgeIndex] : -1;

                if (neighbor < 0 ||
                    neighbor >= static_cast<int>(localPolygons.size()) ||
                    outPolygonEnabled[neighbor]) {
                    continue;
                }

                const Vec2& a = poly.vertices[edgeIndex];
                const Vec2& b = poly.vertices[(edgeIndex + 1) % poly.vertices.size()];
                const std::size_t wallHash = makeWallHash(a, b, static_cast<int>(polyIndex));

                if (!seen.insert(wallHash).second) {
                    continue;
                }

                outActiveDynamicWalls.push_back({a, b, static_cast<int>(polyIndex), true, ""});
            }
        }

        for (const auto& blocker : localRuntimeBlockers) {
            if (!blocker.enabled || !blocker.cellIds.empty()) {
                continue;
            }

            if (blocker.kind == "rect") {
                BuildRectWalls(blocker.x, blocker.y, blocker.w, blocker.h,
                                       -1, blocker.toggleId, outActiveDynamicWalls);
            } else if (blocker.kind == "circle") {
                BuildCircleWalls(blocker.x, blocker.y, blocker.w, blocker.h,
                                         -1, blocker.toggleId, outActiveDynamicWalls);
            }
        }
    }

    outBucketViews.clear();
    if (localBuckets.empty()) {
        return;
    }

    auto isPointClearLocal = [&](const Vec2& pt, int clearanceBucket) -> bool {
        if (clearanceBucket <= 0) {
            return true;
        }

        for (const auto& wall : localBoundaryWalls) {
            if (DistancePointToSegment(wall.a, wall.b, pt) < static_cast<float>(clearanceBucket)) {
                return false;
            }
        }
        for (const auto& wall : outActiveDynamicWalls) {
            if (DistancePointToSegment(wall.a, wall.b, pt) < static_cast<float>(clearanceBucket)) {
                return false;
            }
        }
        return true;
    };

    auto tryGetSharedPortalLocal = [&](int polyAIndex, int polyBIndex, Vec2& outA, Vec2& outB) -> bool {
        return navMesh.TryGetSharedPortal(polyAIndex, polyBIndex, outA, outB);
    };

    auto buildPortalData = [&](const Vec2& a, const Vec2& b, int bucketRadius) -> NavMesh::BucketPortalData {
        NavMesh::BucketPortalData data;
        data.left = a;
        data.right = b;

        if (bucketRadius <= 0) {
            data.valid = true;
            return data;
        }

        const Portal portal{a, b};
        const Portal inset = InsetPortal(portal, static_cast<double>(bucketRadius));

        const double dx = static_cast<double>(inset.right.x - inset.left.x);
        const double dy = static_cast<double>(inset.right.y - inset.left.y);
        const double len = std::sqrt(dx * dx + dy * dy);

        if (len <= 1.0) {
            if (isPointClearLocal(inset.left, bucketRadius)) {
                data.samples.push_back({inset.left, 0.0f});
                data.valid = true;
            }
            return data;
        }

        static const double TS[] = {0.0, 0.2, 0.4, 0.5, 0.6, 0.8, 1.0};
        for (double t : TS) {
            const Vec2 p = {
                static_cast<int>(std::lround(inset.left.x + dx * t)),
                static_cast<int>(std::lround(inset.left.y + dy * t))
            };

            if (!isPointClearLocal(p, bucketRadius)) {
                continue;
            }

            bool duplicate = false;
            for (const auto& sample : data.samples) {
                if (sample.point.x == p.x && sample.point.y == p.y) {
                    duplicate = true;
                    break;
                }
            }
            if (duplicate) {
                continue;
            }

            data.samples.push_back({p, static_cast<float>(std::abs(t - 0.5) * 10.0)});
        }

        data.valid = !data.samples.empty();
        return data;
    };

    outBucketViews.reserve(localBuckets.size());

    for (int bucketRadius : localBuckets) {
        NavMesh::BucketView view;
        view.radius = bucketRadius;
        view.traversableNeighbors.resize(localPolygons.size());

        for (size_t polyIndex = 0; polyIndex < localPolygons.size(); ++polyIndex) {
            if (!outPolygonEnabled[polyIndex]) {
                continue;
            }

            const auto& poly = localPolygons[polyIndex];
            for (int neighbor : poly.neighborIndices) {
                if (neighbor < 0 ||
                    neighbor >= static_cast<int>(localPolygons.size()) ||
                    !outPolygonEnabled[neighbor]) {
                    continue;
                }

                Vec2 sharedA{};
                Vec2 sharedB{};
                if (!tryGetSharedPortalLocal(static_cast<int>(polyIndex), neighbor, sharedA, sharedB)) {
                    continue;
                }

                NavMesh::BucketPortalData portalData = buildPortalData(sharedA, sharedB, bucketRadius);
                if (!portalData.valid) {
                    continue;
                }

                view.traversableNeighbors[polyIndex].push_back(neighbor);
                view.portalData[NavMesh::MakePortalKey(static_cast<int>(polyIndex), neighbor)] = std::move(portalData);
            }
        }

        outBucketViews.push_back(std::move(view));
    }
}

void NavMeshBuckets::BucketWorkerLoop(NavMesh& navMesh) {
    while (true) {
        {
            std::unique_lock<std::mutex> lock(navMesh.bucketMutex);
            navMesh.bucketCv.wait(lock, [&]() {
                return navMesh.stopBucketWorkerFlag || navMesh.bucketRebuildRequested;
            });

            if (navMesh.stopBucketWorkerFlag) { 
                return;
            }

            navMesh.bucketRebuildRequested = false;
        }

        std::vector<uint8_t> nextEnabled;
        std::vector<NavWallSegment> nextWalls;
        std::vector<NavMesh::BucketView> nextViews;
        BuildRuntimeStateFromCurrentData(navMesh, nextEnabled, nextWalls, nextViews);

        {
            std::lock_guard<std::mutex> lock(navMesh.bucketMutex);
            navMesh.polygonEnabled = std::move(nextEnabled);
            navMesh.activeDynamicWalls = std::move(nextWalls);
            navMesh.bucketViewsSnapshot =
                std::make_shared<std::vector<NavMesh::BucketView>>(std::move(nextViews));
            navMesh.blockerRevision += 1;
            navMesh.bucketViewsRevision = navMesh.blockerRevision;
        }

        navMesh.RebuildWallGrid();
    }
}