#include "NavMesh.h"
#include "NavMeshInternal.h"
#include "NavMeshBuckets.h"

#include <fstream>
#include <iostream>
#include <unordered_map>
#include <unordered_set>

using namespace navmesh_internal;

bool NavMesh::LoadExplicitWallEdgesFromJson(const std::string& jsonText) {
    std::vector<NavWallSegment> parsedBoundaryWalls;
    std::vector<NavWallSegment> parsedBlockerWalls;

    std::string wallsBody;
    if (!ExtractSectionArray(jsonText, "wall_edges", wallsBody)) {
        std::lock_guard<std::mutex> lock(bucketMutex);
        exportedBlockerWalls.clear();
        hasExplicitWallEdges = false;
        return false;
    }

    for (const std::string& obj : SplitTopLevelObjects(wallsBody)) {
        NavWallSegment wall;
        if (!ExtractIntField(obj, "ax", wall.a.x) ||
            !ExtractIntField(obj, "ay", wall.a.y) ||
            !ExtractIntField(obj, "bx", wall.b.x) ||
            !ExtractIntField(obj, "by", wall.b.y)) {
            continue;
        }

        ExtractIntField(obj, "owner_poly", wall.ownerPoly);

        std::string wallType;
        ExtractStringField(obj, "wall_type", wallType);
        ExtractStringField(obj, "toggle_id", wall.toggleId);

        if (wallType == "blocker") {
            wall.fromRuntimeBlocker = true;
            parsedBlockerWalls.push_back(std::move(wall));
        } else {
            wall.fromRuntimeBlocker = false;
            parsedBoundaryWalls.push_back(std::move(wall));
        }
    }

    std::lock_guard<std::mutex> lock(bucketMutex);
    if (!parsedBoundaryWalls.empty()) {
        boundaryWalls = std::move(parsedBoundaryWalls);
    }
    exportedBlockerWalls = std::move(parsedBlockerWalls);
    hasExplicitWallEdges = true;
    return true;
}

bool NavMesh::LoadRuntimeBlockersFromJson(const std::string& jsonFilename) {
    std::ifstream in(jsonFilename);
    if (!in) {
        std::cerr << "NavMesh: failed to open runtime blocker json: " << jsonFilename << " ";
        return false;
    }

    std::string json((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());

    LoadExplicitWallEdgesFromJson(json);

    std::vector<RuntimeBlocker> loadedBlockers;

    std::string blockersBody;
    if (ExtractSectionArray(json, "runtime_blockers", blockersBody)) {
        for (const std::string& obj : SplitTopLevelObjects(blockersBody)) {
            RuntimeBlocker blocker;
            ExtractStringField(obj, "label", blocker.label);
            if (!ExtractStringField(obj, "toggle_id", blocker.toggleId)) {
                blocker.toggleId = blocker.label;
            }
            ExtractStringField(obj, "kind", blocker.kind);
            ExtractBoolField(obj, "enabled_on_start", blocker.enabled);
            ExtractFloatField(obj, "x", blocker.x);
            ExtractFloatField(obj, "y", blocker.y);
            ExtractFloatField(obj, "w", blocker.w);
            ExtractFloatField(obj, "h", blocker.h);
            ExtractIntArrayField(obj, "cell_ids", blocker.cellIds);

            loadedBlockers.push_back(std::move(blocker));
        }
    }

    {
        std::lock_guard<std::mutex> lock(bucketMutex);
        runtimeBlockers = std::move(loadedBlockers);
    }

    RebuildDynamicBlockerState();
    return true;
}

bool NavMesh::SetBlockerEnabled(const std::string& toggleId, bool enabled) {
    bool changed = false;
    bool hasBuckets = false;

    {
        std::lock_guard<std::mutex> lock(bucketMutex);
        for (auto& blocker : runtimeBlockers) {
            if (blocker.toggleId == toggleId) {
                if (blocker.enabled == enabled) {
                    return true;
                }
                blocker.enabled = enabled;
                changed = true;
                break;
            }
        }

        hasBuckets = !configuredBuckets.empty();
    }

    if (!changed) {
        return false;
    }

    if (hasBuckets) {
        QueueBucketRebuild();
    } else {
        RebuildDynamicBlockerState();
    }

    return true;
}

bool NavMesh::IsBlockerEnabled(const std::string& toggleId) const {
    std::lock_guard<std::mutex> lock(bucketMutex);
    for (const auto& blocker : runtimeBlockers) {
        if (blocker.toggleId == toggleId) {
            return blocker.enabled;
        }
    }
    return false;
}

uint64_t NavMesh::GetBlockerRevision() const {
    std::lock_guard<std::mutex> lock(bucketMutex);
    return blockerRevision;
}

void NavMesh::BuildBoundaryWalls() {
    boundaryWalls.clear();

    std::unordered_map<EdgeKey, std::vector<EdgeOwner>, EdgeKeyHash> edgeOwners;

    for (size_t polyIndex = 0; polyIndex < polygons.size(); ++polyIndex) {
        const auto& poly = polygons[polyIndex];
        for (size_t edgeIndex = 0; edgeIndex < poly.vertices.size(); ++edgeIndex) {
            const Vec2& a = poly.vertices[edgeIndex];
            const Vec2& b = poly.vertices[(edgeIndex + 1) % poly.vertices.size()];
            edgeOwners[MakeCanonicalEdgeKey(a, b)].push_back(
                {static_cast<int>(polyIndex), static_cast<int>(edgeIndex)}
            );
        }
    }

    for (const auto& [key, owners] : edgeOwners) {
        (void)key;
        if (owners.size() != 1) continue;

        const EdgeOwner owner = owners.front();
        const auto& poly = polygons[owner.polyIndex];
        const Vec2& a = poly.vertices[owner.edgeIndex];
        const Vec2& b = poly.vertices[(owner.edgeIndex + 1) % poly.vertices.size()];
        boundaryWalls.push_back({a, b, owner.polyIndex, false, ""});
    }
}

void NavMesh::BuildSharedPortalMap() {
    sharedPortals.clear();

    std::unordered_map<EdgeKey, std::vector<EdgeOwner>, EdgeKeyHash> edgeOwners;

    for (size_t polyIndex = 0; polyIndex < polygons.size(); ++polyIndex) {
        const auto& poly = polygons[polyIndex];
        for (size_t edgeIndex = 0; edgeIndex < poly.vertices.size(); ++edgeIndex) {
            const Vec2& a = poly.vertices[edgeIndex];
            const Vec2& b = poly.vertices[(edgeIndex + 1) % poly.vertices.size()];
            edgeOwners[MakeCanonicalEdgeKey(a, b)].push_back(
                {static_cast<int>(polyIndex), static_cast<int>(edgeIndex)}
            );
        }
    }

    for (const auto& [key, owners] : edgeOwners) {
        (void)key;
        if (owners.size() != 2) continue;

        const int polyA = owners[0].polyIndex;
        const int polyB = owners[1].polyIndex;
        const auto& vA = polygons[polyA].vertices;
        const Vec2 a1 = vA[owners[0].edgeIndex];
        const Vec2 a2 = vA[(owners[0].edgeIndex + 1) % vA.size()];

        sharedPortals[MakePortalKey(polyA, polyB)] = {a1, a2};
        sharedPortals[MakePortalKey(polyB, polyA)] = {a2, a1};
    }
}

void NavMesh::BuildDynamicWallsFromDisabledPolys() {
    activeDynamicWalls.clear();
}

void NavMesh::BuildFallbackShapeWallsForEnabledBlockers() {
}

void NavMesh::RebuildDynamicBlockerState() {
    std::vector<uint8_t> nextEnabled;
    std::vector<NavWallSegment> nextWalls;
    std::vector<BucketView> nextViews;
    NavMeshBuckets::BuildRuntimeStateFromCurrentData(*this, nextEnabled, nextWalls, nextViews);

    {
        std::lock_guard<std::mutex> lock(bucketMutex);
        polygonEnabled = std::move(nextEnabled);
        activeDynamicWalls = std::move(nextWalls);
        bucketViewsSnapshot = std::make_shared<std::vector<BucketView>>(std::move(nextViews));
        blockerRevision += 1;
        bucketViewsRevision = blockerRevision;
    }

    RebuildWallGrid();
}
