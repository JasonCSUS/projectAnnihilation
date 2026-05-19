#ifndef NAVMESH_H
#define NAVMESH_H

#include <vector>
#include <string>
#include <cstdint>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <unordered_map>
#include <memory>
#include <SDL3/SDL.h>
#include <algorithm>

// DO NOT include NavMeshBuckets.h here

struct Vec2 {
    int x;
    int y;

    bool operator==(const Vec2& other) const {
        return x == other.x && y == other.y;
    }

    bool operator!=(const Vec2& other) const {
        return !(*this == other);
    }
};

struct Portal {
    Vec2 left;
    Vec2 right;
};

struct NavPolygon {
    std::vector<Vec2> vertices;
    std::vector<int> neighborIndices;
};

struct NavWallSegment {
    Vec2 a{};
    Vec2 b{};
    int ownerPoly = -1;
    bool fromRuntimeBlocker = false;
    std::string toggleId;
};

class NavMeshBuckets;

class NavMesh {
public:
    friend class NavMeshBuckets;
    static NavMesh& Instance();
    ~NavMesh();

    bool LoadFromFile(const std::string& filename);
    void Clear();

    bool LoadRuntimeBlockersFromJson(const std::string& jsonFilename);
    bool SetBlockerEnabled(const std::string& toggleId, bool enabled);
    bool IsBlockerEnabled(const std::string& toggleId) const;
    uint64_t GetBlockerRevision() const;

    void InitializeClearanceBuckets(const std::vector<int>& radii);
    void InitializeClearanceBuckets(int firstRadius, int bucketCount, int step);
    void QueueBucketRebuild();

    std::vector<int> FindPath(int startIndex, int goalIndex, int clearanceBucket = 0);

    void DebugRender(SDL_Renderer* renderer, float cameraX, float cameraY) const;

    int GetPolygonIndexAt(int x, int y, int clearanceBucket = 0) const;
    Vec2 ClampToNavMesh(const Vec2& pt, int clearanceBucket = 0);
    bool IsPointWalkable(const Vec2& pt, int clearanceBucket = 0) const;
    bool IsPolygonEnabled(int polyIndex) const;

    const std::vector<NavWallSegment>& GetBoundaryWalls() const { return boundaryWalls; }
    const std::vector<NavWallSegment>& GetActiveDynamicWalls() const { return activeDynamicWalls; }
    void GetAllActiveWalls(std::vector<NavWallSegment>& outWalls) const;

    bool ResolveSoftCollision(const Vec2& center,
                              float radius,
                              float& outPushX,
                              float& outPushY) const;

    static int QuantizeClearanceBucket(int radius);
    static uint32_t GetPathCacheNavVersion();

    std::vector<Vec2> FunnelPath(const std::vector<int>& polyPath,
                                 const Vec2& startPt,
                                 const Vec2& goalPt,
                                 int clearanceBucket = 0);

    bool HasLineOfSight(const Vec2& a, const Vec2& b, int clearanceBucket = 0) const;
    Vec2 GetPolygonCentroid(int polyIndex) const;
    const NavPolygon* GetPolygon(int polyIndex) const;
    bool GetSharedPortal(int polyAIndex, int polyBIndex, Vec2& outA, Vec2& outB) const;
    uint64_t GetPathCacheMapVersion() const;
    const std::vector<int>& GetConfiguredBuckets() const { return configuredBuckets; }

    struct BucketPortalSample {
        Vec2 point{};
        float landingPenalty = 0.0f;
    };

    struct BucketPortalData {
        Vec2 left{};
        Vec2 right{};
        std::vector<BucketPortalSample> samples;
        bool valid = false;
    };

    struct BucketView {
        int radius = 0;
        std::vector<std::vector<int>> traversableNeighbors;
        std::unordered_map<uint64_t, BucketPortalData> portalData;
    };

private:
    struct RuntimeBlocker {
        std::string label;
        std::string toggleId;
        std::string kind;
        bool enabled = false;
        float x = 0.0f;
        float y = 0.0f;
        float w = 0.0f;
        float h = 0.0f;
        std::vector<int> cellIds;
    };

private:
    NavMesh() = default;
    NavMesh(const NavMesh&) = delete;
    NavMesh& operator=(const NavMesh&) = delete;

    void BuildBoundaryWalls();
    void BuildSharedPortalMap();
    void RebuildDynamicBlockerState();
    void BuildDynamicWallsFromDisabledPolys();
    void BuildFallbackShapeWallsForEnabledBlockers();
    bool LoadExplicitWallEdgesFromJson(const std::string& jsonText);

    int GetContainingPolygonIndexAt(int x, int y, int clearanceBucket = 0) const;
    bool IsPointClearOfWalls(const Vec2& pt, int clearanceBucket) const;
    bool IsSegmentClearOfWalls(const Vec2& a, const Vec2& b, int clearanceBucket) const;
    bool FindNearbyClearPoint(const Vec2& desired, int clearanceBucket, Vec2& outPoint) const;

    bool TryGetSharedPortal(int polyAIndex,
                            int polyBIndex,
                            Vec2& outA,
                            Vec2& outB) const;
    bool IsPortalTraversableForClearance(const Vec2& a,
                                         const Vec2& b,
                                         int clearanceBucket) const;
    std::vector<Vec2> BuildClearanceAwareCorridorPath(const std::vector<Portal>& portals,
                                                      const std::vector<int>& polyPath,
                                                      int clearanceBucket) const;

    void EnsureBucketWorkerStarted();
    void StopBucketWorker();
    const BucketView* GetBucketView(int clearanceBucket,
                                    std::shared_ptr<const std::vector<BucketView>>& outSnapshot) const;
    static uint64_t MakePortalKey(int polyA, int polyB);

private:
    std::vector<NavPolygon> polygons;
    std::vector<Vec2> polygonCentroids;
    // uint8_t avoids std::vector<bool> bit-packing; elements read without the
    // lock in IsPolygonEnabled (safe: only resized at load time / blocker toggle,
    // never during normal gameplay).
    std::vector<uint8_t> polygonEnabled;
    std::vector<NavWallSegment> boundaryWalls;
    std::vector<NavWallSegment> activeDynamicWalls;
    std::vector<NavWallSegment> exportedBlockerWalls;
    std::vector<RuntimeBlocker> runtimeBlockers;

    // Precomputed portal endpoints for every adjacent polygon pair.
    // Key: MakePortalKey(polyA, polyB). Populated by BuildSharedPortalMap() at load time.
    std::unordered_map<uint64_t, std::pair<Vec2, Vec2>> sharedPortals;

    // Spatial index over boundaryWalls + activeDynamicWalls.
    // Rebuilt whenever either wall list changes; queried by all per-frame collision functions.
    std::vector<NavWallSegment> wallGridWalls;
    std::unordered_map<long long, std::vector<int>> wallGridCells;
    void RebuildWallGrid();
    void GetWallCandidates(float minX, float minY, float maxX, float maxY,
                           std::vector<int>& out) const;
    bool hasExplicitWallEdges = false;
    uint64_t blockerRevision = 0;

    mutable std::mutex bucketMutex;
    std::condition_variable bucketCv;
    std::thread bucketWorker;
    bool bucketWorkerStarted = false;
    bool stopBucketWorkerFlag = false;
    bool bucketRebuildRequested = false;

    std::vector<int> configuredBuckets;
    // Frozen copy of configuredBuckets written once (under lock) after
    // InitializeClearanceBuckets, then read lock-free from any thread.
    std::vector<int> readOnlyBuckets;
    std::shared_ptr<std::vector<BucketView>> bucketViewsSnapshot;
    uint64_t bucketViewsRevision = 0;
};

#endif