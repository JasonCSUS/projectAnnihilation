#include "NavMesh.h"
#include "NavMeshInternal.h"
#include "NavMeshBuckets.h"

#include <algorithm>
#include <cstring>
#include <fstream>
#include <iostream>
#include <unordered_set>

using namespace navmesh_internal;

namespace {
constexpr uint32_t PATH_CACHE_NAV_VERSION = 2;

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

void HashMixU64(uint64_t& h, uint64_t v) {
    h ^= v + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
}

void HashMixI32(uint64_t& h, int v) {
    HashMixU64(h, static_cast<uint64_t>(static_cast<int64_t>(v)));
}

void HashMixF32(uint64_t& h, float v) {
    uint32_t bits = 0;
    std::memcpy(&bits, &v, sizeof(bits));
    HashMixU64(h, bits);
}

void HashMixString(uint64_t& h, const std::string& s) {
    for (unsigned char c : s) {
        HashMixU64(h, c);
    }
    HashMixU64(h, 0xff);
}

} // namespace

NavMesh& NavMesh::Instance() {
    static NavMesh instance;
    return instance;
}

NavMesh::~NavMesh() {
    StopBucketWorker();
}

uint32_t NavMesh::GetPathCacheNavVersion() {
    return PATH_CACHE_NAV_VERSION;
}

uint64_t NavMesh::MakePortalKey(int polyA, int polyB) {
    return (static_cast<uint64_t>(static_cast<uint32_t>(polyA)) << 32) |
           static_cast<uint32_t>(polyB);
}

void NavMesh::EnsureBucketWorkerStarted() {
    std::lock_guard<std::mutex> lock(bucketMutex);
    if (bucketWorkerStarted) {
        return;
    }

    stopBucketWorkerFlag = false;
    bucketRebuildRequested = false;
    bucketWorker = std::thread(&NavMeshBuckets::BucketWorkerLoop, std::ref(*this));
    bucketWorkerStarted = true;
}

void NavMesh::StopBucketWorker() {
    {
        std::lock_guard<std::mutex> lock(bucketMutex);
        if (!bucketWorkerStarted) {
            return;
        }
        stopBucketWorkerFlag = true;
        bucketRebuildRequested = true;
    }

    bucketCv.notify_all();

    if (bucketWorker.joinable()) {
        bucketWorker.join();
    }

    std::lock_guard<std::mutex> lock(bucketMutex);
    bucketWorkerStarted = false;
    stopBucketWorkerFlag = false;
    bucketRebuildRequested = false;
}

void NavMesh::InitializeClearanceBuckets(const std::vector<int>& radii) {
    std::vector<int> normalized;
    normalized.reserve(radii.size());

    for (int r : radii) {
        if (r > 0) {
            normalized.push_back(r);
        }
    }

    std::sort(normalized.begin(), normalized.end());
    normalized.erase(std::unique(normalized.begin(), normalized.end()), normalized.end());

    {
        std::lock_guard<std::mutex> lock(bucketMutex);
        configuredBuckets = std::move(normalized);
        readOnlyBuckets = configuredBuckets;
    }

    EnsureBucketWorkerStarted();
    RebuildDynamicBlockerState();
}

void NavMesh::InitializeClearanceBuckets(int firstRadius, int bucketCount, int step) {
    std::vector<int> radii;
    radii.reserve(std::max(0, bucketCount));

    int value = firstRadius;
    for (int i = 0; i < bucketCount; ++i) {
        if (value > 0) {
            radii.push_back(value);
        }
        value += step;
    }

    InitializeClearanceBuckets(radii);
}

void NavMesh::QueueBucketRebuild() {
    bool shouldNotify = false;
    {
        std::lock_guard<std::mutex> lock(bucketMutex);
        if (configuredBuckets.empty()) {
            return;
        }

        if (!bucketWorkerStarted) {
            stopBucketWorkerFlag = false;
            bucketRebuildRequested = false;
            bucketWorker = std::thread(&NavMeshBuckets::BucketWorkerLoop, std::ref(*this));
            bucketWorkerStarted = true;
        }

        bucketRebuildRequested = true;
        shouldNotify = true;
    }

    if (shouldNotify) {
        bucketCv.notify_one();
    }
}

bool NavMesh::LoadFromFile(const std::string& filename) {
    Clear();

    std::ifstream file(filename, std::ios::binary);
    if (!file) {
        std::cerr << "Error opening navmesh file: " << filename << "\n";
        return false;
    }

    int numPolygons = 0;
    file.read(reinterpret_cast<char*>(&numPolygons), sizeof(int));
    if (numPolygons <= 0) {
        std::cerr << "Invalid polygon count in navmesh file.\n";
        return false;
    }

    std::vector<NavPolygon> loadedPolys;
    std::vector<Vec2> loadedCentroids;
    loadedPolys.resize(numPolygons);
    loadedCentroids.resize(numPolygons);

    for (int i = 0; i < numPolygons; ++i) {
        int vertexCount = 0;
        file.read(reinterpret_cast<char*>(&vertexCount), sizeof(int));
        if (vertexCount < 3) {
            std::cerr << "Polygon " << i << " has fewer than 3 vertices.\n";
            return false;
        }

        NavPolygon poly;
        poly.vertices.resize(vertexCount);

        for (int j = 0; j < vertexCount; ++j) {
            int x = 0;
            int y = 0;
            file.read(reinterpret_cast<char*>(&x), sizeof(int));
            file.read(reinterpret_cast<char*>(&y), sizeof(int));
            poly.vertices[j] = {x, y};
        }

        int neighborCount = 0;
        file.read(reinterpret_cast<char*>(&neighborCount), sizeof(int));
        poly.neighborIndices.resize(neighborCount);
        for (int j = 0; j < neighborCount; ++j) {
            file.read(reinterpret_cast<char*>(&poly.neighborIndices[j]), sizeof(int));
        }

        loadedCentroids[i] = ComputeCentroid(poly.vertices);
        loadedPolys[i] = std::move(poly);
    }

    {
        std::lock_guard<std::mutex> lock(bucketMutex);
        polygons = std::move(loadedPolys);
        polygonCentroids = std::move(loadedCentroids);
        polygonEnabled.assign(polygons.size(), 1);
        activeDynamicWalls.clear();
        exportedBlockerWalls.clear();
        runtimeBlockers.clear();
        hasExplicitWallEdges = false;
        blockerRevision = 0;
        bucketViewsSnapshot.reset();
        bucketViewsRevision = 0;
        BuildBoundaryWalls();
        BuildSharedPortalMap();
    }

    RebuildDynamicBlockerState();

    std::cout << "Loaded navmesh with " << numPolygons << " polygons.\n";
    return true;
}

void NavMesh::Clear() {
    {
        std::lock_guard<std::mutex> lock(bucketMutex);
        polygons.clear();
        polygonCentroids.clear();
        polygonEnabled.clear();
        boundaryWalls.clear();
        activeDynamicWalls.clear();
        runtimeBlockers.clear();
        exportedBlockerWalls.clear();
        hasExplicitWallEdges = false;
        blockerRevision = 0;
        bucketViewsSnapshot.reset();
        bucketViewsRevision = 0;
    }
    wallGridWalls.clear();
    wallGridCells.clear();
    sharedPortals.clear();
}

void NavMesh::DebugRender(SDL_Renderer* renderer, float cameraX, float cameraY) const {
    for (size_t polyIndex = 0; polyIndex < polygons.size(); ++polyIndex) {
        const auto& poly = polygons[polyIndex];
        const bool enabled = IsPolygonEnabled(static_cast<int>(polyIndex));

        if (enabled) SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
        else SDL_SetRenderDrawColor(renderer, 120, 40, 40, 255);

        const size_t numVerts = poly.vertices.size();
        for (size_t i = 0; i < numVerts; ++i) {
            const Vec2& a = poly.vertices[i];
            const Vec2& b = poly.vertices[(i + 1) % numVerts];
            SDL_RenderLine(renderer,
                           static_cast<float>(a.x - cameraX),
                           static_cast<float>(a.y - cameraY),
                           static_cast<float>(b.x - cameraX),
                           static_cast<float>(b.y - cameraY));
        }
    }

    SDL_SetRenderDrawColor(renderer, 255, 80, 80, 255);
    for (const auto& wall : boundaryWalls) {
        SDL_RenderLine(renderer,
                       static_cast<float>(wall.a.x - cameraX),
                       static_cast<float>(wall.a.y - cameraY),
                       static_cast<float>(wall.b.x - cameraX),
                       static_cast<float>(wall.b.y - cameraY));
    }

    SDL_SetRenderDrawColor(renderer, 255, 180, 40, 255);
    for (const auto& wall : activeDynamicWalls) {
        SDL_RenderLine(renderer,
                       static_cast<float>(wall.a.x - cameraX),
                       static_cast<float>(wall.a.y - cameraY),
                       static_cast<float>(wall.b.x - cameraX),
                       static_cast<float>(wall.b.y - cameraY));
    }
}

int NavMesh::QuantizeClearanceBucket(int radius) {
    // readOnlyBuckets is written once at startup (before workers start) and
    // then only read — no lock needed.
    return QuantizeAgainstList(radius, Instance().readOnlyBuckets);
}

const NavMesh::BucketView* NavMesh::GetBucketView(
    int clearanceBucket,
    std::shared_ptr<const std::vector<BucketView>>& outSnapshot) const {
    return NavMeshBuckets::GetBucketView(*this, clearanceBucket, outSnapshot);
}

Vec2 NavMesh::GetPolygonCentroid(int polyIndex) const {
    std::lock_guard<std::mutex> lock(bucketMutex);
    if (polyIndex < 0 || polyIndex >= static_cast<int>(polygonCentroids.size())) {
        return {0, 0};
    }
    return polygonCentroids[polyIndex];
}

bool NavMesh::HasLineOfSight(const Vec2& a, const Vec2& b, int clearanceBucket) const {
    return IsSegmentClearOfWalls(a, b, clearanceBucket);
}


const NavPolygon* NavMesh::GetPolygon(int polyIndex) const {
    if (polyIndex < 0 || polyIndex >= static_cast<int>(polygons.size())) return nullptr;
    return &polygons[polyIndex];
}

bool NavMesh::GetSharedPortal(int polyAIndex, int polyBIndex, Vec2& outA, Vec2& outB) const {
    return TryGetSharedPortal(polyAIndex, polyBIndex, outA, outB);
}

uint64_t NavMesh::GetPathCacheMapVersion() const {
    std::lock_guard<std::mutex> lock(bucketMutex);

    uint64_t h = 1469598103934665603ull;

    HashMixU64(h, 0x4e41564d45534831ull);
    HashMixU64(h, polygons.size());
    for (const auto& poly : polygons) {
        HashMixU64(h, poly.vertices.size());
        for (const auto& v : poly.vertices) {
            HashMixI32(h, v.x);
            HashMixI32(h, v.y);
        }
        HashMixU64(h, poly.neighborIndices.size());
        for (int n : poly.neighborIndices) {
            HashMixI32(h, n);
        }
    }

    HashMixU64(h, configuredBuckets.size());
    for (int b : configuredBuckets) {
        HashMixI32(h, b);
    }

    HashMixU64(h, runtimeBlockers.size());
    for (const auto& blocker : runtimeBlockers) {
        HashMixString(h, blocker.label);
        HashMixString(h, blocker.toggleId);
        HashMixString(h, blocker.kind);
        HashMixI32(h, blocker.enabled ? 1 : 0);
        HashMixF32(h, blocker.x);
        HashMixF32(h, blocker.y);
        HashMixF32(h, blocker.w);
        HashMixF32(h, blocker.h);
        HashMixU64(h, blocker.cellIds.size());
        for (int id : blocker.cellIds) {
            HashMixI32(h, id);
        }
    }

    HashMixU64(h, boundaryWalls.size());
    for (const auto& wall : boundaryWalls) {
        HashMixI32(h, wall.a.x); HashMixI32(h, wall.a.y);
        HashMixI32(h, wall.b.x); HashMixI32(h, wall.b.y);
        HashMixI32(h, wall.ownerPoly);
        HashMixI32(h, wall.fromRuntimeBlocker ? 1 : 0);
        HashMixString(h, wall.toggleId);
    }

    HashMixU64(h, exportedBlockerWalls.size());
    for (const auto& wall : exportedBlockerWalls) {
        HashMixI32(h, wall.a.x); HashMixI32(h, wall.a.y);
        HashMixI32(h, wall.b.x); HashMixI32(h, wall.b.y);
        HashMixI32(h, wall.ownerPoly);
        HashMixI32(h, wall.fromRuntimeBlocker ? 1 : 0);
        HashMixString(h, wall.toggleId);
    }

    HashMixI32(h, hasExplicitWallEdges ? 1 : 0);
    return h;
}

