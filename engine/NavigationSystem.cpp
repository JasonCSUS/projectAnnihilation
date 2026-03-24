#include "NavigationSystem.h"
#include "EntityManager.h"
#include "NavMesh.h"
#include "GlobalTasks.h"

#include <SDL3/SDL.h>
#include <iostream>
#include <future>
#include <memory>
#include <chrono>
#include <cmath>
#include <vector>
#include <fstream>
#include <filesystem>
#include <algorithm>

namespace {
    constexpr int NAV_CACHE_FILE_VERSION = 2;
}

NavigationSystem& NavigationSystem::Instance() {
    static NavigationSystem instance;
    return instance;
}

bool NavigationSystem::InitializePathCache(const std::string& filePath) {
    if (!filePath.empty()) {
        cacheFilePath = filePath;
    }

    if (cacheFilePath.empty()) {
        return true;
    }

    if (std::filesystem::exists(cacheFilePath)) {
        return LoadPathCacheFromFile(cacheFilePath);
    }

    ClearPathCache();
    return true;
}

void NavigationSystem::SetCacheFilePath(const std::string& filePath) {
    cacheFilePath = filePath;
}

bool NavigationSystem::LoadPathCacheFromFile(const std::string& filePath) {
    std::ifstream in(filePath, std::ios::binary);
    if (!in) {
        std::cout << "NavigationSystem: path cache file not found: " << filePath << "\n";
        return false;
    }

    int version = 0;
    int entryCount = 0;

    in.read(reinterpret_cast<char*>(&version), sizeof(version));
    in.read(reinterpret_cast<char*>(&entryCount), sizeof(entryCount));

    if (!in || version != NAV_CACHE_FILE_VERSION || entryCount < 0) {
        std::cout << "NavigationSystem: invalid path cache file: " << filePath << "\n";
        return false;
    }

    std::unordered_map<PathCacheKey, CachedRoute, PathCacheKeyHash> loaded;

    for (int i = 0; i < entryCount; ++i) {
        int startPoly = -1;
        int goalPoly = -1;
        int polyCount = 0;
        int pointCount = 0;
        unsigned int hits = 0;

        in.read(reinterpret_cast<char*>(&startPoly), sizeof(startPoly));
        in.read(reinterpret_cast<char*>(&goalPoly), sizeof(goalPoly));
        in.read(reinterpret_cast<char*>(&polyCount), sizeof(polyCount));

        if (!in || polyCount <= 0) {
            std::cout << "NavigationSystem: corrupted path cache entry in " << filePath << "\n";
            return false;
        }

        CachedRoute route;
        route.polyPath.resize(polyCount);
        in.read(reinterpret_cast<char*>(route.polyPath.data()), sizeof(int) * polyCount);

        in.read(reinterpret_cast<char*>(&pointCount), sizeof(pointCount));
        if (!in || pointCount <= 0) {
            std::cout << "NavigationSystem: corrupted compressed route in " << filePath << "\n";
            return false;
        }

        route.compressedPath.resize(pointCount);
        for (int p = 0; p < pointCount; ++p) {
            in.read(reinterpret_cast<char*>(&route.compressedPath[p].x), sizeof(int));
            in.read(reinterpret_cast<char*>(&route.compressedPath[p].y), sizeof(int));
        }

        in.read(reinterpret_cast<char*>(&hits), sizeof(hits));
        route.hits = hits;

        if (!in) {
            std::cout << "NavigationSystem: failed reading cache route from " << filePath << "\n";
            return false;
        }

        loaded[{startPoly, goalPoly}] = std::move(route);
    }

    {
        std::lock_guard<std::mutex> lock(cacheMutex);
        pathCache = std::move(loaded);
        cacheDirty = false;
    }

    std::cout << "NavigationSystem: loaded " << entryCount
              << " cached routes from " << filePath << "\n";

    return true;
}

bool NavigationSystem::SavePathCacheToFile(const std::string& filePath) {
    const std::string outPath = !filePath.empty() ? filePath : cacheFilePath;
    if (outPath.empty()) {
        std::cout << "NavigationSystem: no cache file path set; skipping save.\n";
        return false;
    }

    std::unordered_map<PathCacheKey, CachedRoute, PathCacheKeyHash> snapshot;
    {
        std::lock_guard<std::mutex> lock(cacheMutex);
        snapshot = pathCache;
    }

    std::ofstream out(outPath, std::ios::binary | std::ios::trunc);
    if (!out) {
        std::cout << "NavigationSystem: failed to open cache file for writing: " << outPath << "\n";
        return false;
    }

    const int version = NAV_CACHE_FILE_VERSION;
    const int entryCount = static_cast<int>(snapshot.size());

    out.write(reinterpret_cast<const char*>(&version), sizeof(version));
    out.write(reinterpret_cast<const char*>(&entryCount), sizeof(entryCount));

    for (const auto& [key, route] : snapshot) {
        const int polyCount = static_cast<int>(route.polyPath.size());
        const int pointCount = static_cast<int>(route.compressedPath.size());

        out.write(reinterpret_cast<const char*>(&key.startPoly), sizeof(key.startPoly));
        out.write(reinterpret_cast<const char*>(&key.goalPoly), sizeof(key.goalPoly));

        out.write(reinterpret_cast<const char*>(&polyCount), sizeof(polyCount));
        out.write(reinterpret_cast<const char*>(route.polyPath.data()), sizeof(int) * polyCount);

        out.write(reinterpret_cast<const char*>(&pointCount), sizeof(pointCount));
        for (const Vec2& p : route.compressedPath) {
            out.write(reinterpret_cast<const char*>(&p.x), sizeof(p.x));
            out.write(reinterpret_cast<const char*>(&p.y), sizeof(p.y));
        }

        out.write(reinterpret_cast<const char*>(&route.hits), sizeof(route.hits));
    }

    if (!out) {
        std::cout << "NavigationSystem: failed while writing cache file: " << outPath << "\n";
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(cacheMutex);
        cacheDirty = false;
    }

    std::cout << "NavigationSystem: saved " << entryCount
              << " cached routes to " << outPath << "\n";

    return true;
}

void NavigationSystem::ClearPathCache() {
    std::lock_guard<std::mutex> lock(cacheMutex);
    pathCache.clear();
    cacheDirty = false;
}

std::size_t NavigationSystem::GetCachedRouteCount() const {
    std::lock_guard<std::mutex> lock(cacheMutex);
    return pathCache.size();
}

bool NavigationSystem::IsPathCacheDirty() const {
    std::lock_guard<std::mutex> lock(cacheMutex);
    return cacheDirty;
}

void NavigationSystem::Update(EntityManager& entityManager, float /*deltaTime*/) {
    for (auto& entity : entityManager.entities) {
        ProcessPendingPathUpdates(entity);

        if (entity.navHasMoveTarget && entity.path.empty() && !entity.hasPendingPathUpdate) {
            SchedulePathBuild(entity, entity.navTargetX, entity.navTargetY, false);
        }
    }
}

bool NavigationSystem::RequestMove(EntityManager& entityManager, int entityId, float targetX, float targetY, bool forceRepath) {
    Entity* entity = FindEntityById(entityManager, entityId);
    if (!entity) {
        std::cout << "NavigationSystem::RequestMove failed: entity "
                  << entityId << " not found.\n";
        return false;
    }

    Vec2 clampedGoal = {
        static_cast<int>(targetX),
        static_cast<int>(targetY)
    };

    clampedGoal = NavMesh::Instance().ClampToNavMesh(clampedGoal);

    entity->navHasMoveTarget = true;
    entity->navTargetX = clampedGoal.x;
    entity->navTargetY = clampedGoal.y;

    return SchedulePathBuild(*entity, clampedGoal.x, clampedGoal.y, forceRepath);
}

void NavigationSystem::StopNavigation(EntityManager& entityManager, int entityId, bool clearPath) {
    Entity* entity = FindEntityById(entityManager, entityId);
    if (!entity) {
        return;
    }

    entity->navHasMoveTarget = false;
    entity->navTargetX = -1;
    entity->navTargetY = -1;

    if (clearPath) {
        entity->path.clear();
    }
}

Entity* NavigationSystem::FindEntityById(EntityManager& entityManager, int entityId) {
    for (auto& entity : entityManager.entities) {
        if (entity.id == entityId) {
            return &entity;
        }
    }
    return nullptr;
}

void NavigationSystem::ProcessPendingPathUpdates(Entity& entity) {
    if (!entity.hasPendingPathUpdate || !entity.asyncPathFuture.valid()) {
        return;
    }

    if (entity.asyncPathFuture.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
        try {
            entity.path = entity.asyncPathFuture.get();
        } catch (const std::exception& e) {
            std::cout << "Navigation async path build failed for entity "
                      << entity.id << ": " << e.what() << "\n";
            entity.path.clear();
        } catch (...) {
            std::cout << "Navigation async path build failed for entity "
                      << entity.id << ": unknown exception\n";
            entity.path.clear();
        }

        entity.hasPendingPathUpdate = false;
    }
}

bool NavigationSystem::TryGetCachedRoute(int startPoly, int goalPoly, CachedRoute& outRoute) {
    std::lock_guard<std::mutex> lock(cacheMutex);

    const auto it = pathCache.find({startPoly, goalPoly});
    if (it == pathCache.end()) {
        return false;
    }

    outRoute = it->second;
    it->second.hits += 1;
    return true;
}

void NavigationSystem::StoreCachedRoute(int startPoly, int goalPoly,
                                        const std::vector<int>& polyPath,
                                        const std::vector<Vec2>& compressedPath) {
    if (polyPath.empty() || compressedPath.empty()) {
        return;
    }

    std::lock_guard<std::mutex> lock(cacheMutex);

    pathCache[{startPoly, goalPoly}] = { polyPath, compressedPath, 0u };

    if (startPoly != goalPoly) {
        std::vector<int> reversedPoly = polyPath;
        std::reverse(reversedPoly.begin(), reversedPoly.end());

        std::vector<Vec2> reversedPoints = compressedPath;
        std::reverse(reversedPoints.begin(), reversedPoints.end());

        pathCache[{goalPoly, startPoly}] = { std::move(reversedPoly), std::move(reversedPoints), 0u };
    }

    cacheDirty = true;
}

std::vector<int> NavigationSystem::BuildPolyPath(int startPoly, int goalPoly) {
    return NavMesh::Instance().FindPath(startPoly, goalPoly);
}

std::vector<Vec2> NavigationSystem::BuildCompressedRoute(const std::vector<int>& polyPath,
                                                         const Vec2& startPoint,
                                                         const Vec2& goalPoint) {
    return NavMesh::Instance().FunnelPath(polyPath, startPoint, goalPoint);
}

bool NavigationSystem::SchedulePathBuild(Entity& entity, int targetX, int targetY, bool forceRepath) {
    const int duplicateThreshold = 10;
    const float repathCooldownSeconds = 0.20f;

    const float nowSeconds = static_cast<float>(SDL_GetTicks()) / 1000.0f;

    if (!forceRepath && nowSeconds < entity.nextPathUpdateTime) {
        return false;
    }

    if (!forceRepath && entity.hasPendingPathUpdate) {
        if (std::abs(entity.lastQueuedDestX - targetX) <= duplicateThreshold &&
            std::abs(entity.lastQueuedDestY - targetY) <= duplicateThreshold) {
            return false;
        }
    }

    Vec2 startPoint = {
        static_cast<int>(entity.position.x),
        static_cast<int>(entity.position.y)
    };

    Vec2 goalPoint = {
        targetX,
        targetY
    };

    int startPoly = NavMesh::Instance().GetPolygonIndexAt(startPoint.x, startPoint.y);
    if (startPoly == -1) {
        std::cout << "NavigationSystem: no valid start polygon for entity " << entity.id << "\n";
        return false;
    }

    int targetPoly = NavMesh::Instance().GetPolygonIndexAt(goalPoint.x, goalPoint.y);
    if (targetPoly == -1) {
        std::cout << "NavigationSystem: no valid target polygon at ("
                  << goalPoint.x << ", " << goalPoint.y << ")\n";
        return false;
    }

    entity.lastQueuedDestX = targetX;
    entity.lastQueuedDestY = targetY;
    entity.nextPathUpdateTime = nowSeconds + repathCooldownSeconds;

    if (!forceRepath) {
        CachedRoute cachedRoute;
        if (TryGetCachedRoute(startPoly, targetPoly, cachedRoute)) {
            entity.path = cachedRoute.compressedPath;

            if (!entity.path.empty()) {
                entity.path.front() = startPoint;
                entity.path.back() = goalPoint;
            }

            entity.hasPendingPathUpdate = false;
            return !entity.path.empty();
        }
    }

    auto promisePtr = std::make_shared<std::promise<std::vector<Vec2>>>();
    entity.asyncPathFuture = promisePtr->get_future();
    entity.hasPendingPathUpdate = true;

    g_taskGroup.run([startPoly, targetPoly, startPoint, goalPoint, promisePtr]() {
        try {
            NavigationSystem& nav = NavigationSystem::Instance();

            std::vector<int> polyPath = nav.BuildPolyPath(startPoly, targetPoly);
            std::vector<Vec2> compressedPath;

            if (!polyPath.empty()) {
                compressedPath = nav.BuildCompressedRoute(polyPath, startPoint, goalPoint);
            }

            if (!polyPath.empty() && !compressedPath.empty()) {
                nav.StoreCachedRoute(startPoly, targetPoly, polyPath, compressedPath);
            }

            promisePtr->set_value(std::move(compressedPath));
        } catch (...) {
            promisePtr->set_exception(std::current_exception());
        }
    });

    return true;
}