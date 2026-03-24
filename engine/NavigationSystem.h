#ifndef NAVIGATIONSYSTEM_H
#define NAVIGATIONSYSTEM_H

#include <string>
#include <vector>
#include <unordered_map>
#include <mutex>
#include <cstddef>

#include "NavMesh.h"

class EntityManager;
struct Entity;

class NavigationSystem {
public:
    static NavigationSystem& Instance();

    void Update(EntityManager& entityManager, float deltaTime);

    bool RequestMove(EntityManager& entityManager, int entityId, float targetX, float targetY, bool forceRepath = false);
    void StopNavigation(EntityManager& entityManager, int entityId, bool clearPath = true);

    // Cache lifecycle
    bool InitializePathCache(const std::string& filePath = "");
    void SetCacheFilePath(const std::string& filePath);
    bool LoadPathCacheFromFile(const std::string& filePath);
    bool SavePathCacheToFile(const std::string& filePath = "");
    void ClearPathCache();

    std::size_t GetCachedRouteCount() const;
    bool IsPathCacheDirty() const;

private:
    NavigationSystem() = default;
    NavigationSystem(const NavigationSystem&) = delete;
    NavigationSystem& operator=(const NavigationSystem&) = delete;

    struct PathCacheKey {
        int startPoly;
        int goalPoly;

        bool operator==(const PathCacheKey& other) const {
            return startPoly == other.startPoly && goalPoly == other.goalPoly;
        }
    };

    struct PathCacheKeyHash {
        std::size_t operator()(const PathCacheKey& key) const noexcept {
            const std::size_t a = static_cast<std::size_t>(static_cast<unsigned int>(key.startPoly));
            const std::size_t b = static_cast<std::size_t>(static_cast<unsigned int>(key.goalPoly));
            return (a << 32) ^ b;
        }
    };

    struct CachedRoute {
        std::vector<int> polyPath;          // reusable polygon corridor
        std::vector<Vec2> compressedPath;   // reusable route built from corridor
        unsigned int hits = 0;
    };

    Entity* FindEntityById(EntityManager& entityManager, int entityId);

    void ProcessPendingPathUpdates(Entity& entity);
    bool SchedulePathBuild(Entity& entity, int targetX, int targetY, bool forceRepath);

    bool TryGetCachedRoute(int startPoly, int goalPoly, CachedRoute& outRoute);
    void StoreCachedRoute(int startPoly, int goalPoly,
                          const std::vector<int>& polyPath,
                          const std::vector<Vec2>& compressedPath);

    std::vector<int> BuildPolyPath(int startPoly, int goalPoly);
    std::vector<Vec2> BuildCompressedRoute(const std::vector<int>& polyPath,
                                           const Vec2& startPoint,
                                           const Vec2& goalPoint);

private:
    mutable std::mutex cacheMutex;
    std::unordered_map<PathCacheKey, CachedRoute, PathCacheKeyHash> pathCache;
    std::string cacheFilePath;
    bool cacheDirty = false;
};

#endif