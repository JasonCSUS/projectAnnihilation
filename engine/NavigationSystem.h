#ifndef NAVIGATIONSYSTEM_H
#define NAVIGATIONSYSTEM_H

#include "NavigationCache.h"
#include "NavigationGroups.h"
#include "NavigationWorkers.h"
#include "NavigationTraining.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

class EntityManager;
struct Entity;
struct Vec2;

class NavigationSystem {
public:
    static NavigationSystem& Instance();

    void Update(EntityManager& entityManager, float deltaTime);

    bool RequestMove(EntityManager& entityManager,
                     int entityId,
                     float targetX,
                     float targetY,
                     int navRadius,
                     bool forceRepath = false);

    bool RequestMoveStrict(EntityManager& entityManager,
                           int entityId,
                           float targetX,
                           float targetY,
                           int navRadius,
                           bool forceRepath = false);

    bool RequestMoveLoose(EntityManager& entityManager,
                          int entityId,
                          float targetX,
                          float targetY,
                          int navRadius,
                          bool forceRepath = false);

    bool RequestEscapeMove(EntityManager& entityManager,
                           int entityId,
                           float targetX,
                           float targetY,
                           int navRadius);

    bool QueueMove(EntityManager& entityManager,
                   int entityId,
                   float targetX,
                   float targetY,
                   int navRadius,
                   bool allowLooseReuse);

    void ClearQueuedMove(EntityManager& entityManager, int entityId);

    void StopNavigation(EntityManager& entityManager, int entityId, bool clearPath = true);

    bool InitializePathCache(const std::string& filePath);
    bool InitializeStartupPrecompute(const std::string& navJsonPath,
                                     const std::string& stateCachePath,
                                     const std::string& targetRegionLabel,
                                     const std::vector<int>& bucketRadii);

    void SetCacheFilePath(const std::string& filePath);
    bool SavePathCacheToFile(const std::string& filePath = "");
    void ClearPathCache();

    std::size_t GetCachedRouteCount() const;
    bool IsPathCacheDirty() const;
    bool HasLoadedCompatiblePathCache() const;

    bool PrewarmCacheFromNavJson(const std::string& navJsonPath,
                                 const std::string& roomLabel,
                                 const std::vector<int>& bucketRadii);

    bool ArePointsInSamePrimaryRegion(float ax,
                                      float ay,
                                      float bx,
                                      float by,
                                      int navRadius) const;

    bool GetRegionCenter(const std::string& regionLabel, Vec2& outCenter) const;

    struct DebugStats {
        uint64_t requests = 0;
        uint64_t exactHits = 0;
        uint64_t stateHits = 0;
        uint64_t macroHits = 0;
        uint64_t groupHits = 0;
        uint64_t nearbyHits = 0;
        uint64_t workerQueued = 0;
        uint64_t workerRejectedAlreadyPending = 0;
        uint64_t invalidStartOrGoal = 0;
        uint64_t workerSuccess = 0;
        uint64_t workerFailure = 0;
    };

    void ResetDebugStats();
    void DumpDebugStats(const char* prefix = "NavigationSystem") const;
    const DebugStats& GetDebugStats() const { return debugStats; }

private:
    NavigationSystem();
    ~NavigationSystem();
    NavigationSystem(const NavigationSystem&) = delete;
    NavigationSystem& operator=(const NavigationSystem&) = delete;

    bool RequestMoveInternal(EntityManager& entityManager,
                             int entityId,
                             float targetX,
                             float targetY,
                             int navRadius,
                             bool allowLooseReuse,
                             bool forceRepath);

    void ProcessWorkerResults(EntityManager& entityManager);
    void AdvancePathMarkers(Entity& entity);

    bool SchedulePathBuild(Entity& entity,
                           int targetX,
                           int targetY,
                           int clearanceBucket,
                           bool allowLooseReuse,
                           bool forceRepath);

    bool TryBuildDirectMove(Entity& entity,
                            const Vec2& startPoint,
                            const Vec2& goalPoint,
                            int clearanceBucket);

    bool TryReuseStrictExactCache(Entity& entity,
                                  const Vec2& startPoint,
                                  const Vec2& goalPoint,
                                  int startPoly,
                                  int goalPoly,
                                  int clearanceBucket,
                                  uint64_t blockerRevision);

    bool TryAttachToStateSubCache(Entity& entity,
                                  const Vec2& startPoint,
                                  int startPoly,
                                  int clearanceBucket);

    bool TryAttachToMacroRoute(Entity& entity,
                               const Vec2& startPoint,
                               int startPoly,
                               int clearanceBucket);

    bool TryAttachToSharedGroupRoute(Entity& entity,
                                     const Vec2& startPoint,
                                     int goalPoly,
                                     int clearanceBucket);

    bool TryAttachToNearbyCachedRoute(Entity& entity,
                                      const Vec2& startPoint,
                                      int goalPoly,
                                      int clearanceBucket);

    bool GoalMatchesStartupRegion(int goalPoly) const;

    uint64_t GetCurrentStateKey() const;

private:
    NavigationCache cache;
    NavigationGroups groups;
    NavigationWorkers workers;
    NavigationTraining training;
    std::string cacheFilePath;
    std::string stateCacheFilePath;
    std::string startupTargetRegionLabel;
    bool loadedCompatibleCache = false;
    DebugStats debugStats;
    int frameWorkerBudget = 0;

    // Cached blocker state key — recomputed only when blockerRevision changes.
    mutable uint64_t cachedStateKey = 0;
    mutable uint64_t cachedBlockerRevision = static_cast<uint64_t>(-1);
};

#endif
