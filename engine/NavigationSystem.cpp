#include "NavigationSystem.h"
#include "EntityManager.h"
#include "NavMesh.h"

#include <SDL3/SDL.h>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <unordered_map>

namespace {
constexpr int NAV_CACHE_FILE_VERSION = 9;
constexpr float NEARBY_CACHE_REUSE_DISTANCE = 30.0f;
constexpr float GROUP_ROUTE_REUSE_DISTANCE = 96.0f;
constexpr float STRICT_POINT_REUSE_DISTANCE = 48.0f;
constexpr int PATH_WORKER_COUNT = 2;

float DistanceSquared(const Vec2& a, const Vec2& b) {
    const float dx = static_cast<float>(a.x - b.x);
    const float dy = static_cast<float>(a.y - b.y);
    return dx * dx + dy * dy;
}

size_t ComputeRouteStartIndex(const Vec2& currentStart,
                              const std::vector<Vec2>& route,
                              int clearanceBucket) {
    if (route.size() >= 2 &&
        NavMesh::Instance().HasLineOfSight(currentStart, route[1], clearanceBucket)) {
        return 1;
    }
    return 0;
}

void ClearMoveTarget(Entity& entity) {
    entity.navHasMoveTarget = false;
    entity.navTargetX = -1;
    entity.navTargetY = -1;
    entity.navClearanceBucket = 20;
    entity.navAllowLooseReuse = false;
}

void ClearQueuedMoveState(Entity& entity) {
    entity.navQueuedMove = {};
}
}

bool NavigationSystem::TryBuildDirectMove(Entity& entity,
                                          const Vec2& startPoint,
                                          const Vec2& goalPoint,
                                          int clearanceBucket) {
    if (!NavMesh::Instance().IsPointWalkable(goalPoint, clearanceBucket)) {
        return false;
    }

    if (!NavMesh::Instance().HasLineOfSight(startPoint, goalPoint, clearanceBucket)) {
        return false;
    }

    ++entity.navRequestGeneration;
    entity.path.clear();
    entity.path.push_back(goalPoint);
    entity.pathIndex = 0;
    entity.hasPendingPathUpdate = false;
    return true;
}

NavigationSystem::NavigationSystem() = default;
NavigationSystem::~NavigationSystem() = default;

NavigationSystem& NavigationSystem::Instance() {
    static NavigationSystem instance;
    return instance;
}

bool NavigationSystem::InitializePathCache(const std::string& filePath) {
    workers.Start(PATH_WORKER_COUNT);
    loadedCompatibleCache = false;
    if (!filePath.empty()) cacheFilePath = filePath;
    if (cacheFilePath.empty()) return true;

    const uint32_t navVersion = NavMesh::GetPathCacheNavVersion();
    const uint64_t mapVersion = NavMesh::Instance().GetPathCacheMapVersion();

    if (std::filesystem::exists(cacheFilePath)) {
        if (cache.LoadFromFile(cacheFilePath, NAV_CACHE_FILE_VERSION, navVersion, mapVersion)) {
            loadedCompatibleCache = true;
            std::cout << "NavigationSystem: exact path cache loaded from "
                      << cacheFilePath
                      << " entries=" << cache.GetRouteCount()
                      << "\n";
            return true;
        }
        std::error_code ec;
        std::filesystem::remove(cacheFilePath, ec);
    }

    ClearPathCache();
    return true;
}

bool NavigationSystem::InitializeStartupPrecompute(const std::string& navJsonPath,
                                                   const std::string& stateCachePath,
                                                   const std::string& targetRegionLabel,
                                                   const std::vector<int>& bucketRadii) {
    stateCacheFilePath = stateCachePath;
    startupTargetRegionLabel = targetRegionLabel;

    return training.EnsureStartupCaches(navJsonPath,
                                        targetRegionLabel,
                                        bucketRadii,
                                        stateCachePath);
}

void NavigationSystem::SetCacheFilePath(const std::string& filePath) {
    cacheFilePath = filePath;
}

bool NavigationSystem::SavePathCacheToFile(const std::string& filePath) {
    const std::string outPath = !filePath.empty() ? filePath : cacheFilePath;
    if (outPath.empty()) return false;

    const bool ok = cache.SaveToFile(outPath,
                                     NAV_CACHE_FILE_VERSION,
                                     NavMesh::GetPathCacheNavVersion(),
                                     NavMesh::Instance().GetPathCacheMapVersion());
    if (ok) {
        cache.MarkClean();
    }
    return ok;
}

void NavigationSystem::ClearPathCache() {
    cache.Clear();
    groups.Clear();
}

std::size_t NavigationSystem::GetCachedRouteCount() const { return cache.GetRouteCount(); }
bool NavigationSystem::IsPathCacheDirty() const { return cache.IsDirty(); }
bool NavigationSystem::HasLoadedCompatiblePathCache() const { return loadedCompatibleCache; }


void NavigationSystem::AdvancePathMarkers(Entity& entity) {
    while (entity.pathIndex < entity.path.size()) {
        const Vec2& next = entity.path[entity.pathIndex];
        const float dx = static_cast<float>(next.x) - entity.position.x;
        const float dy = static_cast<float>(next.y) - entity.position.y;
        // Tolerance must exceed the soft-collision push radius so the path
        // index advances even when wall repulsion holds the entity slightly shy
        // of the waypoint (the common cause of corner sticking).
        const float tolerance = std::max(16.0f, static_cast<float>(entity.radius) * 1.2f);
        if ((dx * dx + dy * dy) > (tolerance * tolerance)) {
            // Forward-skip: if the entity has line-of-sight to the waypoint after
            // this one it can skip straight to it, giving smooth steering around
            // corners without hard-targeting each intermediate point.
            if (entity.pathIndex + 1 < entity.path.size()) {
                const Vec2 current{
                    static_cast<int>(std::lround(entity.position.x)),
                    static_cast<int>(std::lround(entity.position.y))
                };
                if (NavMesh::Instance().HasLineOfSight(
                        current,
                        entity.path[entity.pathIndex + 1],
                        entity.navClearanceBucket)) {
                    ++entity.pathIndex;
                    continue;
                }
            }
            break;
        }
        ++entity.pathIndex;
    }

    if (entity.navHasMoveTarget && entity.pathIndex >= entity.path.size() && !entity.hasPendingPathUpdate) {
        const Vec2 currentPoint{
            static_cast<int>(std::lround(entity.position.x)),
            static_cast<int>(std::lround(entity.position.y))
        };
        const Vec2 goalPoint{entity.navTargetX, entity.navTargetY};
        const float tolerance = std::max(16.0f, static_cast<float>(entity.radius) * 1.2f);
        if (DistanceSquared(currentPoint, goalPoint) <= tolerance * tolerance) {
            ClearMoveTarget(entity);
        }
    }
}

void NavigationSystem::ProcessWorkerResults(EntityManager& entityManager) {
    NavigationWorkResult result;
    while (workers.TryPopResult(result)) {
        Entity* entity = entityManager.GetEntityById(result.entityId);
        if (!entity) continue;

        entity->hasPendingPathUpdate = false;

        if (result.generation != entity->navRequestGeneration) {
            continue;
        }

        if (!result.success) {
            entity->path.clear();
            entity->pathIndex = 0;
            continue;
        }

        entity->path = result.route;
        {
            const Vec2 currentPos{
                static_cast<int>(std::lround(entity->position.x)),
                static_cast<int>(std::lround(entity->position.y))
            };
            entity->pathIndex = ComputeRouteStartIndex(currentPos, entity->path, result.clearanceBucket);
        }

        const Vec2 startPoint{result.startX, result.startY};
        const Vec2 goalPoint{result.goalX, result.goalY};
        const uint64_t blockerRevision = NavMesh::Instance().GetBlockerRevision();

        cache.StoreExact(result.startPoly,
                         result.goalPoly,
                         result.clearanceBucket,
                         blockerRevision,
                         result.route,
                         startPoint,
                         goalPoint);

        if (!result.route.empty()) {
            groups.RegisterSharedRoute(result.route.front(),
                                       goalPoint,
                                       result.route,
                                       result.goalPoly,
                                       result.clearanceBucket,
                                       blockerRevision);
        }
    }
}

void NavigationSystem::Update(EntityManager& entityManager, float /*deltaTime*/) {
    frameWorkerBudget = PATH_WORKER_COUNT * 2;
    ProcessWorkerResults(entityManager);

    for (auto& entity : entityManager.entities) {
        AdvancePathMarkers(entity);

        if (!entity.hasPendingPathUpdate && entity.pathIndex >= entity.path.size() && entity.navQueuedMove.active) {
            const QueuedMoveRequest queued = entity.navQueuedMove;
            ClearQueuedMoveState(entity);

            SchedulePathBuild(entity,
                              queued.targetX,
                              queued.targetY,
                              queued.clearanceBucket,
                              queued.type == NavMoveRequestType::Loose,
                              true);
        }
    }

}

bool NavigationSystem::RequestMove(EntityManager& entityManager,
                                   int entityId,
                                   float targetX,
                                   float targetY,
                                   int navRadius,
                                   bool forceRepath) {
    return RequestMoveStrict(entityManager,
                             entityId,
                             targetX,
                             targetY,
                             navRadius,
                             forceRepath);
}

bool NavigationSystem::RequestMoveStrict(EntityManager& entityManager,
                                         int entityId,
                                         float targetX,
                                         float targetY,
                                         int navRadius,
                                         bool forceRepath) {
    return RequestMoveInternal(entityManager,
                               entityId,
                               targetX,
                               targetY,
                               navRadius,
                               false,
                               forceRepath);
}

bool NavigationSystem::RequestMoveLoose(EntityManager& entityManager,
                                        int entityId,
                                        float targetX,
                                        float targetY,
                                        int navRadius,
                                        bool forceRepath) {
    return RequestMoveInternal(entityManager,
                               entityId,
                               targetX,
                               targetY,
                               navRadius,
                               true,
                               forceRepath);
}

bool NavigationSystem::RequestMoveInternal(EntityManager& entityManager,
                                           int entityId,
                                           float targetX,
                                           float targetY,
                                           int navRadius,
                                           bool allowLooseReuse,
                                           bool forceRepath) {
    Entity* entity = entityManager.GetEntityById(entityId);
    if (!entity) return false;

    const int clearanceBucket = NavMesh::QuantizeClearanceBucket(navRadius > 0 ? navRadius : entity->radius);

    Vec2 clampedGoal{
        static_cast<int>(std::lround(targetX)),
        static_cast<int>(std::lround(targetY))
    };
    clampedGoal = NavMesh::Instance().ClampToNavMesh(clampedGoal, clearanceBucket);

    entity->navHasMoveTarget = true;
    entity->navTargetX = clampedGoal.x;
    entity->navTargetY = clampedGoal.y;
    entity->navClearanceBucket = clearanceBucket;
    entity->navAllowLooseReuse = allowLooseReuse;
    ClearQueuedMoveState(*entity);

    return SchedulePathBuild(*entity,
                             clampedGoal.x,
                             clampedGoal.y,
                             clearanceBucket,
                             allowLooseReuse,
                             forceRepath);
}

bool NavigationSystem::RequestEscapeMove(EntityManager& entityManager,
                                         int entityId,
                                         float targetX,
                                         float targetY,
                                         int navRadius) {
    Entity* entity = entityManager.GetEntityById(entityId);
    if (!entity) return false;

    const int clearanceBucket = NavMesh::QuantizeClearanceBucket(navRadius > 0 ? navRadius : entity->radius);

    Vec2 goalPoint{
        static_cast<int>(std::lround(targetX)),
        static_cast<int>(std::lround(targetY))
    };
    goalPoint = NavMesh::Instance().ClampToNavMesh(goalPoint, clearanceBucket);

    const Vec2 startPoint{
        static_cast<int>(std::lround(entity->position.x)),
        static_cast<int>(std::lround(entity->position.y))
    };

    const int startPoly = NavMesh::Instance().GetPolygonIndexAt(startPoint.x, startPoint.y, clearanceBucket);
    const int goalPoly = NavMesh::Instance().GetPolygonIndexAt(goalPoint.x, goalPoint.y, clearanceBucket);
    if (startPoly < 0 || goalPoly < 0) return false;

    std::vector<int> polyPath = NavMesh::Instance().FindPath(startPoly, goalPoly, clearanceBucket);
    if (polyPath.empty()) return false;

    std::vector<Vec2> route = NavMesh::Instance().FunnelPath(polyPath, startPoint, goalPoint, clearanceBucket);
    if (route.empty()) return false;

    ++entity->navRequestGeneration;
    ClearMoveTarget(*entity);
    ClearQueuedMoveState(*entity);
    entity->hasPendingPathUpdate = false;
    entity->path = std::move(route);
    entity->pathIndex = 0;
    return true;
}


bool NavigationSystem::QueueMove(EntityManager& entityManager,
                                 int entityId,
                                 float targetX,
                                 float targetY,
                                 int navRadius,
                                 bool allowLooseReuse) {
    Entity* entity = entityManager.GetEntityById(entityId);
    if (!entity) {
        return false;
    }

    const int clearanceBucket = NavMesh::QuantizeClearanceBucket(navRadius > 0 ? navRadius : entity->radius);

    Vec2 clampedGoal{
        static_cast<int>(std::lround(targetX)),
        static_cast<int>(std::lround(targetY))
    };
    clampedGoal = NavMesh::Instance().ClampToNavMesh(clampedGoal, clearanceBucket);

    entity->navQueuedMove.active = true;
    entity->navQueuedMove.targetX = clampedGoal.x;
    entity->navQueuedMove.targetY = clampedGoal.y;
    entity->navQueuedMove.clearanceBucket = clearanceBucket;
    entity->navQueuedMove.type = allowLooseReuse ? NavMoveRequestType::Loose : NavMoveRequestType::Strict;
    return true;
}

void NavigationSystem::ClearQueuedMove(EntityManager& entityManager, int entityId) {
    Entity* entity = entityManager.GetEntityById(entityId);
    if (!entity) {
        return;
    }
    ClearQueuedMoveState(*entity);
}

void NavigationSystem::StopNavigation(EntityManager& entityManager, int entityId, bool clearPath) {
    Entity* entity = entityManager.GetEntityById(entityId);
    if (!entity) return;

    ++entity->navRequestGeneration;
    ClearMoveTarget(*entity);
    ClearQueuedMoveState(*entity);
    entity->hasPendingPathUpdate = false;

    if (clearPath) {
        entity->path.clear();
        entity->pathIndex = 0;
    }
}

bool NavigationSystem::TryReuseStrictExactCache(Entity& entity,
                                                const Vec2& startPoint,
                                                const Vec2& goalPoint,
                                                int startPoly,
                                                int goalPoly,
                                                int clearanceBucket,
                                                uint64_t blockerRevision) {
    std::vector<Vec2> cachedRoute;
    Vec2 cachedStartPoint{};
    Vec2 cachedGoalPoint{};

    if (!cache.TryGetExact(startPoly,
                           goalPoly,
                           clearanceBucket,
                           blockerRevision,
                           cachedRoute,
                           &cachedStartPoint,
                           &cachedGoalPoint)) {
        return false;
    }

    const float tol = std::max(STRICT_POINT_REUSE_DISTANCE,
                               static_cast<float>(clearanceBucket) * 2.0f);

    if (DistanceSquared(startPoint, cachedStartPoint) > tol * tol ||
        DistanceSquared(goalPoint, cachedGoalPoint) > tol * tol) {
        return false;
    }

    ++entity.navRequestGeneration;
    entity.path = cachedRoute;
    entity.pathIndex = ComputeRouteStartIndex(startPoint, entity.path, clearanceBucket);

    return !entity.path.empty();
}

uint64_t NavigationSystem::GetCurrentStateKey() const {
    const uint64_t rev = NavMesh::Instance().GetBlockerRevision();
    if (rev != cachedBlockerRevision) {
        std::unordered_map<std::string, bool> liveBlockers;
        for (const auto& blocker : training.GetRegionGraph().GetBlockers()) {
            liveBlockers[blocker.toggleId] = NavMesh::Instance().IsBlockerEnabled(blocker.toggleId);
        }
        cachedStateKey = training.GetRegionGraph().ComputeStateKey(liveBlockers);
        cachedBlockerRevision = rev;
    }
    return cachedStateKey;
}

bool NavigationSystem::TryAttachToStateSubCache(Entity& entity,
                                                const Vec2& startPoint,
                                                int startPoly,
                                                int clearanceBucket) {
    const std::string regionLabel = training.GetRegionGraph().GetPrimaryRegionForPoly(startPoly);
    if (regionLabel.empty()) return false;

    const uint64_t stateKey = GetCurrentStateKey();

    const NavigationRegionStateSubCache* subCache =
        training.GetStateCache().FindRegionStateCache(regionLabel, stateKey, clearanceBucket);

    if (!subCache) {
        return false;
    }

    auto macroIt = subCache->polyToMacroAttach.find(startPoly);
    if (macroIt != subCache->polyToMacroAttach.end() && !macroIt->second.finalRoute.empty()) {
        ++entity.navRequestGeneration;
        entity.path = macroIt->second.finalRoute;
        entity.pathIndex = ComputeRouteStartIndex(startPoint, entity.path, clearanceBucket);

        return !entity.path.empty();
    }

    auto exitIt = subCache->polyToExitAttach.find(startPoly);
    if (exitIt != subCache->polyToExitAttach.end() && !exitIt->second.finalRoute.empty()) {
        ++entity.navRequestGeneration;
        entity.path = exitIt->second.finalRoute;
        entity.pathIndex = ComputeRouteStartIndex(startPoint, entity.path, clearanceBucket);

        return !entity.path.empty();
    }

    return false;
}

bool NavigationSystem::TryAttachToMacroRoute(Entity& entity,
                                             const Vec2& startPoint,
                                             int startPoly,
                                             int clearanceBucket) {
    const std::string startRegion = training.GetRegionGraph().GetPrimaryRegionForPoly(startPoly);
    if (startRegion.empty()) return false;

    const uint64_t stateKey = GetCurrentStateKey();

    const NavigationMacroRoute* best =
        training.GetMacroRoutes().FindBestRoute(startRegion,
                                                startupTargetRegionLabel,
                                                stateKey,
                                                clearanceBucket,
                                                startPoint);

    if (!best || best->finalRoute.empty()) {
        return false;
    }

    ++entity.navRequestGeneration;
    entity.path = best->finalRoute;
    entity.pathIndex = ComputeRouteStartIndex(startPoint, entity.path, clearanceBucket);

    return !entity.path.empty();
}

bool NavigationSystem::TryAttachToSharedGroupRoute(Entity& entity,
                                                   const Vec2& startPoint,
                                                   int goalPoly,
                                                   int clearanceBucket) {
    std::vector<Vec2> cachedRoute;
    Vec2 cachedStartAnchor{};
    Vec2 cachedGoalPoint{};

    if (!groups.TryGetSharedRoute(startPoint,
                                  goalPoly,
                                  clearanceBucket,
                                  NavMesh::Instance().GetBlockerRevision(),
                                  cachedRoute,
                                  cachedStartAnchor,
                                  cachedGoalPoint,
                                  GROUP_ROUTE_REUSE_DISTANCE)) {
        return false;
    }

    ++entity.navRequestGeneration;
    entity.path = cachedRoute;
    entity.pathIndex = ComputeRouteStartIndex(startPoint, entity.path, clearanceBucket);

    return !entity.path.empty();
}

bool NavigationSystem::TryAttachToNearbyCachedRoute(Entity& entity,
                                                    const Vec2& startPoint,
                                                    int goalPoly,
                                                    int clearanceBucket) {
    std::vector<Vec2> cachedRoute;
    Vec2 cachedStartPoint{};
    Vec2 cachedGoalPoint{};

    if (!cache.TryGetNearbyIndexed(startPoint,
                                   goalPoly,
                                   clearanceBucket,
                                   NavMesh::Instance().GetBlockerRevision(),
                                   cachedRoute,
                                   &cachedStartPoint,
                                   &cachedGoalPoint,
                                   NEARBY_CACHE_REUSE_DISTANCE)) {
        return false;
    }

    entity.path = cachedRoute;
    entity.pathIndex = ComputeRouteStartIndex(startPoint, entity.path, clearanceBucket);

    return !entity.path.empty();
}

bool NavigationSystem::GoalMatchesStartupRegion(int goalPoly) const {
    if (goalPoly < 0 || startupTargetRegionLabel.empty()) {
        return false;
    }
    const std::string goalRegion = training.GetRegionGraph().GetPrimaryRegionForPoly(goalPoly);
    return !goalRegion.empty() && goalRegion == startupTargetRegionLabel;
}

bool NavigationSystem::SchedulePathBuild(Entity& entity,
                                         int targetX,
                                         int targetY,
                                         int clearanceBucket,
                                         bool allowLooseReuse,
                                         bool forceRepath) {
    const Vec2 startPoint{
        static_cast<int>(std::lround(entity.position.x)),
        static_cast<int>(std::lround(entity.position.y))
    };
    const Vec2 goalPoint{targetX, targetY};

    const int startPoly = NavMesh::Instance().GetPolygonIndexAt(startPoint.x, startPoint.y, clearanceBucket);
    const int goalPoly = NavMesh::Instance().GetPolygonIndexAt(goalPoint.x, goalPoint.y, clearanceBucket);
    const uint64_t blockerRevision = NavMesh::Instance().GetBlockerRevision();

    if (startPoly < 0 || goalPoly < 0) {
        entity.path.clear();
        entity.pathIndex = 0;
        entity.hasPendingPathUpdate = false;
        return false;
    }

    if (TryBuildDirectMove(entity, startPoint, goalPoint, clearanceBucket)) {
        return true;
    }

    if (!forceRepath) {
        if (TryReuseStrictExactCache(entity,
                                     startPoint,
                                     goalPoint,
                                     startPoly,
                                     goalPoly,
                                     clearanceBucket,
                                     blockerRevision)) {
            entity.hasPendingPathUpdate = false;
            return !entity.path.empty();
        }

        if (allowLooseReuse && GoalMatchesStartupRegion(goalPoly)) {
            if (TryAttachToStateSubCache(entity, startPoint, startPoly, clearanceBucket)) {
                entity.hasPendingPathUpdate = false;
                return !entity.path.empty();
            }

            if (TryAttachToMacroRoute(entity, startPoint, startPoly, clearanceBucket)) {
                entity.hasPendingPathUpdate = false;
                return !entity.path.empty();
            }
        }

        if (allowLooseReuse) {
            if (TryAttachToSharedGroupRoute(entity, startPoint, goalPoly, clearanceBucket)) {
                entity.hasPendingPathUpdate = false;
                return true;
            }

            if (TryAttachToNearbyCachedRoute(entity, startPoint, goalPoly, clearanceBucket)) {
                entity.hasPendingPathUpdate = false;
                return true;
            }
        }
    }

    if (entity.hasPendingPathUpdate || workers.HasPendingForEntity(entity.id)) {
        return false;
    }

    if (frameWorkerBudget <= 0) {
        return false;
    }
    --frameWorkerBudget;

    entity.hasPendingPathUpdate = true;
    ++entity.navRequestGeneration;

    NavigationWorkRequest request;
    request.entityId = entity.id;
    request.startPoly = startPoly;
    request.goalPoly = goalPoly;
    request.clearanceBucket = clearanceBucket;
    request.startX = startPoint.x;
    request.startY = startPoint.y;
    request.goalX = goalPoint.x;
    request.goalY = goalPoint.y;
    request.generation = entity.navRequestGeneration;

    if (!workers.Enqueue(request)) {
        entity.hasPendingPathUpdate = false;
        ++frameWorkerBudget;
        return false;
    }

    return true;
}

bool NavigationSystem::PrewarmCacheFromNavJson(const std::string& navJsonPath,
                                               const std::string& roomLabel,
                                               const std::vector<int>& bucketRadii) {
    return InitializeStartupPrecompute(navJsonPath,
                                       stateCacheFilePath.empty() ? "assets/navstatecache.bin" : stateCacheFilePath,
                                       roomLabel,
                                       bucketRadii);
}

bool NavigationSystem::ArePointsInSamePrimaryRegion(float ax,
                                                    float ay,
                                                    float bx,
                                                    float by,
                                                    int navRadius) const {
    const int clearanceBucket = NavMesh::QuantizeClearanceBucket(navRadius > 0 ? navRadius : 20);

    const int aPoly = NavMesh::Instance().GetPolygonIndexAt(static_cast<int>(std::lround(ax)),
                                                            static_cast<int>(std::lround(ay)),
                                                            clearanceBucket);
    const int bPoly = NavMesh::Instance().GetPolygonIndexAt(static_cast<int>(std::lround(bx)),
                                                            static_cast<int>(std::lround(by)),
                                                            clearanceBucket);
    if (aPoly < 0 || bPoly < 0) {
        return false;
    }

    const std::string aRegion = training.GetRegionGraph().GetPrimaryRegionForPoly(aPoly);
    const std::string bRegion = training.GetRegionGraph().GetPrimaryRegionForPoly(bPoly);
    return !aRegion.empty() && aRegion == bRegion;
}

bool NavigationSystem::GetRegionCenter(const std::string& regionLabel, Vec2& outCenter) const {
    const NavigationRegionNode* region = training.GetRegionGraph().FindRegion(regionLabel);
    if (!region) {
        return false;
    }

    outCenter = region->center;
    return true;
}
