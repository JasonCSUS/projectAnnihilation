#include "UnitCollisionSystem.h"

#include "../engine/EntityManager.h"
#include "../engine/NavMesh.h"
#include "../engine/NavigationSystem.h"
#include "../engine/Rigidbodies.h"

#include "EntityData.h"
#include "GameEntityManager.h"
#include "Units.h"

#include <SDL3/SDL.h>
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <vector>

namespace {
constexpr float PROGRESS_MOVE_EPSILON = 0.75f;
constexpr float STANDBY_TIME_SECONDS = 0.50f;
constexpr float MAX_IDLE_NUDGE = 2.0f;
constexpr float ESCAPE_TRIGGER_FRUSTRATION = 3.0f;
constexpr float ESCAPE_COOLDOWN_SECONDS = 0.20f;
constexpr float RESOLVE_STUCK_TIME = 0.12f;
constexpr float TRAPPED_ABORT_TIME = 1.2f;
constexpr float ESCAPE_DISTANCE_90 = 40.0f;
constexpr float ESCAPE_DISTANCE_45 = 32.0f;
constexpr float ESCAPE_FORWARD_BIAS = 18.0f;
constexpr float COLLISION_FRUSTRATION_DECAY = 1.2f;
constexpr float PLAYER_TRAPPED_ABORT_TIME = 2.0f;

struct ProgressState {
    float lastX = 0.0f;
    float lastY = 0.0f;
    float stallTime = 0.0f;
    float collisionFrustration = 0.0f;
    float nextEscapeAllowedAt = 0.0f;
    bool initialized = false;

    bool hasPausedMainPath = false;
    bool isInEscapePath = false;
    bool resolvingConflict = false;
    float resolvingSince = 0.0f;
    float nextResolveCheckAt = 0.0f;
    std::vector<Vec2> pausedMainPath;
    size_t pausedMainPathIndex = 0;
};

std::unordered_map<int, ProgressState> g_progressStates;

float Distance2D(float ax, float ay, float bx, float by) {
    const float dx = ax - bx;
    const float dy = ay - by;
    return std::sqrt(dx * dx + dy * dy);
}

float Length(float x, float y) {
    return std::sqrt(x * x + y * y);
}

void Normalize(float& x, float& y) {
    const float len = Length(x, y);
    if (len > 0.0001f) {
        x /= len;
        y /= len;
    }
}

bool HasNavigationGoal(const Entity& e) {
    return e.navHasMoveTarget || e.pathIndex < e.path.size();
}

bool IsMovingUnit(const Entity& e, const EntityInfo* info) {
    return info && !info->isStatic && info->state == UNIT_STATE_MOVING && HasNavigationGoal(e);
}

void SyncBodyPosition(const Entity& entity) {
    const auto* body = RigidbodySystem::GetBody(entity.id);
    if (!body) {
        return;
    }
    RigidbodySystem::RegisterOrUpdateBody(entity.id,
                                          body->desc,
                                          entity.position.x,
                                          entity.position.y,
                                          entity.pathIndex < entity.path.size(),
                                          entity.pathIndex >= entity.path.size());
}

void TryIdleNudge(Entity& idleEntity, const Entity& mover) {
    float dx = idleEntity.position.x - mover.position.x;
    float dy = idleEntity.position.y - mover.position.y;
    float lenSq = dx * dx + dy * dy;
    if (lenSq < 0.0001f) {
        dx = 1.0f;
        dy = 0.0f;
        lenSq = 1.0f;
    }

    const float len = std::sqrt(lenSq);
    const float nx = dx / len;
    const float ny = dy / len;
    const float push = MAX_IDLE_NUDGE;
    const Vec2 nudgeTarget = NavMesh::Instance().ClampToNavMesh(
        {static_cast<int>(std::lround(idleEntity.position.x + nx * push)),
         static_cast<int>(std::lround(idleEntity.position.y + ny * push))},
        idleEntity.radius);
    idleEntity.position.x = static_cast<float>(nudgeTarget.x);
    idleEntity.position.y = static_cast<float>(nudgeTarget.y);
    SyncBodyPosition(idleEntity);
}

Vec2 MakeCandidate(const Entity& entity, float dirX, float dirY, float distance) {
    return {
        static_cast<int>(std::lround(entity.position.x + dirX * distance)),
        static_cast<int>(std::lround(entity.position.y + dirY * distance))
    };
}

void TryResumePausedMainPath(Entity& entity, ProgressState& progress) {
    if (!progress.isInEscapePath || entity.pathIndex < entity.path.size()) {
        return;
    }

    progress.isInEscapePath = false;
    progress.resolvingConflict = false;
    if (progress.hasPausedMainPath) {
        entity.path = std::move(progress.pausedMainPath);
        entity.pathIndex = progress.pausedMainPathIndex;
        progress.pausedMainPath.clear();
        progress.pausedMainPathIndex = 0;
        progress.hasPausedMainPath = false;
    }
}

void AbortTrappedMovement(EntityManager& entityManager, Entity& entity, EntityInfo& info, ProgressState& progress, float nowSeconds) {
    NavigationSystem::Instance().StopNavigation(entityManager, entity.id, true);
    progress.hasPausedMainPath = false;
    progress.pausedMainPath.clear();
    progress.isInEscapePath = false;
    progress.resolvingConflict = false;
    progress.resolvingSince = 0.0f;
    progress.nextResolveCheckAt = nowSeconds + ESCAPE_COOLDOWN_SECONDS;
    progress.collisionFrustration = 0.0f;
    progress.stallTime = 0.0f;
    if (info.controller == PLAYER) {
        info.state = UNIT_STATE_IDLE;
    }
}

bool TryBuildTrafficEscapePath(EntityManager& entityManager,
                               Entity& entity,
                               ProgressState& progress,
                               const RigidbodySystem::CollisionReport& report,
                               float nowSeconds) {
    if (entity.pathIndex >= entity.path.size()) {
        return false;
    }
    if (progress.nextEscapeAllowedAt > nowSeconds) {
        return false;
    }
    if (progress.collisionFrustration < ESCAPE_TRIGGER_FRUSTRATION) {
        return false;
    }

    const Vec2& nextMarker = entity.path[entity.pathIndex];
    float fwdX = static_cast<float>(nextMarker.x) - entity.position.x;
    float fwdY = static_cast<float>(nextMarker.y) - entity.position.y;
    Normalize(fwdX, fwdY);
    if (Length(fwdX, fwdY) <= 0.0001f) {
        return false;
    }

    float repelX = 0.0f;
    float repelY = 0.0f;
    int entityHitCount = 0;
    for (const auto& hit : report.hits) {
        if (hit.kind == RigidbodySystem::CollisionKind::Entity && hit.otherEntityId >= 0) {
            repelX += hit.normalX;
            repelY += hit.normalY;
            ++entityHitCount;
        }
    }

    if (entityHitCount > 0) {
        Normalize(repelX, repelY);
    } else {
        repelX = 0.0f;
        repelY = 0.0f;
    }

    const float leftX = -fwdY;
    const float leftY = fwdX;
    const float rightX = fwdY;
    const float rightY = -fwdX;

    float preferLeftScore = leftX * repelX + leftY * repelY;
    float preferRightScore = rightX * repelX + rightY * repelY;
    const bool preferLeft = preferLeftScore >= preferRightScore;

    struct CandidateDir {
        float x;
        float y;
        float distance;
    };

    std::vector<CandidateDir> dirs;
    dirs.reserve(6);

    auto pushDir = [&dirs](float x, float y, float distance) {
        Normalize(x, y);
        if (Length(x, y) > 0.0001f) {
            dirs.push_back({x, y, distance});
        }
    };

    if (preferLeft) {
        pushDir(fwdX + leftX, fwdY + leftY, ESCAPE_DISTANCE_45);
        pushDir(leftX, leftY, ESCAPE_DISTANCE_90);
        pushDir(fwdX + leftX * 0.5f, fwdY + leftY * 0.5f, ESCAPE_FORWARD_BIAS);
        pushDir(fwdX + rightX, fwdY + rightY, ESCAPE_DISTANCE_45);
        pushDir(rightX, rightY, ESCAPE_DISTANCE_90);
    } else {
        pushDir(fwdX + rightX, fwdY + rightY, ESCAPE_DISTANCE_45);
        pushDir(rightX, rightY, ESCAPE_DISTANCE_90);
        pushDir(fwdX + rightX * 0.5f, fwdY + rightY * 0.5f, ESCAPE_FORWARD_BIAS);
        pushDir(fwdX + leftX, fwdY + leftY, ESCAPE_DISTANCE_45);
        pushDir(leftX, leftY, ESCAPE_DISTANCE_90);
    }
    pushDir(-repelX, -repelY, ESCAPE_FORWARD_BIAS);

    const Vec2 currentPos{
        static_cast<int>(std::lround(entity.position.x)),
        static_cast<int>(std::lround(entity.position.y))
    };

    for (const CandidateDir& dir : dirs) {
        Vec2 candidate = MakeCandidate(entity, dir.x, dir.y, dir.distance);
        candidate = NavMesh::Instance().ClampToNavMesh(candidate, entity.radius);
        if (!NavMesh::Instance().IsPointWalkable(candidate, entity.radius)) {
            continue;
        }
        if (!NavMesh::Instance().HasLineOfSight(currentPos, candidate, entity.radius)) {
            continue;
        }
        if (RigidbodySystem::WouldCollideWithEntities(entity.id,
                                                      static_cast<float>(candidate.x),
                                                      static_cast<float>(candidate.y))) {
            continue;
        }

        if (!progress.hasPausedMainPath) {
            progress.pausedMainPathIndex = entity.pathIndex;
            progress.pausedMainPath = std::move(entity.path);
            progress.hasPausedMainPath = true;
        }
        entity.path.clear();
        entity.pathIndex = 0;

        const bool ok = NavigationSystem::Instance().RequestEscapeMove(
            entityManager,
            entity.id,
            static_cast<float>(candidate.x),
            static_cast<float>(candidate.y),
            entity.radius
        );

        progress.nextEscapeAllowedAt = nowSeconds + ESCAPE_COOLDOWN_SECONDS;
        if (ok) {
            progress.isInEscapePath = true;
            progress.resolvingConflict = true;
            progress.resolvingSince = nowSeconds;
            progress.nextResolveCheckAt = nowSeconds + ESCAPE_COOLDOWN_SECONDS;
            progress.collisionFrustration = 0.0f;
            return true;
        }
    }

    return false;
}

} // namespace

UnitCollisionSystem& UnitCollisionSystem::Instance() {
    static UnitCollisionSystem instance;
    return instance;
}

void UnitCollisionSystem::Update(EntityManager& entityManager,
                                 GameEntityManager& gameEntityManager,
                                 float deltaTime) {
    (void)gameEntityManager;
    const float now = static_cast<float>(SDL_GetTicks()) / 1000.0f;

    for (auto& entity : entityManager.entities) {
        if (entity.isDead) {
            continue;
        }
        ProgressState& progress = g_progressStates[entity.id];
        if (!progress.initialized) {
            progress.lastX = entity.position.x;
            progress.lastY = entity.position.y;
            progress.initialized = true;
        }
    }

    // Immediate game-side reactions to engine collision reports.
    for (auto& entity : entityManager.entities) {
        if (entity.isDead) {
            continue;
        }

        EntityInfo* info = EntityData::TryGet(entity.id);
        if (!info) {
            continue;
        }

        const bool entityMoving = IsMovingUnit(entity, info);
        const RigidbodySystem::CollisionReport* report = RigidbodySystem::GetLastReport(entity.id);
        if (!report || !entityMoving) {
            continue;
        }

        int entityHitCount = 0;
        for (const auto& hit : report->hits) {
            if (hit.kind != RigidbodySystem::CollisionKind::Entity || hit.otherEntityId < 0) {
                continue;
            }
            ++entityHitCount;
            info->lastCollisionPartnerId = hit.otherEntityId;

            Entity* otherEntity = entityManager.GetEntityById(hit.otherEntityId);
            if (!otherEntity || otherEntity->isDead) {
                continue;
            }

            EntityInfo* otherInfo = EntityData::TryGet(otherEntity->id);
            const auto* otherBody = RigidbodySystem::GetBody(otherEntity->id);
            const bool otherMoving = otherBody ? otherBody->moving : IsMovingUnit(*otherEntity, otherInfo);
            const bool canNudge = otherInfo && otherInfo->canBePushed && !otherInfo->heroic && !otherInfo->massive && !(otherBody && otherBody->desc.immovable);
            if (!otherMoving && canNudge) {
                TryIdleNudge(*otherEntity, entity);
            }
        }

        info->collisionFrustration = std::max(info->collisionFrustration,
                                              static_cast<float>(entityHitCount) * 0.5f);
    }

    // Traffic / frustration / yield / escape-path logic.
    for (auto& entity : entityManager.entities) {
        if (entity.isDead) {
            continue;
        }

        EntityInfo* info = EntityData::TryGet(entity.id);
        if (!info) {
            continue;
        }

        ProgressState& progress = g_progressStates[entity.id];
        const float moved = Distance2D(entity.position.x,
                                       entity.position.y,
                                       progress.lastX,
                                       progress.lastY);

        if (!HasNavigationGoal(entity) || info->isStatic) {
            progress.stallTime = 0.0f;
            progress.resolvingConflict = false;
            progress.resolvingSince = 0.0f;
            progress.collisionFrustration = std::max(
                0.0f,
                progress.collisionFrustration - COLLISION_FRUSTRATION_DECAY * deltaTime
            );
            info->collisionFrustration = progress.collisionFrustration;
            progress.lastX = entity.position.x;
            progress.lastY = entity.position.y;
            continue;
        }

        if (info->state == UNIT_STATE_STANDBY || info->state == UNIT_STATE_YIELDING) {
            if (now >= info->yieldUntil) {
                info->state = UNIT_STATE_MOVING;
            } else {
                progress.lastX = entity.position.x;
                progress.lastY = entity.position.y;
                continue;
            }
        }

        if (moved <= PROGRESS_MOVE_EPSILON) {
            progress.stallTime += deltaTime;
        } else {
            progress.stallTime = 0.0f;
        }

        TryResumePausedMainPath(entity, progress);

        const RigidbodySystem::CollisionReport* report = RigidbodySystem::GetLastReport(entity.id);
        int entityHits = 0;
        int movingBlockers = 0;
        bool blockedByEntity = false;
        if (report) {
            blockedByEntity = report->blockedByEntity;
            for (const auto& hit : report->hits) {
                if (hit.kind != RigidbodySystem::CollisionKind::Entity || hit.otherEntityId < 0) {
                    continue;
                }
                ++entityHits;
                const auto* otherBody = RigidbodySystem::GetBody(hit.otherEntityId);
                if (otherBody && otherBody->moving) {
                    ++movingBlockers;
                }
            }
        }

        if (entityHits > 0 || blockedByEntity) {
            progress.collisionFrustration += deltaTime * (0.75f + static_cast<float>(movingBlockers));
        } else {
            progress.collisionFrustration = std::max(
                0.0f,
                progress.collisionFrustration - COLLISION_FRUSTRATION_DECAY * deltaTime
            );
        }
        info->collisionFrustration = progress.collisionFrustration;

        const bool blockedThisFrame = report && blockedByEntity;
        if (blockedThisFrame && moved <= PROGRESS_MOVE_EPSILON) {
            if (!progress.resolvingConflict) {
                progress.resolvingConflict = true;
                progress.resolvingSince = now;
                progress.nextResolveCheckAt = now + ESCAPE_COOLDOWN_SECONDS;
            }
        } else if (moved > PROGRESS_MOVE_EPSILON || (!blockedThisFrame && entityHits == 0)) {
            progress.resolvingConflict = false;
            progress.resolvingSince = 0.0f;
        }

        if (progress.resolvingConflict && !progress.isInEscapePath) {
            const float trapAbortTime = (info->controller == PLAYER) ? PLAYER_TRAPPED_ABORT_TIME : TRAPPED_ABORT_TIME;
            if ((now - progress.resolvingSince) >= trapAbortTime && progress.collisionFrustration >= ESCAPE_TRIGGER_FRUSTRATION) {
                AbortTrappedMovement(entityManager, entity, *info, progress, now);
            } else if (blockedThisFrame && progress.stallTime >= RESOLVE_STUCK_TIME && now >= progress.nextResolveCheckAt && entity.pathIndex < entity.path.size()) {
                const bool escaped = TryBuildTrafficEscapePath(entityManager, entity, progress, *report, now);
                if (!escaped) {
                    progress.nextResolveCheckAt = now + ESCAPE_COOLDOWN_SECONDS;
                }
            }
        }

        if (!progress.isInEscapePath &&
            !progress.resolvingConflict &&
            progress.stallTime >= 2.0f &&
            entityHits == 0 &&
            info->canYield &&
            info->controller != ENEMY) {
            info->state = UNIT_STATE_STANDBY;
            info->yieldUntil = now + STANDBY_TIME_SECONDS;
            NavigationSystem::Instance().StopNavigation(entityManager, entity.id, false);
            progress.stallTime = 0.0f;
        }

        // Enemies have no yield mechanism, so reset their path when stuck on walls
        // long enough that the AI can pick a fresh route on the next update.
        if (!progress.isInEscapePath &&
            !progress.resolvingConflict &&
            progress.stallTime >= TRAPPED_ABORT_TIME &&
            entityHits == 0 &&
            info->controller == ENEMY &&
            HasNavigationGoal(entity)) {
            NavigationSystem::Instance().StopNavigation(entityManager, entity.id, true);
            progress.stallTime = 0.0f;
        }

        progress.lastX = entity.position.x;
        progress.lastY = entity.position.y;
    }

    std::unordered_set<int> liveIds;
    liveIds.reserve(entityManager.entities.size());
    for (const auto& entity : entityManager.entities) {
        if (!entity.isDead) liveIds.insert(entity.id);
    }

    for (auto it = g_progressStates.begin(); it != g_progressStates.end();) {
        if (liveIds.count(it->first) == 0) {
            it = g_progressStates.erase(it);
        } else {
            ++it;
        }
    }
}
