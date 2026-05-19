#include "MovementSystem.h"
#include "AnimationManager.h"
#include "EntityManager.h"
#include "NavMesh.h"
#include "Rigidbodies.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace {
constexpr float kMinMoveEpsilon = 0.001f;

enum class MoveDir { Right, Left, Down, Up };

MoveDir DirFromDelta(float dx, float dy) {
    if (std::abs(dx) >= std::abs(dy)) {
        return dx >= 0.0f ? MoveDir::Right : MoveDir::Left;
    }
    return dy >= 0.0f ? MoveDir::Down : MoveDir::Up;
}

MoveDir DirFromAngle(float angleDegrees) {
    const float rad = angleDegrees * static_cast<float>(3.14159265358979323846 / 180.0);
    return DirFromDelta(std::cos(rad), std::sin(rad));
}

void PlayMoveAnim(AnimationManager& animationManager, int entityId, MoveDir dir) {
    bool flipX = false;
    const char* animId = nullptr;
    switch (dir) {
        case MoveDir::Right: animId = "move_right"; flipX = false; break;
        case MoveDir::Left:  animId = "move_right"; flipX = true;  break;
        case MoveDir::Down:  animId = "move_down";  flipX = false; break;
        case MoveDir::Up:    animId = "move_up";    flipX = false; break;
    }
    animationManager.PlayAnimation(entityId, animId, 0.15f, true, false, flipX);
}

void PlayIdleAnim(AnimationManager& animationManager, int entityId, MoveDir dir) {
    bool flipX = false;
    const char* animId = nullptr;
    switch (dir) {
        case MoveDir::Right: animId = "idle_right"; flipX = false; break;
        case MoveDir::Left:  animId = "idle_right"; flipX = true;  break;
        case MoveDir::Down:  animId = "idle_down";  flipX = false; break;
        case MoveDir::Up:    animId = "idle_up";    flipX = false; break;
    }
    animationManager.PlayAnimation(entityId, animId, 0.25f, true, false, flipX);
}

// Project (vx, vy) out of all blocking normals in the report.
// Returns true and writes the renormalized result if any projection occurred.
bool ProjectOutNormals(const RigidbodySystem::CollisionReport& report,
                       float& vx, float& vy) {
    bool projected = false;
    for (const auto& hit : report.hits) {
        if (hit.kind == RigidbodySystem::CollisionKind::OutOfBounds) continue;
        const float dot = vx * hit.normalX + vy * hit.normalY;
        if (dot < 0.f) {
            vx -= hit.normalX * dot;
            vy -= hit.normalY * dot;
            projected = true;
        }
    }
    if (projected) {
        const float len = std::sqrt(vx * vx + vy * vy);
        if (len > kMinMoveEpsilon) { vx /= len; vy /= len; }
        else { vx = 0.f; vy = 0.f; }
    }
    return projected;
}
} // namespace

void MovementSystem::Update(EntityManager& entityManager, AnimationManager& animationManager, float deltaTime) {
    for (auto& entity : entityManager.entities) {
        if (entity.isDead) {
            RigidbodySystem::ClearEntityReport(entity.id);
            continue;
        }

        if (entity.pathIndex >= entity.path.size()) {
            entity.velX = 0.f;
            entity.velY = 0.f;
            RigidbodySystem::ClearEntityReport(entity.id);
            PlayIdleAnim(animationManager, entity.id, DirFromAngle(entity.movementAngleDegrees));
            continue;
        }

        const Vec2 nextPoint = entity.path[entity.pathIndex];
        const float targetX = static_cast<float>(nextPoint.x);
        const float targetY = static_cast<float>(nextPoint.y);

        const float dx = targetX - entity.position.x;
        const float dy = targetY - entity.position.y;
        const float dist = std::sqrt(dx * dx + dy * dy);

        if (dist < kMinMoveEpsilon) continue;

        // Steer direction vector directly toward next waypoint.
        entity.velX = dx / dist;
        entity.velY = dy / dist;

        entity.movementAngleDegrees = static_cast<float>(
            std::atan2(static_cast<double>(entity.velY), static_cast<double>(entity.velX)) * 180.0 / 3.14159265358979323846);
        PlayMoveAnim(animationManager, entity.id, DirFromDelta(entity.velX, entity.velY));

        // Pre-project velocity out of last frame's entity collision normals (3 passes).
        // By cancelling the inward component before movement, the entity slides along
        // blockers from the start instead of moving in and oscillating back out.
        {
            constexpr int kPreProjectPasses = 3;
            const auto* prevReport = RigidbodySystem::GetLastReport(entity.id);
            if (prevReport) {
                for (int pass = 0; pass < kPreProjectPasses; ++pass) {
                    bool changed = false;
                    for (const auto& hit : prevReport->hits) {
                        if (hit.kind != RigidbodySystem::CollisionKind::Entity) continue;
                        const float dot = entity.velX * hit.normalX + entity.velY * hit.normalY;
                        if (dot < 0.f) {
                            entity.velX -= hit.normalX * dot;
                            entity.velY -= hit.normalY * dot;
                            changed = true;
                        }
                    }
                    if (!changed) break;
                    const float len = std::sqrt(entity.velX * entity.velX + entity.velY * entity.velY);
                    if (len > kMinMoveEpsilon) { entity.velX /= len; entity.velY /= len; }
                    else { entity.velX = 0.f; entity.velY = 0.f; break; }
                }
            }
        }

        // Soft-collision: push entity out of walls, then deflect direction vector
        // away from the wall surface so later TestMove doesn't immediately re-block.
        {
            float pushX = 0.f, pushY = 0.f;
            if (NavMesh::Instance().ResolveSoftCollision(
                    {static_cast<int>(std::lround(entity.position.x)),
                     static_cast<int>(std::lround(entity.position.y))},
                    static_cast<float>(entity.radius),
                    pushX, pushY)) {
                const float pLenSq = pushX * pushX + pushY * pushY;
                if (pLenSq > 0.0001f) {
                    const float pLen = std::sqrt(pLenSq);
                    const float pnx = pushX / pLen, pny = pushY / pLen;
                    const float inward = entity.velX * (-pnx) + entity.velY * (-pny);
                    if (inward > 0.f) {
                        entity.velX += pnx * inward;
                        entity.velY += pny * inward;
                        const float vLen = std::sqrt(entity.velX * entity.velX + entity.velY * entity.velY);
                        if (vLen > kMinMoveEpsilon) { entity.velX /= vLen; entity.velY /= vLen; }
                    }
                }
                entity.position.x += pushX;
                entity.position.y += pushY;
            }
        }

        const float step = std::min(entity.speed * deltaTime, dist);

        // --- Stage 1: attempt move along current direction vector ---
        {
            RigidbodySystem::MoveQuery q;
            q.entityId = entity.id;
            q.fromX = entity.position.x;
            q.fromY = entity.position.y;
            q.toX = entity.position.x + entity.velX * step;
            q.toY = entity.position.y + entity.velY * step;
            auto result = RigidbodySystem::TestMoveForMovement(q);

            if (!result.blocked) {
                entity.position.x = result.acceptedX;
                entity.position.y = result.acceptedY;
                RigidbodySystem::StoreLastReport(entity.id, result.report);
                continue;
            }

            // --- Stage 2: project direction out of blocking normals, retry ---
            ProjectOutNormals(result.report, entity.velX, entity.velY);

            const float vLen = std::sqrt(entity.velX * entity.velX + entity.velY * entity.velY);
            if (vLen > kMinMoveEpsilon) {
                q.toX = entity.position.x + entity.velX * step;
                q.toY = entity.position.y + entity.velY * step;
                auto slideResult = RigidbodySystem::TestMoveForMovement(q);

                if (!slideResult.blocked) {
                    entity.position.x = slideResult.acceptedX;
                    entity.position.y = slideResult.acceptedY;
                    RigidbodySystem::StoreLastReport(entity.id, slideResult.report);
                    continue;
                }
            }

            // --- Stage 3: axis-separated fallback — never fully stop ---
            // Try X and Y independently so movement always occurs on at least one axis.
            const float svx = entity.velX * step;
            const float svy = entity.velY * step;

            bool movedX = false, movedY = false;

            if (std::abs(svx) > kMinMoveEpsilon) {
                RigidbodySystem::MoveQuery qx;
                qx.entityId = entity.id;
                qx.fromX = entity.position.x;
                qx.fromY = entity.position.y;
                qx.toX = entity.position.x + svx;
                qx.toY = entity.position.y;
                auto rx = RigidbodySystem::TestMoveForMovement(qx);
                if (!rx.blocked) {
                    entity.position.x = rx.acceptedX;
                    movedX = true;
                    RigidbodySystem::StoreLastReport(entity.id, rx.report);
                }
            }

            if (std::abs(svy) > kMinMoveEpsilon) {
                RigidbodySystem::MoveQuery qy;
                qy.entityId = entity.id;
                qy.fromX = entity.position.x;
                qy.fromY = entity.position.y;
                qy.toX = entity.position.x;
                qy.toY = entity.position.y + svy;
                auto ry = RigidbodySystem::TestMoveForMovement(qy);
                if (!ry.blocked) {
                    entity.position.y = ry.acceptedY;
                    movedY = true;
                    RigidbodySystem::StoreLastReport(entity.id, ry.report);
                }
            }

            // Keep direction vector consistent with what actually moved.
            if (!movedX) entity.velX = 0.f;
            if (!movedY) entity.velY = 0.f;
            const float vLen2 = std::sqrt(entity.velX * entity.velX + entity.velY * entity.velY);
            if (vLen2 > kMinMoveEpsilon) { entity.velX /= vLen2; entity.velY /= vLen2; }

            if (!movedX && !movedY) {
                RigidbodySystem::StoreLastReport(entity.id, result.report);
            }
        }
    }

    // Second pass: entity separation — accumulated per entity (prevents conflicting pushes
    // when multiple entities overlap the same target), multiple iterations to settle groups.
    {
        constexpr int kSepIterations = 3;
        const int count = static_cast<int>(entityManager.entities.size());

        // Cache body pointers once to avoid repeated hash-map lookups in the O(n²) inner loop.
        std::vector<const RigidbodySystem::BodyState*> bodies(count, nullptr);
        for (int i = 0; i < count; ++i) {
            const auto& e = entityManager.entities[i];
            if (!e.isDead)
                bodies[i] = RigidbodySystem::GetBody(e.id);
        }

        std::vector<float> sepX(count, 0.f), sepY(count, 0.f);

        for (int iter = 0; iter < kSepIterations; ++iter) {
            std::fill(sepX.begin(), sepX.end(), 0.f);
            std::fill(sepY.begin(), sepY.end(), 0.f);

            for (int i = 0; i < count; ++i) {
                const auto& a = entityManager.entities[i];
                if (a.isDead) continue;
                const auto* ba = bodies[i];
                if (!ba || !ba->desc.enabled || !ba->desc.entityCollision || ba->desc.layer < 0) continue;
                const unsigned int aLayerBit = (ba->desc.layer >= 32) ? 0u
                    : (1u << static_cast<unsigned int>(ba->desc.layer));

                for (int j = i + 1; j < count; ++j) {
                    const auto& b = entityManager.entities[j];
                    if (b.isDead) continue;
                    const auto* bb = bodies[j];
                    if (!bb || !bb->desc.enabled || !bb->desc.entityCollision || bb->desc.layer < 0) continue;
                    const unsigned int bLayerBit = (bb->desc.layer >= 32) ? 0u
                        : (1u << static_cast<unsigned int>(bb->desc.layer));

                    // Mirrors LayersCanCollide: both sides must agree to collide.
                    if ((ba->desc.collisionMask & bLayerBit) == 0u) continue;
                    if ((bb->desc.collisionMask & aLayerBit) == 0u) continue;

                    const float combinedR = static_cast<float>(a.radius + b.radius);
                    const float dx = a.position.x - b.position.x;
                    const float dy = a.position.y - b.position.y;
                    const float distSq = dx * dx + dy * dy;
                    if (distSq >= combinedR * combinedR) continue;

                    const float dist = std::sqrt(distSq);
                    float nx, ny;
                    if (dist > 0.001f) { nx = dx / dist; ny = dy / dist; }
                    else               { nx = 1.0f;      ny = 0.0f; }

                    const float overlap = combinedR - dist;
                    const bool aFixed = ba->desc.immovable;
                    const bool bFixed = bb->desc.immovable;

                    // Velocity-aware damping: reduce the push when both entities are
                    // already moving apart on their own (positive dot = separating).
                    // Prevents the path-following system and the separator from fighting
                    // each other, which is the main cause of multi-entity shaking.
                    const float relVdotN = (a.velX - b.velX) * nx + (a.velY - b.velY) * ny;
                    const float velScale = 1.0f - std::clamp(relVdotN * 0.5f, 0.0f, 0.75f);

                    // kSepBias was 0.5f — caused overshooting small overlaps by 6x,
                    // creating frame-to-frame ping-pong in tight multi-entity groups.
                    // A tiny bias (0.05f) still prevents immediate re-overlap without
                    // driving the oscillation.
                    constexpr float kSepBias     = 0.05f;
                    constexpr float kSepFraction = 0.35f;
                    const float correction = (overlap + kSepBias) * kSepFraction * velScale;

                    if (!aFixed && !bFixed) {
                        sepX[i] += nx * correction;  sepY[i] += ny * correction;
                        sepX[j] -= nx * correction;  sepY[j] -= ny * correction;
                    } else if (!aFixed) {
                        sepX[i] += nx * correction * 2.0f;  sepY[i] += ny * correction * 2.0f;
                    } else if (!bFixed) {
                        sepX[j] -= nx * correction * 2.0f;  sepY[j] -= ny * correction * 2.0f;
                    }
                }
            }

            // Apply all accumulated pushes at once so conflicting pushes average out.
            for (int i = 0; i < count; ++i) {
                auto& e = entityManager.entities[i];
                if (e.isDead || (sepX[i] == 0.f && sepY[i] == 0.f)) continue;
                const float nx = e.position.x + sepX[i];
                const float ny = e.position.y + sepY[i];
                if (!RigidbodySystem::WouldCollideWithMap(e.id, nx, ny)) {
                    e.position.x = nx;
                    e.position.y = ny;
                } else {
                    // Axis fallback: apply whichever component clears the wall.
                    if (!RigidbodySystem::WouldCollideWithMap(e.id, e.position.x + sepX[i], e.position.y))
                        e.position.x += sepX[i];
                    if (!RigidbodySystem::WouldCollideWithMap(e.id, e.position.x, e.position.y + sepY[i]))
                        e.position.y += sepY[i];
                }
            }
        }
    }
}
