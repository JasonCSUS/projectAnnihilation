#include "EntityLogic.h"
#include "../engine/NavigationSystem.h"
#include "EntityData.h"
#include "GameEntityManager.h"
#include "GameMain.h"

#include <cmath>
#include <iostream>

static bool IsNearPoint(float x, float y, float targetX, float targetY, float tolerance) {
    const float dx = x - targetX;
    const float dy = y - targetY;
    return (dx * dx + dy * dy) <= (tolerance * tolerance);
}

static void CheckAndApplyTriggerDamage(EntityManager& entityManager, float deltaTime) {
    for (auto& enemy : entityManager.entities) {
        EntityInfo* enemyInfo = EntityData::TryGet(enemy.id);
        if (!enemyInfo) {
            continue;
        }

        if (enemyInfo->controller != ENEMY || enemy.isDead) {
            continue;
        }

        if (enemyInfo->triggerDamageCooldown > 0.0f) {
            enemyInfo->triggerDamageCooldown -= deltaTime;
            if (enemyInfo->triggerDamageCooldown < 0.0f) {
                enemyInfo->triggerDamageCooldown = 0.0f;
            }
        }

        const float centerX = enemy.position.x + enemy.position.w * 0.5f;
        const float centerY = enemy.position.y + enemy.position.h * 0.5f;

        if (IsPointInsideTrigger("room_0", centerX, centerY) &&
            enemyInfo->triggerDamageCooldown <= 0.0f) {
            enemyInfo->hp -= 30;
            if (enemyInfo->hp < 0) {
                enemyInfo->hp = 0;
            }

            enemyInfo->triggerDamageCooldown = 75.0f;

            if (enemyInfo->hp == 0) {
                enemy.isDead = true;
                NavigationSystem::Instance().StopNavigation(entityManager, enemy.id, true);
            }
        }
    }
}

void UpdateEnemyAI(EntityManager& entityManager,
                   GameEntityManager& gameEntityManager,
                   float deltaTime) {
    int defaultX = 350;
    int defaultY = 350;

    Vec2 room0LooseApproach{defaultX, defaultY};

    const float destinationTolerance = 20.0f;

    // Cache player entities once per frame instead of scanning all entities
    // inside each enemy's inner loop (was O(enemies × all_entities)).
    std::vector<Entity*> players;
    for (auto& e : entityManager.entities) {
        if (e.isDead) continue;
        EntityInfo* info = EntityData::TryGet(e.id);
        if (info && info->controller == PLAYER) {
            players.push_back(&e);
        }
    }

    for (auto& enemy : entityManager.entities) {
        EntityInfo* enemyInfo = EntityData::TryGet(enemy.id);
        if (!enemyInfo) {
            continue;
        }

        if (enemyInfo->controller != ENEMY || enemy.isDead) {
            continue;
        }

        if (enemyInfo->state == UNIT_STATE_YIELDING || enemyInfo->state == UNIT_STATE_STANDBY) {
            continue;
        }

        if (enemyInfo->isStatic) {
            continue;
        }

        if (enemyInfo->repathCooldown > 0.0f) {
            enemyInfo->repathCooldown -= deltaTime;
            if (enemyInfo->repathCooldown < 0.0f) enemyInfo->repathCooldown = 0.0f;
        }

        bool targetFound = false;
        const float enemyCenterX = enemy.position.x + enemy.position.w * 0.5f;
        const float enemyCenterY = enemy.position.y + enemy.position.h * 0.5f;

        for (Entity* potentialTarget : players) {
            if (potentialTarget->isDead) continue;

            const float targetCenterX = potentialTarget->position.x + potentialTarget->position.w * 0.5f;
            const float targetCenterY = potentialTarget->position.y + potentialTarget->position.h * 0.5f;

            const float dx = enemyCenterX - targetCenterX;
            const float dy = enemyCenterY - targetCenterY;
            const float distanceSquared = dx * dx + dy * dy;

            if (std::fabs(dx) > std::fabs(dy)) {
                gameEntityManager.SetEntityDirection(enemy.id, dx < 0.0f ? LEFT : RIGHT);
            } else {
                gameEntityManager.SetEntityDirection(enemy.id, dy < 0.0f ? UP : DOWN);
            }

            if (distanceSquared <= enemyInfo->visionRange * enemyInfo->visionRange) {
                targetFound = true;

                if (distanceSquared <= enemyInfo->attackRange * enemyInfo->attackRange) {
                    enemyInfo->state = UNIT_STATE_ATTACKING;
                    NavigationSystem::Instance().StopNavigation(entityManager, enemy.id, true);
                } else {
                    enemyInfo->state = UNIT_STATE_CHASING;

                    const float goalDx = targetCenterX - enemyInfo->lastPathGoalX;
                    const float goalDy = targetCenterY - enemyInfo->lastPathGoalY;
                    const bool targetMoved = (goalDx * goalDx + goalDy * goalDy) > (64.0f * 64.0f);
                    const bool pathExhausted = enemy.pathIndex >= enemy.path.size();

                    if ((pathExhausted || targetMoved) && !enemy.hasPendingPathUpdate && enemyInfo->repathCooldown <= 0.0f) {
                        NavigationSystem::Instance().RequestMoveStrict(
                            entityManager,
                            enemy.id,
                            targetCenterX,
                            targetCenterY,
                            enemy.radius,
                            false
                        );
                        enemyInfo->lastPathGoalX = targetCenterX;
                        enemyInfo->lastPathGoalY = targetCenterY;
                        enemyInfo->repathCooldown = 0.5f + (enemy.id % 5) * 0.04f;
                    }
                }

                break;
            }
        }

        if (!targetFound) {
            const float homeDx = enemyCenterX - static_cast<float>(defaultX);
            const float homeDy = enemyCenterY - static_cast<float>(defaultY);
            const float homeDistanceSquared = homeDx * homeDx + homeDy * homeDy;

            if (IsNearPoint(enemyCenterX, enemyCenterY,
                            static_cast<float>(defaultX),
                            static_cast<float>(defaultY),
                            destinationTolerance)) {
                enemyInfo->state = UNIT_STATE_IDLE;
                NavigationSystem::Instance().StopNavigation(entityManager, enemy.id, true);
            } else {
                enemyInfo->state = UNIT_STATE_MOVING;

                if (enemy.pathIndex >= enemy.path.size() && !enemy.hasPendingPathUpdate && enemyInfo->repathCooldown <= 0.0f) {
                    const bool sameRegionAsHome = NavigationSystem::Instance().ArePointsInSamePrimaryRegion(
                        enemyCenterX,
                        enemyCenterY,
                        static_cast<float>(defaultX),
                        static_cast<float>(defaultY),
                        enemy.radius
                    );

                    const bool useLooseHomeApproach = !sameRegionAsHome && homeDistanceSquared > (270.0f * 270.0f);

                    if (useLooseHomeApproach) {
                        const bool issuedLoose = NavigationSystem::Instance().RequestMoveLoose(
                            entityManager,
                            enemy.id,
                            static_cast<float>(room0LooseApproach.x),
                            static_cast<float>(room0LooseApproach.y),
                            enemy.radius,
                            false
                        );

                        if (issuedLoose) {
                            NavigationSystem::Instance().QueueMove(
                                entityManager,
                                enemy.id,
                                static_cast<float>(defaultX),
                                static_cast<float>(defaultY),
                                enemy.radius,
                                false
                            );
                        } else {
                            NavigationSystem::Instance().RequestMoveStrict(
                                entityManager,
                                enemy.id,
                                static_cast<float>(defaultX),
                                static_cast<float>(defaultY),
                                enemy.radius,
                                false
                            );
                        }
                        enemyInfo->repathCooldown = 0.75f + (enemy.id % 5) * 0.05f;
                    } else {
                        NavigationSystem::Instance().RequestMoveStrict(
                            entityManager,
                            enemy.id,
                            static_cast<float>(defaultX),
                            static_cast<float>(defaultY),
                            enemy.radius,
                            false
                        );
                        enemyInfo->repathCooldown = 0.75f + (enemy.id % 5) * 0.05f;
                    }
                }
            }
        }
    }

    CheckAndApplyTriggerDamage(entityManager, deltaTime);
}