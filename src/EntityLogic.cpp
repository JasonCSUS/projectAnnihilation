#include "EntityLogic.h"
#include "../engine/NavigationSystem.h"
#include "GameMain.h"

#include <cmath>
#include <iostream>

static bool IsNearPoint(float x, float y, float targetX, float targetY, float tolerance) {
    const float dx = x - targetX;
    const float dy = y - targetY;
    return (dx * dx + dy * dy) <= (tolerance * tolerance);
}

//---------------------------------------------------------------------
// Applies trigger-based damage to enemies inside trigger_1.
static void CheckAndApplyTriggerDamage(EntityManager& entityManager) {
    for (auto& enemy : entityManager.entities) {
        if (enemy.controller != ENEMY || enemy.isDead) {
            continue;
        }

        const float centerX = enemy.position.x + enemy.position.w * 0.5f;
        const float centerY = enemy.position.y + enemy.position.h * 0.5f;

        if (IsPointInsideTrigger("trigger_1", centerX, centerY)) {
            enemy.hp -= 100;
            if (enemy.hp < 0) enemy.hp = 0;

            std::cout << "Enemy " << enemy.id << " took trigger damage. New hp: " << enemy.hp << std::endl;

            if (enemy.hp == 0) {
                enemy.isDead = true;
                NavigationSystem::Instance().StopNavigation(entityManager, enemy.id, true);
            }
        }
    }
}

//---------------------------------------------------------------------
// Main enemy AI update function.
void UpdateEnemyAI(EntityManager& entityManager) {
    int defaultX = 960;
    int defaultY = 840;
    GetTriggerCenter("trigger_1", defaultX, defaultY);

    const float destinationTolerance = 20.0f;

    for (auto& enemy : entityManager.entities) {
        if (enemy.controller != ENEMY || enemy.isDead) {
            continue;
        }

        bool targetFound = false;
        const float enemyCenterX = enemy.position.x + enemy.position.w * 0.5f;
        const float enemyCenterY = enemy.position.y + enemy.position.h * 0.5f;

        for (auto& potentialTarget : entityManager.entities) {
            if (potentialTarget.controller != PLAYER || potentialTarget.isDead) {
                continue;
            }

            const float targetCenterX = potentialTarget.position.x + potentialTarget.position.w * 0.5f;
            const float targetCenterY = potentialTarget.position.y + potentialTarget.position.h * 0.5f;

            const float dx = enemyCenterX - targetCenterX;
            const float dy = enemyCenterY - targetCenterY;
            const float distanceSquared = dx * dx + dy * dy;

            if (distanceSquared <= enemy.visionRange * enemy.visionRange) {
                targetFound = true;

                if (distanceSquared <= enemy.attackRange * enemy.attackRange) {
                    NavigationSystem::Instance().StopNavigation(entityManager, enemy.id, true);
                } else {
                    // Only request a new path if we do not already have one pending/active.
                    if (enemy.path.empty() && !enemy.hasPendingPathUpdate) {
                        NavigationSystem::Instance().RequestMove(
                            entityManager,
                            enemy.id,
                            targetCenterX,
                            targetCenterY,
                            false
                        );
                    }
                }

                break;
            }
        }

        if (!targetFound) {
            if (IsNearPoint(enemyCenterX, enemyCenterY, static_cast<float>(defaultX), static_cast<float>(defaultY), destinationTolerance)) {
                NavigationSystem::Instance().StopNavigation(entityManager, enemy.id, true);
            } else if (enemy.path.empty() && !enemy.hasPendingPathUpdate) {
                NavigationSystem::Instance().RequestMove(
                    entityManager,
                    enemy.id,
                    static_cast<float>(defaultX),
                    static_cast<float>(defaultY),
                    false
                );
            }
        }
    }

    CheckAndApplyTriggerDamage(entityManager);
}