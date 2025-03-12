#include "EntityLogic.h"
#include "../engine/GlobalTasks.h"
#include "../engine/EntityManager.h"
#include "../engine/NavMesh.h"  // Use the singleton navmesh interface
#include "GameMain.h"
#include <cmath>
#include <vector>
#include <iostream>
#include <future>
#include <chrono>
#include <tbb/task_group.h>

//---------------------------------------------------------------------
// Applies default damage to enemies within a certain radius.
void CheckAndApplyDefaultDamage(EntityManager& entityManager, int defaultX, int defaultY) {
    const float damageRadius = 450.0f;
    for (auto& enemy : entityManager.entities) {
        if (enemy.controller == ENEMY) {
            float dx = enemy.position.x - defaultX;
            float dy = enemy.position.y - defaultY;
            float distSq = dx * dx + dy * dy;
            if (distSq <= damageRadius * damageRadius) {
                enemy.hp -= 100;
                if (enemy.hp < 0) enemy.hp = 0;
                std::cout << "Enemy " << enemy.id << " took damage. New hp: " << enemy.hp << std::endl;
                if (enemy.hp == 0) {
                    enemy.isDead = true;
                }
            }
        }
    }
}

//---------------------------------------------------------------------
// Checks an enemy's pending asynchronous path update.
// If the future is ready, sets the enemy's path (a vector of Vec2)
// and clears the pending flag.
void ProcessPendingPathUpdates(Entity& enemy) {
    if (enemy.hasPendingPathUpdate && enemy.asyncPathFuture.valid()) {
        if (enemy.asyncPathFuture.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
            enemy.path = enemy.asyncPathFuture.get();
            enemy.hasPendingPathUpdate = false;
        }
    }
}

//---------------------------------------------------------------------
// Asynchronously updates an enemy's path using the new navmesh singleton.
// This function obtains the starting and destination polygon indices via the singleton,
// computes an A* polygon path, and then smooths it with the funnel algorithm.
// The resulting path (a vector of Vec2 points) is returned asynchronously.
void AsyncUpdateEnemyPath(Entity& enemy, int destX, int destY) {
    const int threshold = 10; // Tolerance threshold
    if (enemy.hasPendingPathUpdate &&
        std::abs(enemy.lastQueuedDestX - destX) <= threshold &&
        std::abs(enemy.lastQueuedDestY - destY) <= threshold) {
        return;
    }
    enemy.lastQueuedDestX = destX;
    enemy.lastQueuedDestY = destY;
    
    // Capture the enemy's current position as the starting point.
    Vec2 startPoint = { static_cast<int>(enemy.position.x), static_cast<int>(enemy.position.y) };
    Vec2 goalPoint  = { destX, destY };
    
    // Obtain polygon indices using the singleton.
    int startPoly  = NavMesh::Instance().GetPolygonIndexAt(startPoint.x, startPoint.y);
    if (startPoly == -1) {
        std::cout << "No valid start polygon found for enemy " << enemy.id << "\n";
        return;
    }
    int targetPoly = NavMesh::Instance().GetPolygonIndexAt(destX, destY);
    if (targetPoly == -1) {
        std::cout << "No valid target polygon found for destination (" << destX << ", " << destY << ")\n";
        return;
    }
    
    // Create a promise for a vector of Vec2 points.
    auto promisePtr = std::make_shared<std::promise<std::vector<Vec2>>>();
    enemy.asyncPathFuture = promisePtr->get_future();
    
    // Schedule asynchronous task using the global task group.
    g_taskGroup.run([startPoly, targetPoly, startPoint, goalPoint, promisePtr]() {
        try {
            // Compute the polygon connectivity path.
            std::vector<int> polyPath = NavMesh::Instance().FindPath(startPoly, targetPoly);
            // Smooth the path with the funnel algorithm.
            std::vector<Vec2> smoothPath = NavMesh::Instance().FunnelPath(polyPath, startPoint, goalPoint);
            promisePtr->set_value(smoothPath);
        }
        catch (...) {
            promisePtr->set_exception(std::current_exception());
        }
    });
    enemy.hasPendingPathUpdate = true;
}

//---------------------------------------------------------------------
// Main enemy AI update function using the new navmesh singleton.
void UpdateEnemyAI(EntityManager& entityManager) {
    const int defaultX = 960;
    const int defaultY = 840;
    const int threshold = 10;  // Tolerance for destination differences
    
    for (auto& enemy : entityManager.entities) {
        if (enemy.controller != ENEMY)
            continue;
        
        // Process any pending async path update.
        ProcessPendingPathUpdates(enemy);
        
        bool targetFound = false;
        SDL_FPoint targetPos = {0, 0};
        
        // Look for a PLAYER within vision range.
        for (auto& potentialTarget : entityManager.entities) {
            if (potentialTarget.controller == PLAYER) {
                float dx = enemy.position.x - potentialTarget.position.x;
                float dy = enemy.position.y - potentialTarget.position.y;
                float distanceSquared = dx * dx + dy * dy;
                if (distanceSquared <= enemy.visionRange * enemy.visionRange) {
                    targetFound = true;
                    targetPos = { potentialTarget.position.x, potentialTarget.position.y };
                    
                    // If in attack range, clear the path.
                    if (distanceSquared <= enemy.attackRange * enemy.attackRange) {
                        if (!enemy.path.empty()) {
                            enemy.path.clear();
                        }
                    } else {
                        // Otherwise, update path if needed.
                        if (!enemy.hasPendingPathUpdate) {
                            if (enemy.path.empty()) {
                                AsyncUpdateEnemyPath(enemy, static_cast<int>(targetPos.x), static_cast<int>(targetPos.y));
                            } else {
                                Vec2 lastPoint = enemy.path.back();
                                if (std::abs(lastPoint.x - static_cast<int>(targetPos.x)) > threshold ||
                                    std::abs(lastPoint.y - static_cast<int>(targetPos.y)) > threshold) {
                                    AsyncUpdateEnemyPath(enemy, static_cast<int>(targetPos.x), static_cast<int>(targetPos.y));
                                }
                            }
                        }
                    }
                    break; // Pursue one target at a time.
                }
            }
        }
        
        // If no PLAYER is found, target the default location.
        if (!targetFound && !enemy.hasPendingPathUpdate) {
            if (enemy.path.empty()) {
                AsyncUpdateEnemyPath(enemy, defaultX, defaultY);
            } else {
                Vec2 lastPoint = enemy.path.back();
                if (std::abs(lastPoint.x - defaultX) > threshold ||
                    std::abs(lastPoint.y - defaultY) > threshold) {
                    AsyncUpdateEnemyPath(enemy, defaultX, defaultY);
                }
            }
        }
    }
    
    // Optionally, apply default damage.
    CheckAndApplyDefaultDamage(entityManager, defaultX, defaultY);
}
