#include "MovementSystem.h"
#include "EntityManager.h"
#include "NavMesh.h"    // For reference if needed
#include <SDL3/SDL.h>
#include <cmath>
#include <iostream>

void MovementSystem::Update(std::vector<Entity>& entities, AnimationManager& animationManager, float deltaTime) {
    for (auto& entity : entities) {
        if (entity.path.empty()) {
            continue; // No path to follow
        }

        // Get the next point in the path (now a Vec2, not a NavNode pointer)
        Vec2 nextPoint = entity.path.front();
        float targetX = static_cast<float>(nextPoint.x);
        float targetY = static_cast<float>(nextPoint.y);

        // Calculate direction towards the next point
        float dx = targetX - entity.position.x;
        float dy = targetY - entity.position.y;
        float distance = std::sqrt(dx * dx + dy * dy);
        float speed = entity.speed * deltaTime;
        
        // Swap animations based on movement direction
        if (std::abs(dx) > std::abs(dy)) {
            animationManager.SwapSprite(entity.id, walkRight); // Use walkLeft if needed
            entity.lastDirection = (dx > 0) ? RIGHT : LEFT;
        } else {
            if (dy > 0) {
                animationManager.SwapSprite(entity.id, walkDown); // Moving down
                entity.lastDirection = DOWN;
            } else {
                animationManager.SwapSprite(entity.id, walkUp); // Moving up
                entity.lastDirection = UP;
            }
        } 
        
        if (distance > speed) {
            // Move gradually toward the point
            entity.position.x += (dx / distance) * speed;
            entity.position.y += (dy / distance) * speed;
        } else {
            // Snap to the target point and remove it from the path.
            entity.position.x = targetX;
            entity.position.y = targetY;
            entity.path.erase(entity.path.begin());

            // When path is completed, set idle animation.
            if (entity.path.empty()) {
                switch (entity.lastDirection) {
                    case UP: animationManager.SwapSprite(entity.id, idleUp); break;
                    case DOWN: animationManager.SwapSprite(entity.id, idleDown); break;
                    case LEFT: animationManager.SwapSprite(entity.id, idleRight); break;
                    case RIGHT: animationManager.SwapSprite(entity.id, idleRight); break;
                }
                animationManager.SetChangeState(entity.id, true);
            }
        }
    }
}
