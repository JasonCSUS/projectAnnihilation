#include "EntityManager.h"
#include "AnimationManager.h"
#include "GameLoop.h"
#include "MapLoader.h"
#include <SDL3/SDL.h>
#include <iostream>
#include <future>
#include <chrono>

bool CheckCircularCollision(const SDL_FRect& a, int radiusA, const SDL_FRect& b, int radiusB) {
    float ax = a.x + a.w / 2;
    float ay = a.y + a.h;
    float bx = b.x + b.w / 2;
    float by = b.y + b.h;
    float dx = ax - bx;
    float dy = ay - by;
    float distanceSquared = dx * dx + dy * dy;
    float radiusSum = radiusA + radiusB;
    return distanceSquared < (radiusSum * radiusSum);
}

void EntityManager::LoadTexture(int unitType, SDL_Texture* texture) {
    std::cout << "Loading texture for: " << unitType << std::endl;
    spriteSheets[unitType] = texture;
    if (texture) {
        std::cout << "Texture loaded successfully for: " << unitType << std::endl;
    } else {
        std::cerr << "Failed to load texture for: " << unitType << std::endl;
    }
}

void EntityManager::AddEntity(int controller, int radius, const SDL_FRect& position, int unitType, Animation animation, float speed, float visionRange, float attackRange, int hp) {
    static int nextID = 1; // Persistent counter for unique IDs
    int id = nextID++; // Assign and increment ID
    std::cout << "Adding entity: " << id << " controlled by: " << controller << std::endl;
    animationManager.AddAnimation(id, animation);    
    entities.push_back({id, controller, radius, position, unitType, speed, visionRange, attackRange, hp});
}

void EntityManager::RemoveDeadEntities() {
    for (auto it = entities.begin(); it != entities.end(); ) {
        if (it->isDead) {
            animationManager.RemoveAnimation(it->id);
            it = entities.erase(it);
        } else {
            ++it;
        }
    }
}

void EntityManager::UpdateEntities(float deltaTime) {    
    movementSystem.Update(entities, animationManager, deltaTime);
    animationManager.UpdateAnimations(deltaTime);
    /*
    for (size_t i = 0; i < entities.size(); i++) {
        for (size_t j = i + 1; j < entities.size(); j++) {
            if (CheckCircularCollision(entities[i].position, entities[i].radius, entities[j].position, entities[j].radius)) {
                std::cout << "Collision detected between Entity " << entities[i].id << " and " << entities[j].id << std::endl;
            }
        }
    }*/
}

void EntityManager::RenderEntities(SDL_Renderer* renderer, float& cameraX, float& cameraY, float& cameraW, float& cameraH, float deltaTime) {
    for (auto& entity : entities) {
        if (spriteSheets.find(entity.unitType) != spriteSheets.end()) {
            SDL_Texture* texture = spriteSheets[entity.unitType];
            Animation& animation = animationManager.GetAnimation(entity.id);
            
            if (animation.frames.empty()) {
                continue;
            }// Skip if no animation exists

            // Get entity position relative to the camera
            int renderX = static_cast<int>(entity.position.x - cameraX);
            int renderY = static_cast<int>(entity.position.y - cameraY);

            // Get sprite dimensions
            int spriteW = animation.spriteW;
            int spriteH = animation.spriteH;

            // **Bounds Check: If entity is outside the camera bounds, skip rendering**
            if (renderX + spriteW < 0 || renderX > cameraW || 
                renderY + spriteH < 0 || renderY > cameraH) {
                continue;
            }

            // Determine flip direction
            bool isLeft = (entity.lastDirection == LEFT);
            // Update animation and render
            animationManager.RenderEntity(renderer, texture, animation, renderX, renderY, isLeft);
        } else {
            std::cerr << "No sprite sheet found for entity: " << entity.id << std::endl;
        }
    }
}

