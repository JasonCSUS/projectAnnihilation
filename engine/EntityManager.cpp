#include "EntityManager.h"
#include "AnimationManager.h"
#include "GameLoop.h"
#include "MapLoader.h"
#include "NavigationSystem.h"

#include <SDL3/SDL.h>
#include <iostream>

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
    static int nextID = 1;

    Entity entity{};
    entity.id = nextID++;
    entity.controller = controller;
    entity.radius = radius;
    entity.position = position;
    entity.unitType = unitType;
    entity.speed = speed;
    entity.visionRange = visionRange;
    entity.attackRange = attackRange;
    entity.hp = hp;

    std::cout << "Adding entity: " << entity.id << " controlled by: " << controller << std::endl;

    animationManager.AddAnimation(entity.id, animation);
    entities.push_back(std::move(entity));
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
    NavigationSystem::Instance().Update(*this, deltaTime);
    movementSystem.Update(entities, animationManager, deltaTime);
    animationManager.UpdateAnimations(deltaTime);

    /*
    for (size_t i = 0; i < entities.size(); i++) {
        for (size_t j = i + 1; j < entities.size(); j++) {
            if (CheckCircularCollision(entities[i].position, entities[i].radius, entities[j].position, entities[j].radius)) {
                std::cout << "Collision detected between Entity " << entities[i].id << " and Entity " << entities[j].id << std::endl;
            }
        }
    }
    */
}

void EntityManager::RenderEntities(SDL_Renderer* renderer, float& cameraX, float& cameraY, float& cameraW, float& cameraH, float /*deltaTime*/) {
    for (auto& entity : entities) {
        if (spriteSheets.find(entity.unitType) != spriteSheets.end()) {
            SDL_Texture* texture = spriteSheets[entity.unitType];
            Animation& animation = animationManager.GetAnimation(entity.id);

            if (animation.frames.empty()) {
                continue;
            }

            int renderX = static_cast<int>(entity.position.x - cameraX);
            int renderY = static_cast<int>(entity.position.y - cameraY);

            int spriteW = animation.spriteW;
            int spriteH = animation.spriteH;

            if (renderX + spriteW < 0 || renderX > cameraW ||
                renderY + spriteH < 0 || renderY > cameraH) {
                continue;
            }

            bool isLeft = (entity.lastDirection == LEFT);
            animationManager.RenderEntity(renderer, texture, animation, renderX, renderY, isLeft);
        } else {
            std::cerr << "No sprite sheet found for entity: " << entity.id << std::endl;
        }
    }
}