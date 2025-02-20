#include "EntityManager.h"
#include "AnimationManager.h"
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

void EntityManager::LoadTexture(int name, SDL_Texture* texture) {
    std::cout << "Loading texture for: " << name << std::endl;
    spriteSheets[name] = texture;
    if (texture) {
        std::cout << "Texture loaded successfully for: " << name << std::endl;
    } else {
        std::cerr << "Failed to load texture for: " << name << std::endl;
    }
}

void EntityManager::AddEntity(int id, int controller, int radius, const SDL_FRect& position) {
    std::cout << "Adding entity: " << id << " controlled by: " << controller << std::endl;
    entities.push_back({ id, controller, radius, position });
}

void EntityManager::UpdateEntities(float deltaTime, AnimationManager& animationManager) {    
    movementSystem.Update(entities, animationManager, deltaTime);
    for (size_t i = 0; i < entities.size(); i++) {
        for (size_t j = i + 1; j < entities.size(); j++) {
            if (CheckCircularCollision(entities[i].position, entities[i].radius, entities[j].position, entities[j].radius)) {
                std::cout << "Collision detected between Entity " << entities[i].id << " and " << entities[j].id << std::endl;
            }
        }
    }
}

void EntityManager::RenderEntities(SDL_Renderer* renderer, AnimationManager& animationManager, const SDL_FRect& camera) {
    for (auto& entity : entities) {
        if (spriteSheets.find(entity.id) != spriteSheets.end()) {
            SDL_Texture* texture = spriteSheets[entity.id];

            Animation* animation = animationManager.GetAnimation(entity.id);

            bool isLeft = (entity.lastDirection == LEFT);

            if (animation) {
                animation->Update(0.016f);
                animationManager.RenderEntity(renderer, texture, animation, 
                    static_cast<int>(entity.position.x - camera.x),
                    static_cast<int>(entity.position.y - camera.y), isLeft);
            }
        } else {
            std::cerr << "No sprite sheet found for entity: " << entity.id << std::endl;
        }
    }
}

void EntityManager::HandleInput(const SDL_Event& event) {
    movementSystem.HandleInput(event);
}
