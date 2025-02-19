#include "EntityManager.h"
#include <SDL3/SDL.h>
#include <iostream>

void EntityManager::LoadTexture(int name, SDL_Texture* texture) {
    std::cout << "Loading texture for: " << name << std::endl;
    spriteSheets[name] = texture;
    if (texture) {
        std::cout << "Texture loaded successfully for: " << name << std::endl;
    } else {
        std::cerr << "Failed to load texture for: " << name << std::endl;
    }
}

void EntityManager::AddEntity(int id, int controller, Sprite* sprite, const SDL_FRect& position) {
    std::cout << "Adding entity: " << id << " controlled by: " << controller << std::endl;
    switch (id) {
        case Unit1:
            sprite->spriteW = 64;
            sprite->spriteH = 64;
            break;
        case Unit2:
            sprite->spriteW = 128;
            sprite->spriteH = 128;
            break;
        case Unit3:
            sprite->spriteW = 96;
            sprite->spriteH = 96;
            break;
        default:
            sprite->spriteW = 64;
            sprite->spriteH = 64;
            break;
    }
    entities.push_back({ id, controller, sprite, position });
}

void EntityManager::RenderEntities(SDL_Renderer* renderer, const SDL_FRect& camera) {
    for (auto& entity : entities) {
        if (spriteSheets.find(entity.id) != spriteSheets.end()) {
            SDL_Texture* texture = spriteSheets[entity.id];
            int renderX = static_cast<int>(entity.position.x - camera.x);
            int renderY = static_cast<int>(entity.position.y - camera.y);
            RenderCharacter(renderer, texture, *entity.sprite, renderX, renderY, false);
        } else {
            std::cerr << "No sprite sheet found for entity: " << entity.id << std::endl;
        }
    }
}

void EntityManager::UpdateEntities(float deltaTime) {
    movementSystem.Update(entities, deltaTime);
}

void EntityManager::HandleInput(const SDL_Event& event) {
    movementSystem.HandleInput(event);
}