#include "MovementSystem.h"
#include "EntityManager.h"

void MovementSystem::Update(std::vector<Entity>& entities, float deltaTime) {
    for (auto& entity : entities) {
        if (this->up) entity.position.y -= this->speed * deltaTime;
        if (this->down) entity.position.y += this->speed * deltaTime;
        if (this->left) entity.position.x -= this->speed * deltaTime;
        if (this->right) entity.position.x += this->speed * deltaTime;
    }
}

void MovementSystem::HandleInput(const SDL_Event& event) {
    if (event.type == SDL_EVENT_KEY_DOWN) {
        if (event.key.key == SDLK_W) up = true;
        if (event.key.key == SDLK_S) down = true;
        if (event.key.key == SDLK_A) left = true;
        if (event.key.key == SDLK_D) right = true;
    }
    if (event.type == SDL_EVENT_KEY_UP) {
        if (event.key.key == SDLK_W) up = false;
        if (event.key.key == SDLK_S) down = false;
        if (event.key.key == SDLK_A) left = false;
        if (event.key.key == SDLK_D) right = false;
    }
}
