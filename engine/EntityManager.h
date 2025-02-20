#ifndef ENTITYMANAGER_H
#define ENTITYMANAGER_H

#include <SDL3/SDL.h>
#include <vector>
#include <map>
#include <string>
#include "Character.h"
#include "MovementSystem.h"
#include "AnimationManager.h"

// Entity structure now holds the necessary data for each entity.
struct Entity {
    int controller;
    int id;
    int radius;
    SDL_FRect position;
    Direction lastDirection = DOWN;
};

class EntityManager {
public:
    void LoadTexture(int name, SDL_Texture* texture);
    void AddEntity(int id, int controller, int radius, const SDL_FRect& position);
    void RenderEntities(SDL_Renderer* renderer, AnimationManager& animationManager, const SDL_FRect& camera);
    void UpdateEntities(float deltaTime,  AnimationManager& animationManager);
    void HandleInput(const SDL_Event& event);
    const std::vector<Entity>& GetEntities() const { return entities; }
private:
    std::map<int, SDL_Texture*> spriteSheets;
    std::vector<Entity> entities;
    MovementSystem movementSystem;
};

#endif
