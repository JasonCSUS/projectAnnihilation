#ifndef ENTITYMANAGER_H
#define ENTITYMANAGER_H

#include <SDL3/SDL.h>
#include <vector>
#include <map>
#include <string>
#include "Character.h"
#include "MovementSystem.h"

enum UnitType {
    Unit1 = 1,
    Unit2  = 2,
    Unit3   = 3
};
enum Controller {
    Player = 1,
    Ally = 2,
    Enemy = 3,
};

// Entity structure now holds the necessary data for each entity.
struct Entity {
    int controller;
    int id;
    Sprite* sprite;
    SDL_FRect position;
};

class EntityManager {
public:
    void LoadTexture(int name, SDL_Texture* texture);
    void AddEntity( int id, int controller, Sprite* sprite, const SDL_FRect& position);
    // Updated RenderEntities to take a camera offset parameter.
    void RenderEntities(SDL_Renderer* renderer, const SDL_FRect& camera);
    void UpdateEntities(float deltaTime);
    void HandleInput(const SDL_Event& event);
    const std::vector<Entity>& GetEntities() const { return entities; }
private:
    std::map<int, SDL_Texture*> spriteSheets;
    std::vector<Entity> entities;
    MovementSystem movementSystem;
};

#endif