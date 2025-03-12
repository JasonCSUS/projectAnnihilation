#ifndef ENTITYMANAGER_H
#define ENTITYMANAGER_H

#include <SDL3/SDL.h>
#include <vector>
#include <map>
#include <string>
#include "Character.h"
#include "MovementSystem.h"
#include "AnimationManager.h"
#include "GameLoop.h"
#include "MapLoader.h"
#include <future>
// Entity structure now holds the necessary data for each entity.
struct Entity {
    int id;
    int controller;
    int radius;
    SDL_FRect position;
    int unitType;
    float speed;
    float visionRange;
    float attackRange;
    int hp;
    bool isDead = false; 
    Direction lastDirection = DOWN;
    std::vector<Vec2> path;
    int lastQueuedDestX = -1;
    int lastQueuedDestY = -1;
    std::future<std::vector<Vec2>> asyncPathFuture;
    bool hasPendingPathUpdate = false;
    float nextPathUpdateTime = 0.0f;
};

class EntityManager {
public:
    void LoadTexture(int name, SDL_Texture* texture);
    void AddEntity(int controller, int radius, const SDL_FRect& position, int unitType, Animation animation, float speed, float visionRange, float attackRange, int hp);
    void RenderEntities(SDL_Renderer* renderer, float& cameraX, float& cameraY, float& cameraW, float& cameraH, float deltaTime);
    void UpdateEntities(float deltaTime);
    //void EntityManager::SetEntityPath(MapLoader& mapLoader, int entityId, int targetX, int targetY);
    void EntityManager::RemoveDeadEntities();
    std::vector<Entity> entities;
private:
    std::map<int, SDL_Texture*> spriteSheets;
    MovementSystem movementSystem;
    AnimationManager animationManager;
};

#endif
