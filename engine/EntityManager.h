#ifndef ENTITYMANAGER_H
#define ENTITYMANAGER_H

#include <SDL3/SDL.h>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "Character.h"
#include "AnimationManager.h"
#include "NavMesh.h"

enum class NavMoveRequestType {
    Strict,
    Loose
};

struct QueuedMoveRequest {
    bool active = false;
    int targetX = -1;
    int targetY = -1;
    int clearanceBucket = 20;
    NavMoveRequestType type = NavMoveRequestType::Strict;
};

struct Entity {
    int id = 0;
    int radius = 0;
    SDL_FRect position{};
    float speed = 0.0f;
    bool isDead = false;
    float movementAngleDegrees = 270.0f;
    float velX = 0.0f;
    float velY = 0.0f;

    // Generic render ordering only.
    int renderPriority = 0;

    // Path/nav data can remain engine-side.
    std::vector<Vec2> path;
    size_t pathIndex = 0;
    bool hasPendingPathUpdate = false;

    bool navAllowLooseReuse = false;
    uint32_t navRequestGeneration = 0;

    bool navHasMoveTarget = false;
    int navTargetX = -1;
    int navTargetY = -1;
    int navClearanceBucket = 20;
    QueuedMoveRequest navQueuedMove{};
    float nextPathUpdateTime = 0.0f;
};

class EntityManager {
public:
    std::vector<Entity> entities;

    Entity& AddEntity(int radius,
                      const SDL_FRect& position,
                      float speed);

    void RemoveDeadEntities();

    Entity* GetEntityById(int id);
    const Entity* GetEntityById(int id) const;

    void RenderEntities(SDL_Renderer* renderer,
                        float& cameraX,
                        float& cameraY,
                        float& cameraW,
                        float& cameraH,
                        float deltaTime);

    AnimationManager& GetAnimationManager() { return animationManager; }
    const AnimationManager& GetAnimationManager() const { return animationManager; }

private:
    AnimationManager animationManager;
    std::unordered_map<int, size_t> entityIndex;
    mutable std::vector<Entity*> renderScratch;
};

#endif