#ifndef INPUTHANDLER_H
#define INPUTHANDLER_H

#include <SDL3/SDL.h>

class EntityManager;
struct Entity;

class InputHandler {
public:
    virtual ~InputHandler() = default;

    virtual void HandleInput(const SDL_Event& event,
                             EntityManager& entityManager,
                             float& cameraX,
                             float& cameraY) = 0;

    virtual void UpdateFrame(EntityManager& entityManager,
                             float& cameraX,
                             float& cameraY,
                             int screenW,
                             int screenH,
                             float mapWidth,
                             float mapHeight,
                             float deltaTime) = 0;

    virtual Entity* GetSelectedEntity(EntityManager& entityManager) = 0;
};

#endif
