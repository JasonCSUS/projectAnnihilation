#ifndef INPUTHANDLER_H
#define INPUTHANDLER_H

#include <SDL3/SDL.h>
#include "MapLoader.h"
#include "NavigationSystem.h"

class EntityManager;
struct Entity;

class InputHandler {
public:
    virtual ~InputHandler() = default;
    virtual void HandleInput(const SDL_Event& event, EntityManager& entityManager, MapLoader& mapLoader, float& cameraX, float& cameraY);
    virtual Entity* GetSelectedEntity();

private:
    Entity* FindSelectedEntity(EntityManager& entityManager);
    bool IsEntityNearDestination(const Entity& entity, float destX, float destY, float tolerance) const;

protected:
    int selectedEntity = -1;
    int startX = 0, startY = 0;
    int lastX = 0, lastY = 0;
    bool dragging = false;
    const int DRAG_THRESHOLD = 20;

    // Player move target tracking
    bool hasMoveDestination = false;
    float moveDestinationX = 0.0f;
    float moveDestinationY = 0.0f;
    float arrivalTolerance = 20.0f;
};

#endif