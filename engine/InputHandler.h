#ifndef INPUTHANDLER_H
#define INPUTHANDLER_H

#include <SDL3/SDL.h>
#include "NavMesh.h"
#include "MapLoader.h"

class EntityManager;
struct Entity;

class InputHandler {
public:
    virtual ~InputHandler() = default; // Ensure proper cleanup for derived classes
    virtual void HandleInput(const SDL_Event& event, EntityManager& entityManager, MapLoader& mapLoader, float& cameraX, float& cameraY);
    virtual Entity* GetSelectedEntity();

protected:
    int selectedEntity = -1; // -1 means no entity is selected.
    int startX = 0, startY = 0;  // Track initial mouse position for drag detection
    int lastX = 0, lastY = 0;    // Track last mouse position for movement
    bool dragging = false;
    const int DRAG_THRESHOLD = 20; // Threshold to determine dragging vs. click
};

#endif
