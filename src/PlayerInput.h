#ifndef PLAYERINPUT_H
#define PLAYERINPUT_H

#include "../engine/InputHandler.h"

class SelectionState;

class PlayerInput : public InputHandler {
public:
    void SetSelectionState(SelectionState* selectionStateIn);

    void HandleInput(const SDL_Event& event,
                     EntityManager& entityManager,
                     float& cameraX,
                     float& cameraY) override;

    void UpdateFrame(EntityManager& entityManager,
                     float& cameraX,
                     float& cameraY,
                     int screenW,
                     int screenH,
                     float mapWidth,
                     float mapHeight,
                     float deltaTime) override;

    Entity* GetSelectedEntity(EntityManager& entityManager) override;

    void PanCameraToWorld(float worldX,
                          float worldY,
                          float& cameraX,
                          float& cameraY,
                          int screenW,
                          int screenH,
                          float mapWidth,
                          float mapHeight);

    void MoveSelectedToWorld(EntityManager& entityManager,
                             float worldX,
                             float worldY);

    void SetIgnoreNextLeftRelease(bool ignore);

private:
    Entity* FindPrimarySelectedEntity(EntityManager& entityManager);
    int PickEntityAtWorld(EntityManager& entityManager, float worldX, float worldY) const;
    void ApplySelectionClick(EntityManager& entityManager, float worldX, float worldY, bool additive);
    void ApplySelectionBox(EntityManager& entityManager, float cameraX, float cameraY, bool additive);
    void IssueMoveCommandToSelection(EntityManager& entityManager, float worldX, float worldY, bool forceRepath);
    bool IsEntityNearDestination(const Entity& entity, float destX, float destY, float tolerance) const;
    void ClampCamera(float& cameraX,
                     float& cameraY,
                     int screenW,
                     int screenH,
                     float mapWidth,
                     float mapHeight) const;

private:
    SelectionState* selectionState = nullptr;

    int startX = 0;
    int startY = 0;
    bool draggingSelection = false;
    const int DRAG_THRESHOLD = 20;

    bool hasMoveDestination = false;
    float moveDestinationX = 0.0f;
    float moveDestinationY = 0.0f;
    float arrivalTolerance = 20.0f;

    bool ignoreNextLeftRelease = false;
    bool middleDraggingCamera = false;
    int middleDragLastX = 0;
    int middleDragLastY = 0;

    float edgePanSpeed = 850.0f;
    int edgePanMargin = 16;
    bool focusSelectedRequested = false;
};

#endif
