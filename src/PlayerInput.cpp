#include "PlayerInput.h"

#include "../engine/EntityManager.h"
#include "../engine/NavigationSystem.h"

#include "EntityData.h"
#include "SelectionState.h"

#include <SDL3/SDL.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

void PlayerInput::SetSelectionState(SelectionState* selectionStateIn) {
    selectionState = selectionStateIn;
}

void PlayerInput::SetIgnoreNextLeftRelease(bool ignore) {
    ignoreNextLeftRelease = ignore;
}

Entity* PlayerInput::FindPrimarySelectedEntity(EntityManager& entityManager) {
    if (!selectionState) {
        return nullptr;
    }
    return entityManager.GetEntityById(selectionState->GetPrimarySelectedId());
}

bool PlayerInput::IsEntityNearDestination(const Entity& entity, float destX, float destY, float tolerance) const {
    const float centerX = entity.position.x;
    const float centerY = entity.position.y;

    const float dx = centerX - destX;
    const float dy = centerY - destY;
    return (dx * dx + dy * dy) <= (tolerance * tolerance);
}

static bool ShouldUseLoosePlayerMove(const Entity& entity, float destX, float destY) {
    const bool sameRegion = NavigationSystem::Instance().ArePointsInSamePrimaryRegion(
        entity.position.x,
        entity.position.y,
        destX,
        destY,
        entity.radius
    );

    if (sameRegion) {
        return false;
    }

    const float dx = entity.position.x - destX;
    const float dy = entity.position.y - destY;
    const float distanceSquared = dx * dx + dy * dy;
    return distanceSquared > (400.0f * 400.0f);
}

void PlayerInput::ClampCamera(float& cameraX,
                              float& cameraY,
                              int screenW,
                              int screenH,
                              float mapWidth,
                              float mapHeight) const {
    const float maxCameraX = std::max(0.0f, mapWidth - static_cast<float>(screenW));
    const float maxCameraY = std::max(0.0f, mapHeight - static_cast<float>(screenH));

    cameraX = std::max(0.0f, std::min(cameraX, maxCameraX));
    cameraY = std::max(0.0f, std::min(cameraY, maxCameraY));
}

void PlayerInput::PanCameraToWorld(float worldX,
                                   float worldY,
                                   float& cameraX,
                                   float& cameraY,
                                   int screenW,
                                   int screenH,
                                   float mapWidth,
                                   float mapHeight) {
    cameraX = worldX - static_cast<float>(screenW) * 0.5f;
    cameraY = worldY - static_cast<float>(screenH) * 0.5f;

    ClampCamera(cameraX, cameraY, screenW, screenH, mapWidth, mapHeight);
}

int PlayerInput::PickEntityAtWorld(EntityManager& entityManager, float worldX, float worldY) const {
    int bestId = -1;
    float bestScore = 1.0e30f;

    for (const auto& entity : entityManager.entities) {
        if (entity.isDead) {
            continue;
        }

        const float halfW = entity.position.w * 0.5f;
        const float halfH = entity.position.h * 0.5f;
        if (worldX < entity.position.x - halfW || worldX > entity.position.x + halfW ||
            worldY < entity.position.y - halfH || worldY > entity.position.y + halfH) {
            continue;
        }

        const float dx = worldX - entity.position.x;
        const float dy = worldY - entity.position.y;
        const float score = dx * dx + dy * dy;
        if (score < bestScore) {
            bestScore = score;
            bestId = entity.id;
        }
    }

    return bestId;
}

void PlayerInput::ApplySelectionClick(EntityManager& entityManager, float worldX, float worldY, bool additive) {
    if (!selectionState) {
        return;
    }

    const int clickedId = PickEntityAtWorld(entityManager, worldX, worldY);
    if (clickedId < 0) {
        if (!additive) {
            selectionState->Clear();
        }
        return;
    }

    if (additive) {
        selectionState->AddSelection({clickedId});
        selectionState->SetPrimarySelectedId(clickedId);
    } else {
        selectionState->SetSingle(clickedId);
    }
}

void PlayerInput::ApplySelectionBox(EntityManager& entityManager, float cameraX, float cameraY, bool additive) {
    if (!selectionState || !selectionState->IsMarqueeVisible()) {
        return;
    }

    const SDL_FRect rect = selectionState->GetMarqueeRect();
    std::vector<int> hits;
    hits.reserve(entityManager.entities.size());

    for (const auto& entity : entityManager.entities) {
        if (entity.isDead) {
            continue;
        }

        const float screenX = entity.position.x - cameraX;
        const float screenY = entity.position.y - cameraY;

        if (screenX >= rect.x && screenX <= rect.x + rect.w &&
            screenY >= rect.y && screenY <= rect.y + rect.h) {
            hits.push_back(entity.id);
        }
    }

    if (additive) {
        selectionState->AddSelection(hits);
        if (!hits.empty()) {
            selectionState->SetPrimarySelectedId(hits.front());
        }
    } else {
        selectionState->SetSelection(hits);
    }
}

void PlayerInput::IssueMoveCommandToSelection(EntityManager& entityManager,
                                              float worldX,
                                              float worldY,
                                              bool forceRepath) {
    if (!selectionState || selectionState->Empty()) {
        return;
    }

    moveDestinationX = worldX;
    moveDestinationY = worldY;
    hasMoveDestination = true;

    bool issuedAny = false;

    for (int entityId : selectionState->GetSelectedIds()) {
        Entity* entityPtr = entityManager.GetEntityById(entityId);
        if (!entityPtr || entityPtr->isDead) {
            continue;
        }

        const EntityInfo* info = EntityData::TryGet(entityId);
        if (!info || info->controller != PLAYER || info->isStatic) {
            continue;
        }

        const bool useLoose = ShouldUseLoosePlayerMove(*entityPtr, moveDestinationX, moveDestinationY);
        bool issued = false;

        if (useLoose) {
            issued = NavigationSystem::Instance().RequestMoveLoose(
                entityManager,
                entityPtr->id,
                moveDestinationX,
                moveDestinationY,
                entityPtr->radius,
                forceRepath
            );

            if (issued) {
                NavigationSystem::Instance().QueueMove(
                    entityManager,
                    entityPtr->id,
                    moveDestinationX,
                    moveDestinationY,
                    entityPtr->radius,
                    false
                );
            } else {
                issued = NavigationSystem::Instance().RequestMoveStrict(
                    entityManager,
                    entityPtr->id,
                    moveDestinationX,
                    moveDestinationY,
                    entityPtr->radius,
                    forceRepath
                );
            }
        } else {
            issued = NavigationSystem::Instance().RequestMoveStrict(
                entityManager,
                entityPtr->id,
                moveDestinationX,
                moveDestinationY,
                entityPtr->radius,
                forceRepath
            );
        }

        issuedAny = issuedAny || issued;
    }

    if (!issuedAny) {
        hasMoveDestination = false;
    }
}

void PlayerInput::MoveSelectedToWorld(EntityManager& entityManager,
                                      float worldX,
                                      float worldY) {
    IssueMoveCommandToSelection(entityManager, worldX, worldY, true);
}

void PlayerInput::HandleInput(const SDL_Event& event,
                              EntityManager& entityManager,
                              float& cameraX,
                              float& cameraY) {
    if (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN) {
        if (event.button.button == SDL_BUTTON_LEFT) {
            draggingSelection = false;
            startX = event.button.x;
            startY = event.button.y;
            if (selectionState) {
                selectionState->BeginMarquee(static_cast<float>(startX), static_cast<float>(startY));
            }
        }

        if (event.button.button == SDL_BUTTON_MIDDLE) {
            middleDraggingCamera = true;
            middleDragLastX = event.button.x;
            middleDragLastY = event.button.y;
        }

        if (event.button.button == SDL_BUTTON_RIGHT) {
            const float worldX = event.button.x + cameraX;
            const float worldY = event.button.y + cameraY;
            IssueMoveCommandToSelection(entityManager, worldX, worldY, true);
        }
    }

    if (event.type == SDL_EVENT_KEY_DOWN && event.key.key == SDLK_SPACE) {
        if (FindPrimarySelectedEntity(entityManager) != nullptr) {
            focusSelectedRequested = true;
        }
    }

    if (event.type == SDL_EVENT_KEY_DOWN && event.key.key == SDLK_X) {
        float mouseX = 0.0f;
        float mouseY = 0.0f;
        SDL_GetMouseState(&mouseX, &mouseY);

        const float worldX = mouseX + cameraX;
        const float worldY = mouseY + cameraY;
    }

    if (event.type == SDL_EVENT_MOUSE_MOTION) {
        if (middleDraggingCamera) {
            const int dx = event.motion.x - middleDragLastX;
            const int dy = event.motion.y - middleDragLastY;
            cameraX -= static_cast<float>(dx);
            cameraY -= static_cast<float>(dy);
            middleDragLastX = event.motion.x;
            middleDragLastY = event.motion.y;
        }

        if (SDL_GetMouseState(NULL, NULL) & SDL_BUTTON_MASK(SDL_BUTTON_LEFT)) {
            const int totalDx = std::abs(event.motion.x - startX);
            const int totalDy = std::abs(event.motion.y - startY);

            if (totalDx > DRAG_THRESHOLD || totalDy > DRAG_THRESHOLD) {
                draggingSelection = true;
            }

            if (selectionState) {
                selectionState->UpdateMarquee(static_cast<float>(event.motion.x),
                                              static_cast<float>(event.motion.y));
            }
        }
    }

    if (event.type == SDL_EVENT_MOUSE_BUTTON_UP && event.button.button == SDL_BUTTON_MIDDLE) {
        middleDraggingCamera = false;
    }

    if (event.type == SDL_EVENT_MOUSE_BUTTON_UP && event.button.button == SDL_BUTTON_LEFT) {
        if (ignoreNextLeftRelease) {
            ignoreNextLeftRelease = false;
            if (selectionState) {
                selectionState->EndMarquee();
            }
            draggingSelection = false;
            return;
        }

        const bool additive = (SDL_GetModState() & SDL_KMOD_SHIFT) != 0;

        if (draggingSelection && selectionState && selectionState->IsMarqueeVisible()) {
            ApplySelectionBox(entityManager, cameraX, cameraY, additive);
        } else {
            const float worldX = event.button.x + cameraX;
            const float worldY = event.button.y + cameraY;
            ApplySelectionClick(entityManager, worldX, worldY, additive);
        }

        if (selectionState) {
            selectionState->EndMarquee();
        }

        draggingSelection = false;
    }

    if (hasMoveDestination && selectionState) {
        bool anyControllableSelected = false;
        bool allArrived = true;

        for (int entityId : selectionState->GetSelectedIds()) {
            Entity* entityPtr = entityManager.GetEntityById(entityId);
            if (!entityPtr || entityPtr->isDead) {
                continue;
            }

            const EntityInfo* info = EntityData::TryGet(entityId);
            if (!info || info->controller != PLAYER || info->isStatic) {
                continue;
            }

            anyControllableSelected = true;

            if (IsEntityNearDestination(*entityPtr, moveDestinationX, moveDestinationY, arrivalTolerance)) {
                NavigationSystem::Instance().StopNavigation(entityManager, entityPtr->id, true);
            } else {
                allArrived = false;
            }
        }

        if (!anyControllableSelected || allArrived) {
            hasMoveDestination = false;
        }
    }
}

void PlayerInput::UpdateFrame(EntityManager& entityManager,
                              float& cameraX,
                              float& cameraY,
                              int screenW,
                              int screenH,
                              float mapWidth,
                              float mapHeight,
                              float deltaTime) {
    (void)entityManager;

    float mouseX = 0.0f;
    float mouseY = 0.0f;
    const SDL_MouseButtonFlags buttons = SDL_GetMouseState(&mouseX, &mouseY);

    if (focusSelectedRequested) {
        Entity* selected = FindPrimarySelectedEntity(entityManager);
        if (selected) {
            PanCameraToWorld(selected->position.x,
                             selected->position.y,
                             cameraX,
                             cameraY,
                             screenW,
                             screenH,
                             mapWidth,
                             mapHeight);
        }
        focusSelectedRequested = false;
    }

    if (!middleDraggingCamera) {
        float panX = 0.0f;
        float panY = 0.0f;

        if (mouseX <= static_cast<float>(edgePanMargin)) {
            panX -= edgePanSpeed * deltaTime;
        } else if (mouseX >= static_cast<float>(screenW - edgePanMargin)) {
            panX += edgePanSpeed * deltaTime;
        }

        if (mouseY <= static_cast<float>(edgePanMargin)) {
            panY -= edgePanSpeed * deltaTime;
        } else if (mouseY >= static_cast<float>(screenH - edgePanMargin)) {
            panY += edgePanSpeed * deltaTime;
        }

        cameraX += panX;
        cameraY += panY;
    }

    if ((buttons & SDL_BUTTON_MASK(SDL_BUTTON_MIDDLE)) == 0) {
        middleDraggingCamera = false;
    }

    ClampCamera(cameraX, cameraY, screenW, screenH, mapWidth, mapHeight);
}

Entity* PlayerInput::GetSelectedEntity(EntityManager& entityManager) {
    return FindPrimarySelectedEntity(entityManager);
}
