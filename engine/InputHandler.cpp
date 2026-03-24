#include "InputHandler.h"
#include "EntityManager.h"
#include "MapLoader.h"
#include "NavigationSystem.h"
#include <iostream>
#include <cmath>

Entity* InputHandler::FindSelectedEntity(EntityManager& entityManager) {
    for (auto& entity : entityManager.entities) {
        if (entity.id == selectedEntity) {
            return &entity;
        }
    }
    return nullptr;
}

bool InputHandler::IsEntityNearDestination(const Entity& entity, float destX, float destY, float tolerance) const {
    const float centerX = entity.position.x;
    const float centerY = entity.position.y;

    const float dx = centerX - destX;
    const float dy = centerY - destY;
    return (dx * dx + dy * dy) <= (tolerance * tolerance);
}

void InputHandler::HandleInput(const SDL_Event& event, EntityManager& entityManager, MapLoader& /*mapLoader*/, float& cameraX, float& cameraY) {
    if (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN) {
        if (event.button.button == SDL_BUTTON_LEFT) {
            dragging = false;
            startX = event.button.x;
            startY = event.button.y;
        }

        if (event.button.button == SDL_BUTTON_RIGHT) {
            if (selectedEntity == -1) {
                std::cout << "No entity selected, right-click ignored.\n";
                return;
            }

            Entity* entityPtr = FindSelectedEntity(entityManager);
            if (!entityPtr) {
                std::cout << "Selected entity with ID " << selectedEntity << " not found.\n";
                return;
            }

            moveDestinationX = event.button.x + cameraX;
            moveDestinationY = event.button.y + cameraY;
            hasMoveDestination = true;

            // Clear any stale path immediately and issue one fresh request.
            NavigationSystem::Instance().RequestMove(
                entityManager,
                entityPtr->id,
                moveDestinationX,
                moveDestinationY,
                true
            );
        }
    }

    if (event.type == SDL_EVENT_KEY_DOWN) {
        if (event.key.key == SDLK_X) {
            float mouseX, mouseY;
            SDL_GetMouseState(&mouseX, &mouseY);

            float worldX = mouseX + cameraX;
            float worldY = mouseY + cameraY;

            std::cout << "[X Key Pressed] Cursor World Position: ("
                      << worldX << ", " << worldY << ")\n";
        }
    }

    if (event.type == SDL_EVENT_MOUSE_MOTION) {
        if (SDL_GetMouseState(NULL, NULL) & SDL_BUTTON_MASK(SDL_BUTTON_LEFT)) {
            cameraX -= event.motion.xrel;
            cameraY -= event.motion.yrel;

            int totalDx = std::abs(event.motion.x - startX);
            int totalDy = std::abs(event.motion.y - startY);

            if (totalDx > DRAG_THRESHOLD || totalDy > DRAG_THRESHOLD) {
                dragging = true;
            }
        }
    }

    if (event.type == SDL_EVENT_MOUSE_BUTTON_UP) {
        if (!dragging && event.button.button == SDL_BUTTON_LEFT) {
            int mouseX = event.button.x;
            int mouseY = event.button.y;

            float worldX = mouseX + cameraX;
            float worldY = mouseY + cameraY;

            selectedEntity = -1;
            for (auto& entity : entityManager.entities) {
                SDL_FRect entityRect = entity.position;
                if (worldX >= entityRect.x - entityRect.w / 2 && worldX <= entityRect.x + entityRect.w / 2 &&
                    worldY >= entityRect.y - entityRect.h / 2 && worldY <= entityRect.y + entityRect.h / 2) {
                    selectedEntity = entity.id;
                    std::cout << "Selected entity ID: " << selectedEntity << std::endl;
                    break;
                }
            }
        }

        dragging = false;
    }

    // Player movement upkeep: only request a path if we have a destination,
    // we are not already there, and we currently have no path pending/active.
    if (hasMoveDestination) {
        Entity* entityPtr = FindSelectedEntity(entityManager);
        if (entityPtr && !entityPtr->isDead) {
            if (IsEntityNearDestination(*entityPtr, moveDestinationX, moveDestinationY, arrivalTolerance)) {
                hasMoveDestination = false;
                NavigationSystem::Instance().StopNavigation(entityManager, entityPtr->id, true);
            } else if (entityPtr->path.empty() && !entityPtr->hasPendingPathUpdate) {
                NavigationSystem::Instance().RequestMove(
                    entityManager,
                    entityPtr->id,
                    moveDestinationX,
                    moveDestinationY,
                    false
                );
            }
        }
    }
}

Entity* InputHandler::GetSelectedEntity() {
    return nullptr;
}