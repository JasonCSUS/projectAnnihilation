#include "InputHandler.h"
#include "EntityManager.h"
#include "MapLoader.h"
#include "NavMesh.h"  // Use our new navmesh singleton
#include <iostream>
#include <cmath>

void InputHandler::HandleInput(const SDL_Event& event, EntityManager& entityManager, MapLoader& /*mapLoader*/, float& cameraX, float& cameraY) {
    if (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN) {
        if (event.button.button == SDL_BUTTON_LEFT) {
            dragging = false;
        }
        if (event.button.button == SDL_BUTTON_RIGHT) {
            std::cout << "Right-click detected!\n";
            if (selectedEntity != -1) {
                // Find the entity with this id.
                Entity* entityPtr = nullptr;
                for (auto& entity : entityManager.entities) {
                    if (entity.id == selectedEntity) {
                        entityPtr = &entity;
                        break;
                    }
                }
                if (entityPtr) {
                    std::cout << "Entity selected: ID " << entityPtr->id << std::endl;
                    // Convert mouse position to world coordinates.
                    float worldX = event.button.x + cameraX;
                    float worldY = event.button.y + cameraY;
                    std::cout << "Mouse click converted to world coordinates: (" << worldX << ", " << worldY << ")\n";
                    
                    // Use the singleton navmesh to get polygon indices.
                    int targetPoly = NavMesh::Instance().GetPolygonIndexAt(static_cast<int>(worldX), static_cast<int>(worldY));
                    if (targetPoly == -1) {
                        std::cout << "ERROR: No valid polygon found at target coordinates.\n";
                        return;
                    }
                    std::cout << "Target polygon index: " << targetPoly << "\n";

                    int startPoly = NavMesh::Instance().GetPolygonIndexAt(static_cast<int>(entityPtr->position.x), static_cast<int>(entityPtr->position.y));
                    if (startPoly == -1) {
                        std::cout << "ERROR: No valid polygon found at entity position (" 
                                  << entityPtr->position.x << ", " << entityPtr->position.y << ")\n";
                        return;
                    }
                    std::cout << "Start polygon index: " << startPoly << "\n";

                    // Compute an A* path between polygons.
                    std::vector<int> polyPath = NavMesh::Instance().FindPath(startPoly, targetPoly);
                    if (polyPath.empty()) {
                        std::cout << "No polygon path found between start and target.\n";
                        return;
                    }
                    std::cout << "Polygon path found with " << polyPath.size() << " polygons.\n";
                    
                    // Define start and goal points using current entity and target click.
                    Vec2 startPoint = { static_cast<int>(entityPtr->position.x), static_cast<int>(entityPtr->position.y) };
                    Vec2 goalPoint  = { static_cast<int>(worldX), static_cast<int>(worldY) };

                    // Smooth the path using the funnel algorithm.
                    std::vector<Vec2> smoothPath = NavMesh::Instance().FunnelPath(polyPath, startPoint, goalPoint);
                    std::cout << "Smoothed path has " << smoothPath.size() << " points.\n";
                    for (const auto& pt : smoothPath) {
                        std::cout << " -> (" << pt.x << ", " << pt.y << ")\n";
                    }
                    // Set the entity's path to the newly computed smooth path.
                    entityPtr->path = smoothPath;
                } else {
                    std::cout << "Selected entity with ID " << selectedEntity << " not found.\n";
                }
            } else {
                std::cout << "No entity selected, right-click ignored.\n";
            }
        }        
    }

    // Handle key presses (e.g., print cursor world position).
    if (event.type == SDL_EVENT_KEY_DOWN) {
        if (event.key.key == SDLK_X) { 
            float mouseX, mouseY;
            SDL_GetMouseState(&mouseX, &mouseY);
            float worldX = mouseX + cameraX;
            float worldY = mouseY + cameraY;
            std::cout << "[X Key Pressed] Cursor World Position: (" << worldX << ", " << worldY << ")\n";
        }
    }

    // Process mouse motion for camera movement.
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
    
    // On mouse button up, check for entity selection.
    if (event.type == SDL_EVENT_MOUSE_BUTTON_UP) {
        if (!dragging) { // Only select an entity if it was a click.
            int mouseX = event.button.x;
            int mouseY = event.button.y;
            float worldX = mouseX + cameraX;
            float worldY = mouseY + cameraY;
            bool found = false;
            for (auto& entity : entityManager.entities) {
                SDL_FRect entityRect = entity.position;
                if (worldX >= entityRect.x - entityRect.w / 2 && worldX <= entityRect.x + entityRect.w / 2 &&
                    worldY >= entityRect.y - entityRect.h / 2 && worldY <= entityRect.y + entityRect.h / 2) {
                    selectedEntity = entity.id;
                    std::cout << "Selected entity ID: " << selectedEntity << std::endl;
                    found = true;
                    break;
                }
            }
            // Optionally clear selection if no entity was clicked.
            // if (!found) { selectedEntity = -1; }
        }
        dragging = false;
    }
}

Entity* InputHandler::GetSelectedEntity() {
    // Not used in this implementation.
    return nullptr;
}
