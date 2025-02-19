#include "GameLoop.h"
#include "MapLoader.h"
#include <SDL3/SDL.h>
#include <iostream>
#include "EntityManager.h"
#include "Character.h"
#include "MovementSystem.h"
#include "MapTile.h" 
#include <vector>

struct Camera {
    float x = 1000.0f;
    float y = 1000.0f;
} camera;

bool dragging = false;
int lastX, lastY;


// Store tiles in a vector
std::vector<MapTile> mapTiles;

void AddMapTile(const std::string& file, int x, int y, SDL_Renderer* renderer) {
    SDL_Texture* texture = LoadTexture(file, renderer);
    if (texture) {
        mapTiles.push_back({ texture, x, y });
        std::cout << "Added map tile: " << file << " at (" << x << ", " << y << ")" << std::endl;
    } else {
        std::cerr << "Failed to load map tile: " << file << std::endl;
    }
}

void GameLoop(SDL_Window* window, SDL_Renderer* renderer, EntityManager& entityManager, UpdateFunc updateGame) {
    bool running = true;
    SDL_Event event;

    SDL_FRect cameraOffset = { camera.x, camera.y, 0, 0 };
    while (running) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_EVENT_QUIT || (event.type == SDL_EVENT_KEY_DOWN && event.key.key == SDLK_ESCAPE)) {
                running = false;
            }
            entityManager.HandleInput(event);
            if (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN) {
                dragging = true;
                lastX = event.button.x;
                lastY = event.button.y;
            }
            if (event.type == SDL_EVENT_MOUSE_BUTTON_UP) {
                dragging = false;
            }
            if (event.type == SDL_EVENT_MOUSE_MOTION && dragging) {
                int dx = event.motion.x - lastX;
                int dy = event.motion.y - lastY;
                camera.x -= dx;
                camera.y -= dy;
                lastX = event.motion.x;
                lastY = event.motion.y;
            }
            cameraOffset = { camera.x, camera.y, 0, 0 };
        }

        float deltaTime = 0.016f;
        updateGame(deltaTime);
        entityManager.UpdateEntities(deltaTime);

        SDL_RenderClear(renderer);
        RenderMap(renderer, mapTiles, camera.x, camera.y);
        entityManager.RenderEntities(renderer, cameraOffset);
        SDL_RenderPresent(renderer);
    }

    for (auto& tile : mapTiles) {
        SDL_DestroyTexture(tile.texture);
    }
    mapTiles.clear();
}
