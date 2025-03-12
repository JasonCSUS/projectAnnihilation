#include "GameLoop.h"
#include "MapLoader.h"
#include <SDL3/SDL.h>
#include <iostream>
#include "EntityManager.h"
#include "Character.h"
#include "MovementSystem.h"
#include "MapTile.h" 
#include "InputHandler.h"
#include <vector>

struct Camera {
    float x = 500.0f;
    float y = 500.0f;
    float w;
    float h;

    void Initialize(SDL_Window* window) {
        int windowW, windowH;
        SDL_GetWindowSize(window, &windowW, &windowH);
        w = static_cast<float>(windowW);
        h = static_cast<float>(windowH);
    }
} camera;


bool dragging = false;
int lastX, lastY;

void GameLoop(SDL_Window* window, SDL_Renderer* renderer, MapLoader& mapLoader, EntityManager& entityManager, InputHandler& inputHandler, UpdateFunc updateGame) {
    bool running = true;
    SDL_Event event;
    camera.Initialize(window); 
    uint64_t lastCounter = SDL_GetTicks();
    while (running) {
        while (SDL_PollEvent(&event)) {
            //remove this later when exiting is implemented in input handler
            if (event.type == SDL_EVENT_QUIT || (event.type == SDL_EVENT_KEY_DOWN && event.key.key == SDLK_ESCAPE)) {
                running = false;
            }
            if (event.type == SDL_EVENT_WINDOW_RESIZED) {
                int newWidth = event.window.data1;
                int newHeight = event.window.data2;
                
                // Update camera width and height
                camera.w = static_cast<float>(newWidth);
                camera.h = static_cast<float>(newHeight);
            }
            inputHandler.HandleInput(event, entityManager, mapLoader, camera.x, camera.y);
        }
        uint64_t currentCounter = SDL_GetTicks();
        float deltaTime = (float)((currentCounter - lastCounter) / 1000.0f);
        lastCounter = currentCounter;
        updateGame(deltaTime);
        entityManager.UpdateEntities(deltaTime);
        entityManager.RemoveDeadEntities();

        SDL_RenderClear(renderer);
        mapLoader.RenderMap(renderer, camera.x, camera.y);
        entityManager.RenderEntities(renderer, camera.x, camera.y, camera.w, camera.h, deltaTime);
        NavMesh::Instance().DebugRender(renderer, camera.x, camera.y);
        SDL_RenderPresent(renderer);
    }
    mapLoader.Clear();
}
