#include "GameLoop.h"
#include "MapLoader.h"
#include "EntityManager.h"
#include "Character.h"
#include "MovementSystem.h"
#include "InputHandler.h"
#include "HUD.h"
#include "NavMesh.h"

#include <SDL3/SDL.h>
#include <iostream>
#include <vector>

struct Camera {
    float x = 500.0f;
    float y = 500.0f;
    float w = 0.0f;
    float h = 0.0f;

    void Initialize(SDL_Window* window) {
        int windowW = 0;
        int windowH = 0;
        SDL_GetWindowSize(window, &windowW, &windowH);
        w = static_cast<float>(windowW);
        h = static_cast<float>(windowH);
    }
} camera;

SDL_Renderer* gRenderer = nullptr;

void GameLoop(SDL_Window* window,
              SDL_Renderer* renderer,
              MapLoader& mapLoader,
              EntityManager& entityManager,
              InputHandler& inputHandler,
              UpdateFunc updateGame,
              HUD& hud) {
    bool running = true;
    bool showNavDebug = false;
    SDL_Event event;
    gRenderer = renderer;
    camera.Initialize(window);

    constexpr uint64_t TARGET_MS_PER_FRAME = 16; // ~62.5 FPS cap

    uint64_t lastCounter = SDL_GetTicks();

    while (running) {
        const uint64_t frameStart = SDL_GetTicks();

        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_EVENT_QUIT ||
                (event.type == SDL_EVENT_KEY_DOWN && event.key.key == SDLK_ESCAPE)) {
                running = false;
            }

            if (event.type == SDL_EVENT_KEY_DOWN && event.key.key == SDLK_F3) {
                showNavDebug = !showNavDebug;
            }

            if (event.type == SDL_EVENT_WINDOW_RESIZED) {
                const int newWidth = event.window.data1;
                const int newHeight = event.window.data2;

                camera.w = static_cast<float>(newWidth);
                camera.h = static_cast<float>(newHeight);
            }

            const bool hudConsumed =
                hud.HandleEvent(event, camera.x, camera.y,
                                static_cast<int>(camera.w),
                                static_cast<int>(camera.h));

            if (!hudConsumed) {
                inputHandler.HandleInput(event, entityManager, camera.x, camera.y);
            }
        }

        const uint64_t currentCounter = SDL_GetTicks();
        const float deltaTime = static_cast<float>((currentCounter - lastCounter) / 1000.0f);
        lastCounter = currentCounter;

        inputHandler.UpdateFrame(entityManager,
                                 camera.x,
                                 camera.y,
                                 static_cast<int>(camera.w),
                                 static_cast<int>(camera.h),
                                 static_cast<float>(mapLoader.GetMapWidth()),
                                 static_cast<float>(mapLoader.GetMapHeight()),
                                 deltaTime);

        updateGame(deltaTime);
        entityManager.RemoveDeadEntities();
        hud.Update(deltaTime);

        SDL_RenderClear(renderer);

        mapLoader.RenderMap(renderer, camera.x, camera.y);
        entityManager.RenderEntities(renderer, camera.x, camera.y, camera.w, camera.h, deltaTime);
        if (showNavDebug) NavMesh::Instance().DebugRender(renderer, camera.x, camera.y);

        hud.Render(renderer,
                   mapLoader,
                   entityManager,
                   camera.x,
                   camera.y,
                   static_cast<int>(camera.w),
                   static_cast<int>(camera.h));

        SDL_RenderPresent(renderer);

        const uint64_t frameMs = SDL_GetTicks() - frameStart;
        if (frameMs < TARGET_MS_PER_FRAME) {
            SDL_Delay(static_cast<Uint32>(TARGET_MS_PER_FRAME - frameMs));
        }
    }

    mapLoader.Clear();
}
