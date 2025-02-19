#include "GameLoop.h"
#include "MapLoader.h"
#include <SDL3/SDL.h>
#include <iostream>
#include "EntityManager.h"
#include "Character.h"
#include "MovementSystem.h"

struct Camera {
    float x = 1000.0f;
    float y = 1000.0f;
} camera;

bool dragging = false;
int lastX, lastY;

void GameLoop(SDL_Window *window, SDL_Renderer *renderer) {
    bool running = true;
    SDL_Event event;
    InitializeSprites();
    SDL_Texture* map1 = LoadTexture("./assets/map1.bmp", renderer);
    SDL_Texture* map2 = LoadTexture("./assets/map2.bmp", renderer);

    EntityManager entityManager;
    SDL_Texture* spriteSheet = LoadTexture("./assets/placeholderSpriteSheet.bmp", renderer);
    entityManager.LoadTexture(UnitType::Unit1, spriteSheet);

    SDL_FRect playerPos = {1000.0f, 1000.0f, 64.0f, 64.0f};
    Sprite playerSprite = idleDown[0];
    entityManager.AddEntity( Unit1, Player, &playerSprite, playerPos);

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
        entityManager.UpdateEntities(deltaTime);

        SDL_RenderClear(renderer);
        RenderMap(renderer, map1, map2, camera.x, camera.y);
        entityManager.RenderEntities(renderer, cameraOffset);
        SDL_RenderPresent(renderer);
    }

    SDL_DestroyTexture(spriteSheet);
    SDL_DestroyTexture(map1);
    SDL_DestroyTexture(map2);
}
