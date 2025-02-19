#include "GameMain.h"
#include "../engine/EntityManager.h"
#include "../engine/GameLoop.h"
#include "../engine/MapLoader.h"
#include "../engine/Character.h"
#include <iostream>

void UpdateGame(float deltaTime) {
    std::cout << "Game update running with deltaTime: " << deltaTime << " seconds" << std::endl;
    // Game logic here, e.g., updating AI, animations, physics, etc.
}

void GameMain(SDL_Window *window, SDL_Renderer *renderer) {
    std::cout << "Starting GameMain..." << std::endl;

    EntityManager entityManager;
    InitializeSprites();
    // Example entity addition
    SDL_Texture* spriteSheet = LoadTexture("./assets/placeholderSpriteSheet.bmp", renderer);
    entityManager.LoadTexture(1, spriteSheet);

    SDL_FRect playerPos = { 1000.0f, 1000.0f, 64.0f, 64.0f };
    Sprite playerSprite = idleDown[0];

    std::cout << "Player sprite: (" << playerSprite.x << ", " << playerSprite.y << ")" << std::endl;
    entityManager.AddEntity(1, 1, &playerSprite, playerPos);

    // Load map textures and add them as map tiles
    AddMapTile("./assets/map1.bmp", 0, 0, renderer);
    AddMapTile("./assets/map2.bmp", 2000, 0, renderer);

    // Start the game loop
    GameLoop(window, renderer, entityManager, UpdateGame);
}
