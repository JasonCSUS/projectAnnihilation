#include "GameMain.h"
#include "../engine/EntityManager.h"
#include "../engine/GameLoop.h"
#include "../engine/MapLoader.h"
#include "../engine/Character.h"
#include "../engine/AnimationManager.h"
#include <iostream>
#include <vector>

std::vector<std::vector<bool>> collisionMap;


void UpdateGame(float deltaTime) {
    // Game logic here, e.g., updating AI, animations, physics, etc.
}

void GameMain(SDL_Window *window, SDL_Renderer *renderer) {
    std::cout << "Starting GameMain..." << std::endl;

    EntityManager entityManager;
    AnimationManager animationManager;
    
    // Load player sprite sheet
    SDL_Texture* spriteSheet = LoadTexture("./assets/placeholderSpriteSheet.bmp", renderer);
    entityManager.LoadTexture(1, spriteSheet);

    // Define player position
    SDL_FRect playerPos = { 1000.0f, 1000.0f, 64.0f, 64.0f };
    
    // Create player animations
    Animation unit1Animation = { idleDown, 4, 0.2f, 0, 0, 64, 64 };
    
    // Add animations to animation manager
    animationManager.AddAnimation(1, &unit1Animation);
    
    // Add player entity with idle animation
    entityManager.AddEntity(1, 1, 32, playerPos);
    
    // Load map textures and add them as map tiles
    AddMapTile("./assets/map1.bmp", 0, 0, renderer);
    AddMapTile("./assets/map2.bmp", 2000, 0, renderer);

    // Start the game loop
    GameLoop(window, renderer, entityManager, animationManager, UpdateGame);
}
