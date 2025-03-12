#include "GameMain.h"
#include "../engine/EntityManager.h"
#include "../engine/GameLoop.h"
#include "../engine/MapLoader.h"
#include "../engine/Character.h"
#include "../engine/AnimationManager.h"
#include "../engine/InputHandler.h"
#include "EntityLogic.h"
#include <iostream>
#include <vector>

EntityManager entityManager;
InputHandler inputHandler;
MapLoader mapLoader;
float elapsedTime;
void Spawn(EntityManager& entityManager, int unitType, int x, int y) {
    int r=32;
    float baseSpeed=150.0f;
    int controller=PLAYER;
    float visionRange;
    float attackRange;
    int hp;
    switch(unitType)
    {
        case UNIT1:
            r=32; baseSpeed=250.0f; controller=PLAYER; visionRange=500.0f; attackRange=500.0f; hp=150;
            break;
        case UNIT2:
            r=32; baseSpeed=150.0f; controller=ENEMY; visionRange=500.0f; attackRange=300.0f; hp=20;
            break;
        default:
            break;
    }
    SDL_FRect pos = {x, y, r*2, r*2};
    std::vector<Sprite> frames = idleDown;  // This makes a copy.
    Animation animation;
    animation.frames = frames;
    animation.frameTime = 0.25f;
    animation.spriteW = r * 2;
    animation.spriteH = r * 2;
    animation.elapsedTime=0;
    animation.currentFrame=0;    
    animation.elapsedTime = (static_cast<float>(rand()) / RAND_MAX) * animation.frameTime;
    animation.currentFrame = rand() % frames.size();   
    entityManager.AddEntity(controller, r, pos, unitType, animation, baseSpeed, visionRange, attackRange, hp);
}

void UpdateGame(float deltaTime) {
    elapsedTime+=deltaTime;
    UpdateEnemyAI(entityManager);
    // Game logic here, e.g., updating AI, animations, physics, etc.
    if(elapsedTime>=0.4f)
    {
        Spawn(entityManager, UNIT2, 2870, 1670 );
        Spawn(entityManager, UNIT2, 2800, 1670 );
        Spawn(entityManager, UNIT2, 2900, 1670 );
        Spawn(entityManager, UNIT2, 2840, 1670 );

        
        elapsedTime-=0.4f;
    }
}

//start method
void GameMain(SDL_Window *window, SDL_Renderer *renderer) {
    std::cout << "Starting GameMain..." << std::endl;

    // Load player sprite sheet
    SDL_Texture* unit1Sprite = mapLoader.LoadTexture("./assets/placeholderSpriteSheet.bmp", renderer);
    entityManager.LoadTexture(UNIT1, unit1Sprite);
    SDL_Texture* unit1Sprite2 = mapLoader.LoadTexture("./assets/placeholderSpriteSheet2.bmp", renderer);
    entityManager.LoadTexture(UNIT2, unit1Sprite2);
    Spawn(entityManager, UNIT1, 770, 660);
    Spawn(entityManager, UNIT2, 2870, 1670 );
    
    // Load map textures and add them as map tiles
    mapLoader.LoadMapTile("./assets/map1.bmp", 0, 0, renderer);
    mapLoader.LoadMapTile("./assets/map2.bmp", 2000, 0, renderer);
    // Load navmesh for this tile
    NavMesh::Instance().LoadFromFile("./assets/navmesh_polygons.nav");
    // Start the game loop
    GameLoop(window, renderer, mapLoader, entityManager, inputHandler, UpdateGame);
}
