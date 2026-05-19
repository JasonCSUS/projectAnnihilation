#include "GameMain.h"

#include "EntityDefinitions.h"
#include "EntityLogic.h"
#include "EntityData.h"
#include "GameContext.h"
#include "Metadata.h"
#include "MinimapSection.h"
#include "RuntimeData.h"
#include "SelectionPanelSection.h"
#include "SpawnFactory.h"
#include "SpawnerSystem.h"
#include "UnitAnimations.h"
#include "UnitCollisionSystem.h"

#include "../engine/GameLoop.h"
#include "../engine/NavMesh.h"
#include "../engine/NavPriorDB.h"
#include "../engine/NavigationSystem.h"

#include <iostream>
#include <memory>
#include <vector>

void UpdateGame(float deltaTime) {
    elapsedTime += deltaTime;

    gameEntityManager.Update(entityManager, deltaTime);

    UpdateSpawners(deltaTime);
    UpdateEnemyAI(entityManager, gameEntityManager, deltaTime);
    UnitCollisionSystem::Instance().Update(entityManager, gameEntityManager, deltaTime);
    EntityData::SyncWithEntities(entityManager);

    entityManager.RemoveDeadEntities();
    selectionState.RemoveMissing(entityManager);
}

void GameMain(SDL_Window* window, SDL_Renderer* renderer) {

    gameEntityManager.Clear();
    EntityData::Clear();
    ClearRuntimeData();
    selectionState.Clear();
    entityManager.entities.reserve(256);

    inputHandler.SetSelectionState(&selectionState);

    UnitAnimations::RegisterDefaultAnimationGroups(entityManager.GetAnimationManager());

    if (!mapLoader.LoadMap("./assets/map_preview.bmp", renderer)) {
        std::cerr << "Failed to load map image.\n";
    }

    if (!NavMesh::Instance().LoadFromFile("./assets/navmesh_polygons.nav")) {
        std::cerr << "Failed to load navmesh.\n";
    }

    if (!NavMesh::Instance().LoadRuntimeBlockersFromJson("./assets/navmesh.json")) {
        std::cerr << "Failed to load runtime nav blockers from navmesh.json.\n";
    }

    NavMesh::Instance().InitializeClearanceBuckets(20, 3, 10);

    if (!NavPriorDB::Instance().LoadFromFile("./assets/nav_priors.json")) {
        std::cerr << "Failed to load nav priors from nav_priors.json.\n";
    }

    if (!LoadRuntimeMetadata("./assets/navmesh.json")) {
        std::cerr << "Failed to load runtime metadata.\n";
    }

    if (!LoadEntityDefinitionsFromProject("./assets/entities.json")) {
        std::cerr << "Failed to load entity definitions from entities.json.\n";
    }

    RegisterEntitySheets(renderer);
    RefreshActiveSpawnersForRoom("room_1");

    NavigationSystem& nav = NavigationSystem::Instance();

    nav.InitializePathCache("assets/navcache.bin");

    const bool startupPrecomputeOk = nav.InitializeStartupPrecompute(
        "./assets/navmesh.json",
        "assets/navstatecache.bin",
        "room_0",
        std::vector<int>{20, 30, 40}
    );

    std::cout << "GameMain: startup precompute "
            << (startupPrecomputeOk ? "OK" : "FAILED") << "\n";

    if (nav.IsPathCacheDirty()) {
        nav.SavePathCacheToFile();
    }

    int playerX = 270;
    int playerY = 460;
    GetPointPosition("player_spawn", playerX, playerY);

    const int playerId = SpawnPlayerUnit("vanguard", playerX, playerY);
    if (playerId >= 0) {
        selectionState.SetSingle(playerId);
    }

    gameHUD.ClearSections();
    gameHUD.AddSection(std::make_unique<MinimapSection>(
        inputHandler,
        entityManager,
        gameEntityManager
    ));
    gameHUD.AddSection(std::make_unique<SelectionPanelSection>(selectionState));

    GameLoop(window, renderer, mapLoader, entityManager, inputHandler, UpdateGame, gameHUD);
}
