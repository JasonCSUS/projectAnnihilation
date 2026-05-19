#ifndef GAMEENTITYMANAGER_H
#define GAMEENTITYMANAGER_H

#include "../engine/EntityManager.h"
#include "../engine/MovementSystem.h"
#include <SDL3/SDL.h>
#include <string>
#include <unordered_map>

class MapLoader;

struct GameEntityDefinition {
    std::string name;
    std::string spritePath;
    int radius = 32;
    int spriteX = 64;
    int spriteY = 64;
    float moveSpeed = 100.0f;
    bool isStatic = false;
    bool massive = false;
    bool heroic = false;
    bool titanic = false;
};

class GameEntityManager {
public:
    void Clear();

    bool RegisterSheetForEntity(EntityManager& entityManager,
                                MapLoader& mapLoader,
                                SDL_Renderer* renderer,
                                const std::string& entityName,
                                const std::string& spritePath,
                                int cellWidth,
                                int cellHeight,
                                int rows,
                                int cols);

    void RegisterDefinition(const GameEntityDefinition& def);
    const GameEntityDefinition* TryGetDefinition(const std::string& entityName) const;

    int SpawnUnit(EntityManager& entityManager,
                  const std::string& entityName,
                  float x,
                  float y,
                  int renderPriority = 0);

    int SpawnBuilding(EntityManager& entityManager,
                      const std::string& entityName,
                      float x,
                      float y,
                      int renderPriority = 0);

    void KillEntity(EntityManager& entityManager, int entityId);

    void SetEntityDirection(int entityId, Direction direction);
    Direction GetEntityDirection(int entityId) const;

    void Update(EntityManager& entityManager, float deltaTime);
    bool IsEntityStatic(int entityId) const;
    bool IsEntityHeroic(int entityId) const;
    bool IsEntityMassive(int entityId) const;
    int GetEntityController(int entityId) const;
    bool ShouldEntitiesCollide(int entityIdA, int entityIdB) const;

private:
    struct RegisteredVisual {
        int sheetId = -1;
        bool isBuilding = false;
    };

private:
    std::unordered_map<std::string, GameEntityDefinition> definitions;
    std::unordered_map<std::string, RegisteredVisual> visuals;
    std::unordered_map<int, Direction> directions;

    int nextSheetId = 1;
    MovementSystem movementSystem;
};

#endif