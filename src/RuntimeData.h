#ifndef RUNTIMEDATA_H
#define RUNTIMEDATA_H

#include <string>
#include <unordered_map>
#include <vector>

struct RuntimePoint {
    std::string label;
    float x = 0.0f;
    float y = 0.0f;
};

struct RuntimeObject {
    std::string label;
    std::string kind;
    float x = 0.0f;
    float y = 0.0f;
    float w = 0.0f;
    float h = 0.0f;
};

struct RuntimeTrigger {
    std::string label;
    std::string kind;
    float x = 0.0f;
    float y = 0.0f;
    float w = 0.0f;
    float h = 0.0f;
};

struct RuntimeEntityDefinition {
    std::string name;
    std::string spritePath;

    int spriteX = 64;
    int spriteY = 64;

    bool isStatic = false;
    bool massive = false;
    bool heroic = false;
    bool ignoreCollision = false;

    int radius = 32;
    int hp = 100;
    int armor = 0;
    int damage = 10;

    float armorPenPercent = 0.0f;
    float damageReduction = 0.0f;
    float attackRange = 100.0f;
    float visionRange = 300.0f;
    float moveSpeed = 100.0f;
    float attackSpeed = 1.0f;

    int aetherReward = 0;
    int resonanceReward = 0;

    std::string unitSpawnedOnAttack = "none";
    std::string unitSpawnedOnDeath = "none";
    int amountUnitsSpawned = 0;
    std::string auraId = "none";

    std::string ability1 = "none";
    std::string ability2 = "none";
    std::string ability3 = "none";
    std::string ability4 = "none";

    std::unordered_map<std::string, std::string> customFields;
};

enum class SpawnerFamily {
    Base,
    Hatchery
};

struct RuntimeSpawner {
    std::string label;
    std::string buildingEntityName;
    SpawnerFamily family = SpawnerFamily::Base;
    int slotIndex = 0;
    float x = 0.0f;
    float y = 0.0f;
    int spawnedBuildingEntityId = -1;
    bool active = false;
};

extern std::unordered_map<std::string, RuntimeEntityDefinition> g_entityDefs;
extern std::vector<RuntimePoint> g_points;
extern std::vector<RuntimeObject> g_objects;
extern std::vector<RuntimeTrigger> g_triggers;
extern std::vector<RuntimeSpawner> g_activeSpawners;
extern float g_spawnerTimer;

void ClearRuntimeData();

#endif
