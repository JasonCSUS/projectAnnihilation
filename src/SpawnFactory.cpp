#include "SpawnFactory.h"

#include "Difficulty.h"
#include "EntityData.h"
#include "GameContext.h"
#include "RuntimeData.h"
#include "Units.h"

#include <cmath>
#include <iostream>

bool IsEntityAliveById(int entityId) {
    const Entity* entity = entityManager.GetEntityById(entityId);
    return entity && !entity->isDead;
}

int SpawnNamedUnit(const std::string& entityName, int x, int y) {
    auto defIt = g_entityDefs.find(entityName);
    if (defIt == g_entityDefs.end()) {
        std::cerr << "SpawnNamedUnit missing def: " << entityName << std::endl;
        return -1;
    }

    const RuntimeEntityDefinition& def = defIt->second;
    const int entityId = gameEntityManager.SpawnUnit(
        entityManager,
        entityName,
        static_cast<float>(x),
        static_cast<float>(y),
        def.massive ? RENDER_MASSIVE : RENDER_BASE
    );

    if (entityId < 0) {
        return -1;
    }

    const DifficultyProfile& difficulty = GetCurrentDifficultyProfile();

    static int spawnBurstCounter = 0;

    EntityData::InitializeUnit(
        entityId,
        UNIT2,
        ENEMY,
        entityName,
        def.moveSpeed,
        difficulty.enemySpeedMultiplier,
        difficulty.enemySpeedMultiplier,
        def.visionRange,
        def.attackRange,
        std::max(1, static_cast<int>(std::round(def.hp * difficulty.enemyHealthMultiplier))),
        std::max(0, static_cast<int>(std::round(def.armor * difficulty.enemyArmorMultiplier))),
        std::max(0, static_cast<int>(std::round(def.damage * difficulty.enemyDamageMultiplier))),
        def.armorPenPercent,
        difficulty.enemyDamageIncrease,
        difficulty.enemyDamageReduction,
        difficulty.damageIncreaseReduction,
        difficulty.critDamageReduction,
        def.aetherReward,
        def.resonanceReward,
        def.unitSpawnedOnAttack,
        def.unitSpawnedOnDeath,
        def.amountUnitsSpawned,
        def.auraId,
        def.isStatic,
        def.massive,
        def.heroic,
        false
    );

    EntityInfo* spawnedInfo = EntityData::TryGet(entityId);
    if (spawnedInfo) {
        spawnedInfo->repathCooldown = (spawnBurstCounter % 5) * 0.1f;
    }
    ++spawnBurstCounter;

    return entityId;
}

int SpawnNamedBuilding(const std::string& entityName, int x, int y) {
    auto defIt = g_entityDefs.find(entityName);
    if (defIt == g_entityDefs.end()) {
        std::cerr << "SpawnNamedBuilding missing def: " << entityName << std::endl;
        return -1;
    }

    const RuntimeEntityDefinition& def = defIt->second;
    const int entityId = gameEntityManager.SpawnBuilding(
        entityManager,
        entityName,
        static_cast<float>(x),
        static_cast<float>(y),
        def.massive ? RENDER_MASSIVE : RENDER_BASE
    );

    if (entityId < 0) {
        return -1;
    }

    const DifficultyProfile& difficulty = GetCurrentDifficultyProfile();

    EntityData::InitializeUnit(
        entityId,
        UNIT2,
        ENEMY,
        entityName,
        0.0f,
        0.0f,
        1.0f,
        def.visionRange,
        def.attackRange,
        std::max(1, static_cast<int>(std::round(def.hp * difficulty.enemyHealthMultiplier))),
        std::max(0, static_cast<int>(std::round(def.armor * difficulty.enemyArmorMultiplier))),
        std::max(0, static_cast<int>(std::round(def.damage * difficulty.enemyDamageMultiplier))),
        def.armorPenPercent,
        difficulty.enemyDamageIncrease,
        difficulty.enemyDamageReduction,
        difficulty.damageIncreaseReduction,
        difficulty.critDamageReduction,
        def.aetherReward,
        def.resonanceReward,
        def.unitSpawnedOnAttack,
        def.unitSpawnedOnDeath,
        def.amountUnitsSpawned,
        def.auraId,
        true,
        def.massive,
        def.heroic,
        false
    );

    return entityId;
}

int SpawnPlayerUnit(const std::string& entityName, int x, int y) {
    auto defIt = g_entityDefs.find(entityName);
    if (defIt == g_entityDefs.end()) {
        std::cerr << "Missing player entity definition for '" << entityName << "'\n";
        return -1;
    }

    const RuntimeEntityDefinition& def = defIt->second;
    const int playerId = gameEntityManager.SpawnUnit(
        entityManager,
        entityName,
        static_cast<float>(x),
        static_cast<float>(y),
        RENDER_BASE
    );

    if (playerId < 0) {
        return -1;
    }

    EntityData::InitializeUnit(
        playerId,
        UNIT1,
        PLAYER,
        entityName,
        def.moveSpeed,
        1.0f,
        1.0f,
        def.visionRange,
        def.attackRange,
        def.hp,
        def.armor,
        def.damage,
        def.armorPenPercent,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        def.aetherReward,
        def.resonanceReward,
        def.unitSpawnedOnAttack,
        def.unitSpawnedOnDeath,
        def.amountUnitsSpawned,
        def.auraId,
        def.isStatic,
        def.massive,
        def.heroic,
        false
    );

    return playerId;
}
