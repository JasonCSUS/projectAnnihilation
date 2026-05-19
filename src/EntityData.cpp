#include "EntityData.h"
#include "../engine/EntityManager.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <unordered_map>
#include <unordered_set>

namespace {
std::unordered_map<int, EntityInfo> g_entityData;
}

namespace EntityData {

EntityInfo& GetOrCreate(int entityId) {
    return g_entityData[entityId];
}

EntityInfo* TryGet(int entityId) {
    auto it = g_entityData.find(entityId);
    if (it == g_entityData.end()) {
        return nullptr;
    }
    return &it->second;
}

void Remove(int entityId) {
    g_entityData.erase(entityId);
}

void Clear() {
    g_entityData.clear();
}

void InitializeUnit(int entityId,
                    int unitType,
                    int controller,
                    const std::string& entityName,
                    float baseSpeed,
                    float moveSpeedMultiplier,
                    float attackSpeedMultiplier,
                    float visionRange,
                    float attackRange,
                    int hp,
                    int armor,
                    int damage,
                    float armorPenPercent,
                    float damageIncrease,
                    float damageReduction,
                    float damageIncreaseReduction,
                    float critDamageReduction,
                    int aetherReward,
                    int resonanceReward,
                    const std::string& unitSpawnedOnAttack,
                    const std::string& unitSpawnedOnDeath,
                    int amountUnitsSpawned,
                    const std::string& auraId,
                    bool isStatic,
                    bool massive,
                    bool heroic,
                    bool titanic) {
    EntityInfo& info = g_entityData[entityId];
    info.unitType = unitType;
    info.controller = controller;
    info.entityName = entityName;

    info.baseSpeed = baseSpeed;
    info.moveSpeedMultiplier = moveSpeedMultiplier;
    info.attackSpeedMultiplier = attackSpeedMultiplier;

    info.visionRange = visionRange;
    info.attackRange = attackRange;

    info.hp = hp;
    info.maxHp = hp;
    info.shield = 0;
    info.maxShield = 0;
    info.armor = armor;
    info.shieldArmor = 0;
    info.damage = damage;

    info.armorPenPercent = armorPenPercent;
    info.damageIncrease = damageIncrease;
    info.damageReduction = damageReduction;
    info.damageIncreaseReduction = damageIncreaseReduction;
    info.critDamageReduction = critDamageReduction;

    info.aetherReward = aetherReward;
    info.resonanceReward = resonanceReward;
    info.unitSpawnedOnAttack = unitSpawnedOnAttack;
    info.unitSpawnedOnDeath = unitSpawnedOnDeath;
    info.amountUnitsSpawned = amountUnitsSpawned;
    info.auraId = auraId;

    info.state = UNIT_STATE_IDLE;
    info.collisionFrustration = 0.0f;
    info.yieldUntil = 0.0f;
    info.lastCollisionPartnerId = -1;

    info.isStatic = isStatic;
    info.massive = massive;
    info.heroic = heroic;
    info.titanic = titanic;
}

void SyncWithEntities(const EntityManager& entityManager) {
    std::unordered_set<int> liveIds;
    liveIds.reserve(entityManager.entities.size());

    for (const auto& entity : entityManager.entities) {
        if (!entity.isDead) {
            liveIds.insert(entity.id);
        }
    }

    for (auto it = g_entityData.begin(); it != g_entityData.end();) {
        if (liveIds.find(it->first) == liveIds.end()) {
            it = g_entityData.erase(it);
        } else {
            ++it;
        }
    }
}

int GetStatePriority(int state) {
    switch (state) {
        case UNIT_STATE_ATTACKING: return 5;
        case UNIT_STATE_CHASING:   return 4;
        case UNIT_STATE_MOVING:    return 3;
        case UNIT_STATE_YIELDING:  return 2;
        case UNIT_STATE_STANDBY:   return 1;
        case UNIT_STATE_IDLE:
        default:                   return 0;
    }
}

std::string GetDisplayName(const EntityInfo& info) {
    if (info.entityName.empty()) {
        return "Unknown";
    }

    std::string result = info.entityName;
    for (char& c : result) {
        if (c == '_') {
            c = ' ';
        }
    }

    bool capitalize = true;
    for (char& c : result) {
        if (std::isspace(static_cast<unsigned char>(c))) {
            capitalize = true;
            continue;
        }

        if (capitalize) {
            c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
            capitalize = false;
        }
    }

    return result;
}

float GetFinalMoveSpeed(const EntityInfo& info) {
    return info.baseSpeed * info.moveSpeedMultiplier;
}

float GetDisplayedAttackSpeed(const EntityInfo& info) {
    return std::max(0.0f, info.attackSpeedMultiplier);
}

int GetDisplayedDamage(const EntityInfo& info) {
    return static_cast<int>(std::lround(static_cast<float>(info.damage) * (1.0f + info.damageIncrease)));
}

int GetDisplayedArmor(const EntityInfo& info) {
    return info.armor;
}

int GetDisplayedShieldArmor(const EntityInfo& info) {
    return info.shieldArmor;
}

} // namespace EntityData
