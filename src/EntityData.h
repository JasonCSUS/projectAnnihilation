#ifndef ENTITYDATA_H
#define ENTITYDATA_H

#include "Units.h"
#include <string>

class EntityManager;

struct EntityInfo {
    int unitType = 0;
    int controller = 0;

    std::string entityName;

    float baseSpeed = 0.0f;
    float moveSpeedMultiplier = 1.0f;
    float attackSpeedMultiplier = 1.0f;

    float visionRange = 0.0f;
    float attackRange = 0.0f;

    int hp = 0;
    int maxHp = 0;
    int shield = 0;
    int maxShield = 0;
    int armor = 0;
    int shieldArmor = 0;
    int damage = 0;

    float armorPenPercent = 0.0f;
    float damageIncrease = 0.0f;
    float damageReduction = 0.0f;
    float damageIncreaseReduction = 0.0f;
    float critDamageReduction = 0.0f;

    int aetherReward = 0;
    int resonanceReward = 0;

    std::string unitSpawnedOnAttack = "none";
    std::string unitSpawnedOnDeath = "none";
    int amountUnitsSpawned = 0;
    std::string auraId = "none";

    int state = UNIT_STATE_IDLE;

    float collisionFrustration = 0.0f;
    float yieldUntil = 0.0f;
    float triggerDamageCooldown = 0.0f;
    int lastCollisionPartnerId = -1;

    float repathCooldown = 0.0f;
    float lastPathGoalX = 0.0f;
    float lastPathGoalY = 0.0f;

    bool isStatic = false;
    bool canBePushed = true;
    bool canYield = true;
    bool massive = false;
    bool heroic = false;
    bool titanic = false;
};

namespace EntityData {
    EntityInfo& GetOrCreate(int entityId);
    EntityInfo* TryGet(int entityId);
    void Remove(int entityId);
    void Clear();

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
                        bool titanic);

    void SyncWithEntities(const EntityManager& entityManager);
    int GetStatePriority(int state);

    std::string GetDisplayName(const EntityInfo& info);
    float GetFinalMoveSpeed(const EntityInfo& info);
    float GetDisplayedAttackSpeed(const EntityInfo& info);
    int GetDisplayedDamage(const EntityInfo& info);
    int GetDisplayedArmor(const EntityInfo& info);
    int GetDisplayedShieldArmor(const EntityInfo& info);
}

#endif
