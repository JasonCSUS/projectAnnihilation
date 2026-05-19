#include "EntityDefinitions.h"

#include "GameContext.h"
#include "GameEntityManager.h"
#include "JsonUtils.h"
#include "RuntimeData.h"

#include <algorithm>
#include <iostream>

bool LoadEntityDefinitionsFromProject(const std::string& jsonPath) {
    g_entityDefs.clear();

    const std::string json = ReadTextFile(jsonPath);
    if (json.empty()) {
        std::cerr << "Failed to read entity json: " << jsonPath << std::endl;
        return false;
    }

    std::string body;
    if (!ExtractSectionArray(json, "entities", body)) {
        return true;
    }

    for (const std::string& obj : SplitTopLevelObjects(body)) {
        RuntimeEntityDefinition def;
        if (!GetStringField(obj, "name", def.name) || def.name.empty()) {
            continue;
        }

        GetStringField(obj, "sprite_path", def.spritePath);
        def.customFields = ParseCustomFields(obj);

        def.isStatic = ParseBoolString(def.customFields, "static", false);
        def.massive = ParseBoolString(def.customFields, "massive", false);
        def.heroic = ParseBoolString(def.customFields, "heroic", false);
        def.ignoreCollision = ParseBoolString(def.customFields, "ignore_collision", false);

        def.spriteX = ParseIntString(def.customFields, "spritex", 64);
        def.spriteY = ParseIntString(def.customFields, "spritey", 64);

        def.radius = ParseIntString(def.customFields, "radius", 32);
        def.hp = ParseIntString(def.customFields, "hp", 100);
        def.armor = ParseIntString(def.customFields, "armor", 0);
        def.damage = ParseIntString(def.customFields, "damage", 10);

        def.armorPenPercent = ParseFloatString(def.customFields, "armor_pen_percent", 0.0f);
        def.damageReduction = ParseFloatString(def.customFields, "damage_reduction", 0.0f);
        def.attackRange = ParseFloatString(def.customFields, "attack_range", 100.0f);
        def.visionRange = ParseFloatString(def.customFields, "vision_range", 300.0f);
        def.moveSpeed = ParseFloatString(def.customFields, "move_speed", 100.0f);
        def.attackSpeed = ParseFloatString(def.customFields, "attack_speed", 2.0f);

        def.aetherReward = ParseIntString(def.customFields, "aether_reward", 0);
        def.resonanceReward = ParseIntString(def.customFields, "resonance_reward", 0);

        def.unitSpawnedOnAttack = ParseStringValue(def.customFields, "unit_spawned_on_attack", "none");
        def.unitSpawnedOnDeath = ParseStringValue(def.customFields, "unit_spawned_on_death", "none");
        def.amountUnitsSpawned = ParseIntString(def.customFields, "amount_units_spawned", 0);
        def.auraId = ParseStringValue(def.customFields, "aura_id", "none");

        def.ability1 = ParseStringValue(def.customFields, "ability_1", "none");
        def.ability2 = ParseStringValue(def.customFields, "ability_2", "none");
        def.ability3 = ParseStringValue(def.customFields, "ability_3", "none");
        def.ability4 = ParseStringValue(def.customFields, "ability_4", "none");

        g_entityDefs[def.name] = def;

        GameEntityDefinition gameDef;
        gameDef.name = def.name;
        gameDef.spritePath = NormalizeAssetPath(def.spritePath);
        gameDef.radius = def.radius;
        gameDef.moveSpeed = def.moveSpeed;
        gameDef.isStatic = def.isStatic;
        gameDef.massive = def.massive;
        gameDef.heroic = def.heroic;
        gameDef.spriteX = def.spriteX;
        gameDef.spriteY = def.spriteY;

        gameEntityManager.RegisterDefinition(gameDef);
    }

    return true;
}

void RegisterEntitySheets(SDL_Renderer* renderer) {
    for (const auto& [name, def] : g_entityDefs) {
        if (def.spritePath.empty()) {
            continue;
        }

        const bool isBuilding = def.isStatic;
        const int rows = isBuilding ? 1 : 5;
        const int cols = 5;

        gameEntityManager.RegisterSheetForEntity(
            entityManager,
            mapLoader,
            renderer,
            name,
            NormalizeAssetPath(def.spritePath),
            std::max(1, def.spriteX),
            std::max(1, def.spriteY),
            rows,
            cols
        );
    }
}
