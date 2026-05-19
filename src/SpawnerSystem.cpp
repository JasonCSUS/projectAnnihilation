#include "SpawnerSystem.h"

#include "Difficulty.h"
#include "Metadata.h"
#include "RuntimeData.h"
#include "SpawnFactory.h"

#include <algorithm>
#include <cmath>
#include <regex>
#include <string>
#include <vector>

namespace {
constexpr int SPAWNS_PER_TICK = 5;

const std::vector<std::string> g_baseMonsterStack = {
    "blightborn",
    "skitterfang",
    "bilemaw",
    "quillfang",
    "rendbeast",
    "spinebeast",
    "dreadmaw",
    "dreadseer"
};

const std::vector<std::string> g_hatcheryMonsterStack = {
    "blight_tyrant",
    "skitter_tyrant",
    "bile_host",
    "quill_host",
    "rend_bloomer",
    "spine_bloomer",
    "dread_bloomer"
};

const std::vector<std::string> g_baseSpawnerBuildingStack = {
    "blightborn_spawner",
    "skitterfang_spawner",
    "bilemaw_spawner",
    "quillfang_spawner",
    "rendbeast_spawner"
};

const std::vector<std::string> g_hatcherySpawnerBuildingStack = {
    "baroness_spawner",
    "duchess_spawner",
    "queen_spawner",
    "empress_spawner"
};

bool TryParseSpawnerPoint(const std::string& label, SpawnerFamily& outFamily, int& outSlotIndex) {
    static const std::regex re("^(base_spawner|hatchery_spawner)_(\\d+)$");
    std::smatch match;
    if (!std::regex_match(label, match, re)) {
        return false;
    }

    const std::string familyText = match[1].str();
    const int oneBasedIndex = std::stoi(match[2].str());
    outSlotIndex = std::max(0, oneBasedIndex - 1);

    if (familyText == "base_spawner") {
        outFamily = SpawnerFamily::Base;
    } else {
        outFamily = SpawnerFamily::Hatchery;
    }

    return true;
}

const std::vector<std::string>& ResolveProducedFamilyList(SpawnerFamily family) {
    return (family == SpawnerFamily::Hatchery) ? g_hatcheryMonsterStack : g_baseMonsterStack;
}

const std::vector<std::string>& ResolveSpawnerBuildingStack(SpawnerFamily family) {
    return (family == SpawnerFamily::Hatchery)
        ? g_hatcherySpawnerBuildingStack
        : g_baseSpawnerBuildingStack;
}

int ResolveFamilyTier(SpawnerFamily /*family*/) {
    return GetCurrentDifficultyProfile().tierUp;
}

void GetBurstSpawnOffset(int burstIndex, float& outX, float& outY) {
    constexpr float radius = 85.0f;
    constexpr float step = 72.0f;
    const float angleDeg = -90.0f + (burstIndex % 5) * step;
    const float angleRad = angleDeg * 3.14159265358979323846f / 180.0f;

    outX = std::cos(angleRad) * radius;
    outY = std::sin(angleRad) * radius;
}
} // namespace

void RefreshActiveSpawnersForRoom(const std::string& roomLabel) {
    g_activeSpawners.clear();

    for (const auto& p : g_points) {
        SpawnerFamily family;
        int slotIndex = 0;
        if (!TryParseSpawnerPoint(p.label, family, slotIndex)) {
            continue;
        }

        if (!IsPointInsideTrigger(roomLabel, p.x, p.y)) {
            continue;
        }

        const auto& buildingStack = ResolveSpawnerBuildingStack(family);
        if (buildingStack.empty()) {
            continue;
        }

        const int buildingIndex = std::clamp(slotIndex, 0, static_cast<int>(buildingStack.size()) - 1);
        const std::string& buildingEntityName = buildingStack[buildingIndex];

        const int spawnedBuildingId = SpawnNamedBuilding(
            buildingEntityName,
            static_cast<int>(std::lround(p.x)),
            static_cast<int>(std::lround(p.y))
        );

        if (spawnedBuildingId < 0) {
            continue;
        }

        RuntimeSpawner spawner;
        spawner.label = p.label;
        spawner.buildingEntityName = buildingEntityName;
        spawner.family = family;
        spawner.slotIndex = slotIndex;
        spawner.x = p.x;
        spawner.y = p.y;
        spawner.spawnedBuildingEntityId = spawnedBuildingId;
        spawner.active = true;
        g_activeSpawners.push_back(spawner);
    }
}

void UpdateSpawners(float deltaTime) {
    g_spawnerTimer += deltaTime;
    if (g_spawnerTimer < 15.0f) {
        return;
    }
    g_spawnerTimer = 0.0f;

    for (RuntimeSpawner& spawner : g_activeSpawners) {
        if (!spawner.active) {
            continue;
        }

        if (!IsEntityAliveById(spawner.spawnedBuildingEntityId)) {
            spawner.active = false;
            continue;
        }

        const auto& familyList = ResolveProducedFamilyList(spawner.family);
        if (familyList.empty()) {
            continue;
        }

        const int tier = ResolveFamilyTier(spawner.family);
        const int producedIndex = std::clamp(
            spawner.slotIndex + tier,
            0,
            static_cast<int>(familyList.size()) - 1
        );

        const std::string& producedEntityName = familyList[producedIndex];

        for (int burstIndex = 0; burstIndex < SPAWNS_PER_TICK; ++burstIndex) {
            float offsetX = 0.0f;
            float offsetY = 0.0f;
            GetBurstSpawnOffset(burstIndex, offsetX, offsetY);

            SpawnNamedUnit(
                producedEntityName,
                static_cast<int>(std::lround(spawner.x + offsetX)),
                static_cast<int>(std::lround(spawner.y + offsetY))
            );
        }
    }
}
