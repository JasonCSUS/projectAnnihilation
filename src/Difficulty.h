#ifndef DIFFICULTY_H
#define DIFFICULTY_H

#include <string>

enum class DifficultyLevel {
    VeryEasy = 0,
    Easy,
    Normal,
    Hard,
    VeryHard,
    Insane,
    Colossal,
    Nightmare,
    Torment,
    Hell,
    Titanic,
    Mythic,
    Divine,
    Impossible,
    Cataclysm,
    Oblivion,
    Ruination,
    Annihilation
};

struct DifficultyProfile {
    std::string name;

    float enemyHealthMultiplier = 1.0f;
    float enemyArmorMultiplier = 1.0f;
    float enemyDamageMultiplier = 1.0f;

    float enemyDamageIncrease = 0.0f;
    float enemyDamageReduction = 0.0f;
    float enemySpeedMultiplier = 1.0f;
    float enemyAttackSpeedMultiplier = 1.0f;

    bool colossusEnabled = false;
    float fear = 0.0f;
    float statReduction = 0.0f;
    int startingUpgrades = 0;
    float titanicChance = 0.0f;
    float bossBoost = 0.0f;
    float critDamageReduction = 0.0f;
    int tierUp = 0;
    float upgradeCostIncrease = 0.0f;
    float damageIncreaseReduction = 0.0f;
};

const DifficultyProfile& GetCurrentDifficultyProfile();
DifficultyLevel GetCurrentDifficultyLevel();

#endif
