#include "Difficulty.h"

namespace {
DifficultyLevel g_currentDifficultyLevel = DifficultyLevel::VeryEasy;
DifficultyProfile g_currentDifficultyProfile = {
    "Very Easy",
    0.66f,
    0.66f,
    0.66f,
    0.0f,
    0.0f,
    1.0f,
    1.0f,
    false,
    0.0f,
    0.0f,
    0,
    0.0f,
    0.0f,
    0.0f,
    0,
    0.0f,
    0.0f
};
} // namespace

const DifficultyProfile& GetCurrentDifficultyProfile() {
    return g_currentDifficultyProfile;
}

DifficultyLevel GetCurrentDifficultyLevel() {
    return g_currentDifficultyLevel;
}
