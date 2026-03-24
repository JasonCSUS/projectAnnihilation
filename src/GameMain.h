#ifndef GAMEMAIN_H
#define GAMEMAIN_H

#include <SDL3/SDL.h>
#include "../engine/EntityManager.h"
#include <string>

enum UnitType
{
    UNIT1=1,
    UNIT2=2
};

enum controller
{
    PLAYER=1,
    ENEMY=2
};

void UpdateGame(float deltaTime);
void Spawn(EntityManager& entityManager, int unitType, int x, int y);
void GameMain(SDL_Window *window, SDL_Renderer *renderer);

// Metadata helpers loaded from navmesh.json
bool GetPointPosition(const std::string& label, int& outX, int& outY);
bool GetObjectCenter(const std::string& label, int& outX, int& outY);
bool GetTriggerCenter(const std::string& label, int& outX, int& outY);
bool IsPointInsideTrigger(const std::string& label, float x, float y);

// Stress-test stats
int GetAliveEnemyCount();
int GetFrozenEnemyCount();
float GetCurrentFPS();
bool IsEnemySpawningPaused();

#endif