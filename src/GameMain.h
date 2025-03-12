#ifndef GAMEMAIN_H
#define GAMEMAIN_H

#include <SDL3/SDL.h>
#include "../engine/EntityManager.h"

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
void Spawn(EntityManager& entityManager, int unitType, int x, int y) ;
// Declare GameMain function
void GameMain(SDL_Window *window, SDL_Renderer *renderer);


#endif
