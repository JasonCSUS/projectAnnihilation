#ifndef GAMEMAIN_H
#define GAMEMAIN_H

#include <SDL3/SDL.h>

#include "Difficulty.h"
#include "Metadata.h"

void UpdateGame(float deltaTime);
void GameMain(SDL_Window* window, SDL_Renderer* renderer);

#endif
