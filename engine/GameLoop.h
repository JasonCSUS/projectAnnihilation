#ifndef GAMELOOP_H
#define GAMELOOP_H

#include <SDL3/SDL.h>
#include "EntityManager.h"
#include "InputHandler.h"
#include "MapLoader.h"
#include "HUD.h"

typedef void (*UpdateFunc)(float);

extern MapLoader mapLoader;
extern SDL_Renderer* gRenderer;

void GameLoop(SDL_Window* window,
              SDL_Renderer* renderer,
              MapLoader& mapLoader,
              EntityManager& entityManager,
              InputHandler& inputHandler,
              UpdateFunc updateGame,
              HUD& hud);

#endif