#ifndef GAMELOOP_H
#define GAMELOOP_H

#include <SDL3/SDL.h>
#include <string>
#include "../engine/EntityManager.h"

class EntityManager;
typedef void (*UpdateFunc)(float);

void GameLoop(SDL_Window* window, SDL_Renderer* renderer, EntityManager& entityManager, AnimationManager& animationManager, UpdateFunc updateGame);

void AddMapTile(const std::string& file, int x, int y, SDL_Renderer* renderer);
#endif
