#ifndef GAMELOOP_H
#define GAMELOOP_H

#include <SDL3/SDL.h>
#include <string>
#include "EntityManager.h"
#include "InputHandler.h"
#include "MapLoader.h"

class EntityManager;
typedef void (*UpdateFunc)(float);
extern MapLoader mapLoader;

void GameLoop(SDL_Window* window, SDL_Renderer* renderer, MapLoader& mapLoader, EntityManager& entityManager, InputHandler& inputHandler, UpdateFunc updateGame);

void AddMapTile(const std::string& file, const std::string& collisionFile, int x, int y, SDL_Renderer* renderer);
#endif
