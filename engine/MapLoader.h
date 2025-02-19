#ifndef MAPLOADER_H
#define MAPLOADER_H

#include <SDL3/SDL.h>
#include <vector>
#include <string>
#include "MapTile.h"  // Include the new MapTile header

SDL_Texture* LoadTexture(const std::string &file, SDL_Renderer* renderer);
void RenderMap(SDL_Renderer* renderer, const std::vector<MapTile>& tiles, float cameraX, float cameraY);
#endif
