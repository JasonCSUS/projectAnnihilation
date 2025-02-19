#ifndef MAPLOADER_H
#define MAPLOADER_H
#include <SDL3/SDL.h>
#include <string>

SDL_Texture* LoadTexture(const std::string &file, SDL_Renderer* renderer);
void RenderMap(SDL_Renderer* renderer, SDL_Texture* map1, SDL_Texture* map2, float cameraX, float cameraY);

#endif