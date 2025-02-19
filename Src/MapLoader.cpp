#include "MapLoader.h"
#include <SDL3_image/SDL_image.h>
#include <iostream>

SDL_Texture* LoadTexture(const std::string &file, SDL_Renderer* renderer) {
    SDL_Surface* surface = SDL_LoadBMP(file.c_str());
    if (!surface) {
        std::cerr << "Failed to load image: " << file << " - SDL Error: " << SDL_GetError() << std::endl;
        return nullptr;
    }
    SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);
    SDL_DestroySurface(surface);
    return texture;
}

void RenderMap(SDL_Renderer* renderer, SDL_Texture* map1, SDL_Texture* map2, float cameraX, float cameraY) {
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);

    SDL_FRect dest1 = { -cameraX, -cameraY, 2000, 2000 };
    SDL_FRect dest2 = { 2000 - cameraX, -cameraY, 2000, 2000 };

    if (map1) SDL_RenderTexture(renderer, map1, nullptr, &dest1);
    if (map2) SDL_RenderTexture(renderer, map2, nullptr, &dest2);

}