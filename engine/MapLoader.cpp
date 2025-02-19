#include "MapLoader.h"
#include <SDL3_image/SDL_image.h>
#include <iostream>

// List to store map tiles

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

void RenderMap(SDL_Renderer* renderer, const std::vector<MapTile>& mapTiles, float cameraX, float cameraY) {
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);

    for (const auto& tile : mapTiles) {
        SDL_FRect dest = { tile.x - cameraX, tile.y - cameraY, 2000, 2000 };
        if (tile.texture) {
            SDL_RenderTexture(renderer, tile.texture, nullptr, &dest);
        }
    }
}
