#include "MapLoader.h"
#include <fstream>
#include <iostream>

SDL_Texture* MapLoader::LoadTexture(const std::string& file, SDL_Renderer* renderer) {
    SDL_Surface* surface = SDL_LoadBMP(file.c_str());
    if (!surface) {
        std::cerr << "Failed to load texture: " << file << " - " << SDL_GetError() << std::endl;
        return nullptr;
    }

    SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);
    SDL_DestroySurface(surface);

    if (!texture) {
        std::cerr << "Failed to create texture from: " << file << " - " << SDL_GetError() << std::endl;
    }

    return texture;
}

void MapLoader::LoadMapTile(const std::string& textureFile, int worldX, int worldY, SDL_Renderer* renderer) {
    SDL_Texture* texture = LoadTexture(textureFile, renderer);
    if (!texture) {
        std::cerr << "Failed to load texture: " << textureFile << std::endl;
        return;
    }
    
    MapTile newTile = { texture, worldX, worldY };
    mapTiles.push_back(newTile);
}

void MapLoader::Clear() {
    mapTiles.clear();
}

void MapLoader::RenderMap(SDL_Renderer* renderer, float cameraX, float cameraY) {
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);

    for (const auto& tile : mapTiles) {
        SDL_FRect dest = { tile.x - cameraX, tile.y - cameraY, 2000, 2000 };
        if (tile.texture) {
            SDL_RenderTexture(renderer, tile.texture, nullptr, &dest);
        }
    }
}
    