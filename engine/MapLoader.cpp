#include "MapLoader.h"
#include <iostream>

SDL_Texture* MapLoader::LoadTexture(const std::string& file, SDL_Renderer* renderer) {
    SDL_Surface* surface = SDL_LoadBMP(file.c_str());
    if (!surface) {
        std::cerr << "Failed to load texture: " << file
                  << " - " << SDL_GetError() << std::endl;
        return nullptr;
    }

    SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);
    SDL_DestroySurface(surface);

    if (!texture) {
        std::cerr << "Failed to create texture from: " << file
                  << " - " << SDL_GetError() << std::endl;
    }

    return texture;
}

bool MapLoader::LoadMap(const std::string& textureFile, SDL_Renderer* renderer) {
    Clear();

    SDL_Surface* surface = SDL_LoadBMP(textureFile.c_str());
    if (!surface) {
        std::cerr << "Failed to load map texture: " << textureFile
                  << " - " << SDL_GetError() << std::endl;
        return false;
    }

    mapWidth = surface->w;
    mapHeight = surface->h;

    mapTexture = SDL_CreateTextureFromSurface(renderer, surface);
    SDL_DestroySurface(surface);

    if (!mapTexture) {
        std::cerr << "Failed to create map texture from: " << textureFile
                  << " - " << SDL_GetError() << std::endl;
        mapWidth = 0;
        mapHeight = 0;
        return false;
    }

    return true;
}

void MapLoader::Clear() {
    mapTexture = nullptr;
    mapWidth = 0;
    mapHeight = 0;
}

void MapLoader::RenderMap(SDL_Renderer* renderer, float cameraX, float cameraY) {
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);

    if (!mapTexture) {
        return;
    }

    SDL_FRect dest = {
        -cameraX,
        -cameraY,
        static_cast<float>(mapWidth),
        static_cast<float>(mapHeight)
    };

    SDL_RenderTexture(renderer, mapTexture, nullptr, &dest);
}