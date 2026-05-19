#ifndef MAPLOADER_H
#define MAPLOADER_H

#include <string>
#include <SDL3/SDL.h>

class MapLoader {
public:
    SDL_Texture* LoadTexture(const std::string& file, SDL_Renderer* renderer);
    bool LoadMap(const std::string& textureFile, SDL_Renderer* renderer);
    void RenderMap(SDL_Renderer* renderer, float cameraX, float cameraY);
    void Clear();

    int GetMapWidth() const { return mapWidth; }
    int GetMapHeight() const { return mapHeight; }
    SDL_Texture* GetMapTexture() const { return mapTexture; }

private:
    SDL_Texture* mapTexture = nullptr;
    int mapWidth = 0;
    int mapHeight = 0;
};

#endif // MAPLOADER_H