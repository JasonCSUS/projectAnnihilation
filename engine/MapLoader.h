#ifndef MAPLOADER_H
#define MAPLOADER_H

#include <string>
#include <SDL3/SDL.h>

class MapLoader {
public:
    // Loads a general texture from file.
    SDL_Texture* LoadTexture(const std::string& file, SDL_Renderer* renderer);

    // Loads the single map texture used by the level.
    bool LoadMap(const std::string& textureFile, SDL_Renderer* renderer);

    // Renders the single loaded map.
    void RenderMap(SDL_Renderer* renderer, float cameraX, float cameraY);

    // Clears the loaded map texture reference/state.
    void Clear();

    int GetMapWidth() const { return mapWidth; }
    int GetMapHeight() const { return mapHeight; }

private:
    SDL_Texture* mapTexture = nullptr;
    int mapWidth = 0;
    int mapHeight = 0;
};

#endif // MAPLOADER_H