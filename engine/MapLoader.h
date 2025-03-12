#ifndef MAPLOADER_H
#define MAPLOADER_H

#include <vector>
#include <string>
#include "MapTile.h"
#include "NavMesh.h"
#include <SDL3/SDL.h>

class MapLoader {
public:

    // Loads a map tile from a texture file and places it at the specified world coordinates.
    void LoadMapTile(const std::string& textureFile, int worldX, int worldY, SDL_Renderer* renderer);

    // Renders all loaded map tiles.
    void RenderMap(SDL_Renderer* renderer, float cameraX, float cameraY);

    // Clears all loaded map tiles.
    void Clear();

    // Loads a texture from file.
    SDL_Texture* LoadTexture(const std::string& file, SDL_Renderer* renderer);

private:
    std::vector<MapTile> mapTiles;
};

#endif // MAPLOADER_H
