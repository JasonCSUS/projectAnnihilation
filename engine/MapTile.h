#ifndef MAPTILE_H
#define MAPTILE_H

#include <SDL3/SDL.h>

// Struct representing a map tile with a texture and world position
struct MapTile {
    SDL_Texture* texture;
    int x, y;
};

#endif
