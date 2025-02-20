#include "MovementSystem.h"
#include "EntityManager.h"
#include "AnimationManager.h"
#include <SDL3/SDL.h>
#include <iostream>

extern std::vector<std::vector<bool>> collisionMap;

bool IsWalkable(float x, float y);

void MovementSystem::Update(std::vector<Entity>& entities, AnimationManager& animationManager, float deltaTime) {
    for (auto& entity : entities) {
        float newX = entity.position.x;
        float newY = entity.position.y;
        Sprite* newSprite = nullptr;
        Direction lastDirection = entity.lastDirection;

        if (up) { newY -= speed * deltaTime; newSprite = walkUp; lastDirection = UP; }
        if (down) { newY += speed * deltaTime; newSprite = walkDown; lastDirection = DOWN; }
        if (left) { newX -= speed * deltaTime; newSprite = walkRight; lastDirection = LEFT; }
        if (right) { newX += speed * deltaTime; newSprite = walkRight; lastDirection = RIGHT; }

        if (IsWalkable(newX, newY)) {
            entity.position.x = newX;
            entity.position.y = newY;
            entity.lastDirection = lastDirection;
            if (newSprite) {
                animationManager.SwapSprite(entity.id, newSprite);
            }
        }
        else{
          up = false;
          down = false;
          left = false;
          right = false;
        }
      
            // If no movement, swap to idle based on last direction
        if (!up && !down && !left && !right) {
             switch (entity.lastDirection) {
                case UP: animationManager.SwapSprite(entity.id, idleUp); break;
                case DOWN: animationManager.SwapSprite(entity.id, idleDown); break;
                case LEFT: animationManager.SwapSprite(entity.id, idleRight); break;
                case RIGHT: animationManager.SwapSprite(entity.id, idleRight); break;
                default: break;
            }
        }
    }
}

void MovementSystem::HandleInput(const SDL_Event& event) {
    if (event.type == SDL_EVENT_KEY_DOWN) {
        if (event.key.key == SDLK_W) up = true;
        if (event.key.key == SDLK_S) down = true;
        if (event.key.key == SDLK_A) left = true;
        if (event.key.key == SDLK_D) right = true;
    }
    if (event.type == SDL_EVENT_KEY_UP) {
        if (event.key.key == SDLK_W) up = false;
        if (event.key.key == SDLK_S) down = false;
        if (event.key.key == SDLK_A) left = false;
        if (event.key.key == SDLK_D) right = false;
    }
}

std::vector<std::vector<bool>> LoadCollisionMap(const std::string& file) {
    SDL_Surface* surface = SDL_LoadBMP(file.c_str());
    if (!surface) {
        std::cerr << "Failed to load collision map: " << SDL_GetError() << std::endl;
        return {};
    }

    std::vector<std::vector<bool>> collisionMap(surface->h, std::vector<bool>(surface->w, false));

    Uint8* pixels = static_cast<Uint8*>(surface->pixels);
    
    for (int y = 0; y < surface->h; y++) {
        for (int x = 0; x < surface->w; x++) {
            Uint32 pixelColor = *(Uint32*)(pixels + y * surface->pitch + x * 4);
            Uint8 r, g, b;
            const SDL_PixelFormatDetails* fmtDetails = SDL_GetPixelFormatDetails(surface->format);
            if (!fmtDetails) {
                std::cerr << "Failed to get pixel format details." << std::endl;
                SDL_DestroySurface(surface);
                return {};
            }
            Uint32 maskR = fmtDetails->Rmask;
            Uint32 maskG = fmtDetails->Gmask;
            Uint32 maskB = fmtDetails->Bmask;
            r = (pixelColor & maskR) >> fmtDetails->Rshift;
            g = (pixelColor & maskG) >> fmtDetails->Gshift;
            b = (pixelColor & maskB) >> fmtDetails->Bshift;

            collisionMap[y][x] = (r == 0 && g == 0 && b == 0); // True if black (wall)
        }
    }

    SDL_DestroySurface(surface);
    return collisionMap;
}

bool IsWalkable(float x, float y) {
    if (collisionMap.empty()) return true; // Ensure collisionMap is not empty to avoid crashes
    
    int mapX = static_cast<int>(x);
    int mapY = static_cast<int>(y);
    
    if (mapX < 0 || mapY < 0 || mapY >= collisionMap.size() || mapX >= collisionMap[0].size()) {
        return false; // Out of bounds, treat as non-walkable
    }
    
    return !collisionMap[mapY][mapX];
}
