#ifndef ANIMATIONMANAGER_H
#define ANIMATIONMANAGER_H

#include "Character.h"
#include <vector>
#include <map>
#include <string>
#include <SDL3/SDL.h>

struct Animation {
    Sprite* frames;
    int frameCount;
    float frameTime;
    float elapsedTime = 0;
    int currentFrame = 0;
    int spriteW, spriteH; // Track width and height here

    void Update(float deltaTime) {
        elapsedTime += deltaTime;
        if (elapsedTime >= frameTime) {
            elapsedTime = 0;
            currentFrame = (currentFrame + 1) % frameCount;
        }
    }

    Sprite& GetCurrentFrame() { return frames[currentFrame]; }
};

class AnimationManager {
public:
    void AddAnimation(int entityId, Animation* animation);
    void UpdateAnimations(float deltaTime);
    Animation* GetAnimation(int entityId);
    void SwapSprite(int entityId, Sprite* newSprite);
    void RenderEntity(SDL_Renderer* renderer, SDL_Texture* spriteSheet, Animation* animation, int x, int y, bool flip);

private:
    std::map<int, Animation*> animations;
};

#endif
