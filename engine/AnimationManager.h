#ifndef ANIMATIONMANAGER_H
#define ANIMATIONMANAGER_H

#include "Character.h"
#include <vector>
#include <map>
#include <string>
#include <SDL3/SDL.h>

enum AnimState { IDLE, MOVING, ATTACKING };

struct Animation {
    std::vector<Sprite> frames;
    float frameTime;
    int spriteW, spriteH; // Track width and height here
    float elapsedTime;
    int currentFrame;
    AnimState lastState = IDLE;
    bool changeState = false;
    Sprite& GetCurrentFrame() { return frames[currentFrame]; }
};

class AnimationManager {
public:
    void AddAnimation(int entityId, Animation animation);
    Animation& GetAnimation(int entityId);
    void SwapSprite(int entityId, std::vector<Sprite> newSprite);
    void RenderEntity(SDL_Renderer* renderer, SDL_Texture* spriteSheet, Animation animation, int x, int y, bool flip);
    void AnimationManager::SetChangeState(int entityId, bool state);
    void RemoveAnimation(int entityId);
    void UpdateAnimations(float deltaTime);
private:
    std::map<int, Animation> animations;
};

#endif
