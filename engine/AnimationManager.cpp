#include "AnimationManager.h"
#include <iostream>

void AnimationManager::AddAnimation(int entityId, Animation animation) {
    animations[entityId] = animation;
}
void AnimationManager::RemoveAnimation(int entityId) {
    animations.erase(entityId);
}

void AnimationManager::UpdateAnimations(float deltaTime) {
    for (auto& pair : animations) {
        Animation& anim = pair.second;
        anim.elapsedTime += deltaTime;
        if (!anim.frames.empty() && anim.elapsedTime >= anim.frameTime) {
            anim.elapsedTime -= anim.frameTime;
            anim.currentFrame = (anim.currentFrame + 1) % anim.frames.size();
        }
    }
}

Animation& AnimationManager::GetAnimation(int entityId) {
    if (animations.find(entityId) == animations.end()) {
        // Handle error appropriately; for example, throw or add a new Animation.
        throw std::runtime_error("Animation not found for entity id");
    }
    return animations[entityId];
}

void AnimationManager::SwapSprite(int entityId, std::vector<Sprite> newSprite) {
    if (animations.find(entityId) != animations.end()) {
        animations[entityId].frames = newSprite;
        if(animations[entityId].changeState)
        {
            animations[entityId].currentFrame=0;
            animations[entityId].changeState=false;
        }
    }
}
void AnimationManager::SetChangeState(int entityId, bool state)
{
    if(animations.find(entityId) != animations.end()){
        animations[entityId].changeState=state;
    }
}

void AnimationManager::RenderEntity(SDL_Renderer* renderer, SDL_Texture* spriteSheet, Animation animation, int x, int y, bool flip) {
    if (animation.frames.empty()) {
        return;
    }
    if (!spriteSheet) {
        return;
    }

    Sprite& currentFrame = animation.GetCurrentFrame();
    


    SDL_FRect src = {
        static_cast<float>(currentFrame.x * animation.spriteW), 
        static_cast<float>(currentFrame.y * animation.spriteH), 
        static_cast<float>(animation.spriteW), 
        static_cast<float>(animation.spriteH)
    };

    SDL_FRect dst = {
        static_cast<float>(x-animation.spriteW/2), 
        static_cast<float>(y-animation.spriteH/2), 
        static_cast<float>(animation.spriteW), 
        static_cast<float>(animation.spriteH)
    };

    SDL_RenderTextureRotated(renderer, spriteSheet, &src, &dst, 0.0, NULL, flip ? SDL_FLIP_HORIZONTAL : SDL_FLIP_NONE);
}
