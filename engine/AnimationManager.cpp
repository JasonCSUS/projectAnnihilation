#include "AnimationManager.h"
#include <iostream>

void AnimationManager::AddAnimation(int entityId, Animation* animation) {
    animations[entityId] = animation;
}

void AnimationManager::UpdateAnimations(float deltaTime) {
    for (auto& pair : animations) {
        pair.second->Update(deltaTime);
    }
}

Animation* AnimationManager::GetAnimation(int entityId) {
    if (animations.find(entityId) != animations.end()) {
        return animations[entityId];
    }
    return nullptr;
}

void AnimationManager::SwapSprite(int entityId, Sprite* newSprite) {
    if (animations.find(entityId) != animations.end()) {
        int oldW = animations[entityId]->spriteW;
        int oldH = animations[entityId]->spriteH;
        animations[entityId]->frames = newSprite;
        animations[entityId]->currentFrame = 0;
        animations[entityId]->spriteW = oldW;
        animations[entityId]->spriteH = oldH;
    }
}

void AnimationManager::RenderEntity(SDL_Renderer* renderer, SDL_Texture* spriteSheet, Animation* animation, int x, int y, bool flip) {
    if (!animation) {
        return;
    }
    if (!spriteSheet) {
        return;
    }

    Sprite& currentFrame = animation->GetCurrentFrame();
    


    SDL_FRect src = {
        static_cast<float>(currentFrame.x * animation->spriteW), 
        static_cast<float>(currentFrame.y * animation->spriteH), 
        static_cast<float>(animation->spriteW), 
        static_cast<float>(animation->spriteH)
    };

    SDL_FRect dst = {
        static_cast<float>(x), 
        static_cast<float>(y), 
        static_cast<float>(animation->spriteW), 
        static_cast<float>(animation->spriteH)
    };

    SDL_RenderTextureRotated(renderer, spriteSheet, &src, &dst, 0.0, NULL, flip ? SDL_FLIP_HORIZONTAL : SDL_FLIP_NONE);
}
