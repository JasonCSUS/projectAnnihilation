#include "AnimationManager.h"

#include <algorithm>
#include <cmath>

bool AnimationManager::CreateSheet(int sheetId,
                                   const std::string& name,
                                   SDL_Texture* texture,
                                   int cellWidth,
                                   int cellHeight,
                                   int rows,
                                   int cols) {
    if (sheetId < 0 || name.empty() || !texture ||
        cellWidth <= 0 || cellHeight <= 0 || rows <= 0 || cols <= 0) {
        return false;
    }

    sheets[sheetId] = AnimationSheet{
        sheetId,
        name,
        texture,
        cellWidth,
        cellHeight,
        rows,
        cols
    };
    return true;
}

bool AnimationManager::RemoveSheet(int sheetId) {
    return sheets.erase(sheetId) > 0;
}

const AnimationSheet* AnimationManager::TryGetSheet(int sheetId) const {
    auto it = sheets.find(sheetId);
    if (it == sheets.end()) {
        return nullptr;
    }
    return &it->second;
}

AnimationSheet* AnimationManager::TryGetSheet(int sheetId) {
    auto it = sheets.find(sheetId);
    if (it == sheets.end()) {
        return nullptr;
    }
    return &it->second;
}

bool AnimationManager::CreateGroup(const std::string& groupId) {
    if (groupId.empty()) {
        return false;
    }

    groups[groupId] = AnimationGroup{groupId, {}};
    return true;
}

bool AnimationManager::RemoveGroup(const std::string& groupId) {
    return groups.erase(groupId) > 0;
}

bool AnimationManager::AddAnimationToGroup(const std::string& groupId,
                                           const std::string& animId,
                                           const std::vector<Sprite>& frames) {
    auto it = groups.find(groupId);
    if (it == groups.end() || animId.empty() || frames.empty()) {
        return false;
    }

    it->second.animations[animId] = frames;
    return true;
}

const AnimationGroup* AnimationManager::TryGetGroup(const std::string& groupId) const {
    auto it = groups.find(groupId);
    if (it == groups.end()) {
        return nullptr;
    }
    return &it->second;
}

AnimationGroup* AnimationManager::TryGetGroup(const std::string& groupId) {
    auto it = groups.find(groupId);
    if (it == groups.end()) {
        return nullptr;
    }
    return &it->second;
}

void AnimationManager::CreateEntityAnim(int entityId, int sheetId, const std::string& groupId) {
    EntityAnim& entityAnim = entityAnims[entityId];
    entityAnim.entityId = entityId;
    entityAnim.sheetId = sheetId;
    entityAnim.groupId = groupId;
    entityAnim.currentAnimId.clear();
    entityAnim.currentFrame = 0;
    entityAnim.elapsedTime = 0.0f;
    entityAnim.frameDuration = 0.25f;
    entityAnim.loop = true;
    entityAnim.isPlaying = false;
    entityAnim.flipX = false;
    entityAnim.renderWidth = 0;
    entityAnim.renderHeight = 0;
}

void AnimationManager::RemoveEntityAnim(int entityId) {
    entityAnims.erase(entityId);
}

EntityAnim* AnimationManager::TryGetEntityAnim(int entityId) {
    auto it = entityAnims.find(entityId);
    if (it == entityAnims.end()) {
        return nullptr;
    }
    return &it->second;
}

const EntityAnim* AnimationManager::TryGetEntityAnim(int entityId) const {
    auto it = entityAnims.find(entityId);
    if (it == entityAnims.end()) {
        return nullptr;
    }
    return &it->second;
}

bool AnimationManager::SetEntitySheetAndGroup(int entityId, int sheetId, const std::string& groupId) {
    if (!TryGetSheet(sheetId) || !TryGetGroup(groupId)) {
        return false;
    }

    EntityAnim& entityAnim = entityAnims[entityId];
    entityAnim.entityId = entityId;
    entityAnim.sheetId = sheetId;
    entityAnim.groupId = groupId;
    return true;
}

bool AnimationManager::PlayAnimation(int entityId,
                                     const std::string& animId,
                                     float frameDuration,
                                     bool loopValue,
                                     bool restart,
                                     bool flipValue) {
    EntityAnim* entityAnim = TryGetEntityAnim(entityId);
    if (!entityAnim || animId.empty() || frameDuration <= 0.0f) {
        return false;
    }

    const AnimationGroup* group = TryGetGroup(entityAnim->groupId);
    if (!group) {
        return false;
    }

    auto animIt = group->animations.find(animId);
    if (animIt == group->animations.end() || animIt->second.empty()) {
        return false;
    }

    const bool sameAnim =
        entityAnim->isPlaying &&
        entityAnim->currentAnimId == animId &&
        std::abs(entityAnim->frameDuration - frameDuration) < 0.0001f &&
        entityAnim->loop == loopValue &&
        entityAnim->flipX == flipValue;

    if (sameAnim && !restart) {
        return true;
    }

    entityAnim->currentAnimId = animId;
    entityAnim->currentFrame = 0;
    entityAnim->elapsedTime = 0.0f;
    entityAnim->frameDuration = frameDuration;
    entityAnim->loop = loopValue;
    entityAnim->isPlaying = true;
    entityAnim->flipX = flipValue;
    return true;
}

void AnimationManager::SetFlipX(int entityId, bool flipX) {
    EntityAnim* entityAnim = TryGetEntityAnim(entityId);
    if (!entityAnim) {
        return;
    }
    entityAnim->flipX = flipX;
}

void AnimationManager::SetRenderSize(int entityId, int renderWidth, int renderHeight) {
    EntityAnim* entityAnim = TryGetEntityAnim(entityId);
    if (!entityAnim) {
        return;
    }

    entityAnim->renderWidth = std::max(1, renderWidth);
    entityAnim->renderHeight = std::max(1, renderHeight);
}

void AnimationManager::StopAnimation(int entityId) {
    EntityAnim* entityAnim = TryGetEntityAnim(entityId);
    if (!entityAnim) {
        return;
    }

    entityAnim->isPlaying = false;
    entityAnim->elapsedTime = 0.0f;
    entityAnim->currentFrame = 0;
}

const std::vector<Sprite>* AnimationManager::TryGetCurrentFrames(const EntityAnim& entityAnim) const {
    const AnimationGroup* group = TryGetGroup(entityAnim.groupId);
    if (!group) {
        return nullptr;
    }

    auto it = group->animations.find(entityAnim.currentAnimId);
    if (it == group->animations.end()) {
        return nullptr;
    }

    return &it->second;
}

Sprite AnimationManager::GetSafeFrame(const AnimationSheet& sheet,
                                      const std::vector<Sprite>& frames,
                                      int frameIndex) const {
    if (frames.empty()) {
        return Sprite{0, 0};
    }

    const int clampedIndex = std::clamp(frameIndex, 0, static_cast<int>(frames.size()) - 1);
    Sprite frame = frames[clampedIndex];

    if (frame.x < 0 || frame.x >= sheet.cols || frame.y < 0 || frame.y >= sheet.rows) {
        return Sprite{0, 0};
    }

    return frame;
}

void AnimationManager::UpdateAnimations(float deltaTime) {
    for (auto& [entityId, entityAnim] : entityAnims) {
        if (!entityAnim.isPlaying) {
            continue;
        }

        const std::vector<Sprite>* frames = TryGetCurrentFrames(entityAnim);
        if (!frames || frames->empty()) {
            continue;
        }

        entityAnim.elapsedTime += deltaTime;

        while (entityAnim.elapsedTime >= entityAnim.frameDuration) {
            entityAnim.elapsedTime -= entityAnim.frameDuration;
            entityAnim.currentFrame++;

            if (entityAnim.currentFrame >= static_cast<int>(frames->size())) {
                if (entityAnim.loop) {
                    entityAnim.currentFrame = 0;
                } else {
                    entityAnim.currentFrame = static_cast<int>(frames->size()) - 1;
                    entityAnim.isPlaying = false;
                    break;
                }
            }
        }
    }
}

bool AnimationManager::GetCurrentFrameSourceRect(int entityId, SDL_FRect& outSrc) const {
    const EntityAnim* entityAnim = TryGetEntityAnim(entityId);
    if (!entityAnim) {
        return false;
    }

    const AnimationSheet* sheet = TryGetSheet(entityAnim->sheetId);
    const std::vector<Sprite>* frames = TryGetCurrentFrames(*entityAnim);
    if (!sheet || !frames || frames->empty()) {
        return false;
    }

    const Sprite frame = GetSafeFrame(*sheet, *frames, entityAnim->currentFrame);

    outSrc = SDL_FRect{
        static_cast<float>(frame.x * sheet->cellWidth),
        static_cast<float>(frame.y * sheet->cellHeight),
        static_cast<float>(sheet->cellWidth),
        static_cast<float>(sheet->cellHeight)
    };
    return true;
}

bool AnimationManager::GetCurrentFrameDrawSize(int entityId, int& outW, int& outH) const {
    const EntityAnim* entityAnim = TryGetEntityAnim(entityId);
    if (!entityAnim) {
        return false;
    }

    if (entityAnim->renderWidth > 0 && entityAnim->renderHeight > 0) {
        outW = entityAnim->renderWidth;
        outH = entityAnim->renderHeight;
        return true;
    }

    const AnimationSheet* sheet = TryGetSheet(entityAnim->sheetId);
    if (!sheet) {
        return false;
    }

    outW = sheet->cellWidth;
    outH = sheet->cellHeight;
    return true;
}

void AnimationManager::RenderEntity(SDL_Renderer* renderer,
                                    int entityId,
                                    int x,
                                    int y) const {
    if (!renderer) {
        return;
    }

    const EntityAnim* entityAnim = TryGetEntityAnim(entityId);
    if (!entityAnim) {
        return;
    }

    const AnimationSheet* sheet = TryGetSheet(entityAnim->sheetId);
    if (!sheet || !sheet->texture) {
        return;
    }

    SDL_FRect src{};
    if (!GetCurrentFrameSourceRect(entityId, src)) {
        return;
    }

    int spriteW = 0;
    int spriteH = 0;
    if (!GetCurrentFrameDrawSize(entityId, spriteW, spriteH)) {
        return;
    }

    SDL_FRect dst = {
        static_cast<float>(x - spriteW / 2),
        static_cast<float>(y - spriteH / 2),
        static_cast<float>(spriteW),
        static_cast<float>(spriteH)
    };

    SDL_RenderTextureRotated(
        renderer,
        sheet->texture,
        &src,
        &dst,
        0.0,
        nullptr,
        entityAnim->flipX ? SDL_FLIP_HORIZONTAL : SDL_FLIP_NONE
    );
}