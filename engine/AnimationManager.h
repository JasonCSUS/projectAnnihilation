#ifndef ANIMATIONMANAGER_H
#define ANIMATIONMANAGER_H

#include "Character.h"
#include <SDL3/SDL.h>

#include <string>
#include <unordered_map>
#include <vector>

struct AnimationSheet {
    int sheetId = -1;
    std::string name;
    SDL_Texture* texture = nullptr;
    int cellWidth = 0;
    int cellHeight = 0;
    int rows = 0;
    int cols = 0;
};

struct AnimationGroup {
    std::string groupId;
    std::unordered_map<std::string, std::vector<Sprite>> animations;
};

struct EntityAnim {
    int entityId = -1;
    int sheetId = -1;
    std::string groupId;
    std::string currentAnimId;

    int currentFrame = 0;
    float elapsedTime = 0.0f;
    float frameDuration = 0.25f;
    bool loop = true;
    bool isPlaying = false;

    bool flipX = false;

    int renderWidth = 0;
    int renderHeight = 0;
};

class AnimationManager {
public:
    bool CreateSheet(int sheetId,
                     const std::string& name,
                     SDL_Texture* texture,
                     int cellWidth,
                     int cellHeight,
                     int rows,
                     int cols);

    bool RemoveSheet(int sheetId);

    const AnimationSheet* TryGetSheet(int sheetId) const;
    AnimationSheet* TryGetSheet(int sheetId);

    bool CreateGroup(const std::string& groupId);
    bool RemoveGroup(const std::string& groupId);

    bool AddAnimationToGroup(const std::string& groupId,
                             const std::string& animId,
                             const std::vector<Sprite>& frames);

    const AnimationGroup* TryGetGroup(const std::string& groupId) const;
    AnimationGroup* TryGetGroup(const std::string& groupId);

    void CreateEntityAnim(int entityId, int sheetId, const std::string& groupId);
    void RemoveEntityAnim(int entityId);

    EntityAnim* TryGetEntityAnim(int entityId);
    const EntityAnim* TryGetEntityAnim(int entityId) const;

    bool SetEntitySheetAndGroup(int entityId, int sheetId, const std::string& groupId);

    bool PlayAnimation(int entityId,
                       const std::string& animId,
                       float frameDuration,
                       bool loop,
                       bool restart = false,
                       bool flipX = false);

    void SetFlipX(int entityId, bool flipX);
    void SetRenderSize(int entityId, int renderWidth, int renderHeight);

    void StopAnimation(int entityId);
    void UpdateAnimations(float deltaTime);

    bool GetCurrentFrameSourceRect(int entityId, SDL_FRect& outSrc) const;
    bool GetCurrentFrameDrawSize(int entityId, int& outW, int& outH) const;

    void RenderEntity(SDL_Renderer* renderer,
                      int entityId,
                      int x,
                      int y) const;

private:
    const std::vector<Sprite>* TryGetCurrentFrames(const EntityAnim& entityAnim) const;
    Sprite GetSafeFrame(const AnimationSheet& sheet,
                        const std::vector<Sprite>& frames,
                        int frameIndex) const;

private:
    std::unordered_map<int, AnimationSheet> sheets;
    std::unordered_map<std::string, AnimationGroup> groups;
    std::unordered_map<int, EntityAnim> entityAnims;
};

#endif