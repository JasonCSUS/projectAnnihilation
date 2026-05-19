#include "UnitAnimations.h"

namespace UnitAnimations {
namespace {

std::vector<Sprite> RepeatFrame(const Sprite& frame, int count) {
    std::vector<Sprite> result;
    result.reserve(count);
    for (int i = 0; i < count; ++i) {
        result.push_back(frame);
    }
    return result;
}

} // namespace

void RegisterDefaultAnimationGroups(AnimationManager& animationManager) {
    animationManager.CreateGroup(GROUP_UNIT);
    animationManager.CreateGroup(GROUP_BUILDING);

    // Unit layout based on your current unit1/unit2 sheet structure.
    animationManager.AddAnimationToGroup(GROUP_UNIT, IDLE_UP, RepeatFrame({0, 4}, 4));
    animationManager.AddAnimationToGroup(GROUP_UNIT, IDLE_RIGHT, RepeatFrame({2, 4}, 4));
    animationManager.AddAnimationToGroup(GROUP_UNIT, IDLE_DOWN, RepeatFrame({4, 4}, 4));

    animationManager.AddAnimationToGroup(GROUP_UNIT, MOVE_UP, {
        {0, 3}, {0, 2}, {0, 1}, {0, 0}
    });
    animationManager.AddAnimationToGroup(GROUP_UNIT, MOVE_RIGHT, {
        {2, 3}, {2, 2}, {2, 1}, {2, 0}
    });
    animationManager.AddAnimationToGroup(GROUP_UNIT, MOVE_DOWN, {
        {4, 3}, {4, 2}, {4, 1}, {4, 0}
    });

    // Placeholder attack clips for now.
    animationManager.AddAnimationToGroup(GROUP_UNIT, ATTACK_UP, RepeatFrame({0, 4}, 4));
    animationManager.AddAnimationToGroup(GROUP_UNIT, ATTACK_RIGHT, RepeatFrame({2, 4}, 4));
    animationManager.AddAnimationToGroup(GROUP_UNIT, ATTACK_DOWN, RepeatFrame({4, 4}, 4));

    // Building layout: idle at 0,0 and death along row 0 from cells 1-4.
    animationManager.AddAnimationToGroup(GROUP_BUILDING, BUILDING_IDLE, {
        {0, 0}
    });

    animationManager.AddAnimationToGroup(GROUP_BUILDING, BUILDING_DEATH, {
        {1, 0}, {2, 0}, {3, 0}, {4, 0}
    });
}

void BindUnitEntity(AnimationManager& animationManager,
                    int entityId,
                    int sheetId) {
    animationManager.CreateEntityAnim(entityId, sheetId, GROUP_UNIT);
    animationManager.PlayAnimation(entityId, IDLE_DOWN, 0.25f, true, true);
}

void BindBuildingEntity(AnimationManager& animationManager,
                        int entityId,
                        int sheetId) {
    animationManager.CreateEntityAnim(entityId, sheetId, GROUP_BUILDING);
    animationManager.PlayAnimation(entityId, BUILDING_IDLE, 1.0f, true, true);
}

bool SetUnitAnimation(AnimationManager& animationManager,
                      int entityId,
                      const std::string& animId,
                      float frameDuration,
                      bool loop,
                      bool restart,
                      bool flipX) {
    return animationManager.PlayAnimation(entityId, animId, frameDuration, loop, restart, flipX);
}

bool SetBuildingAnimation(AnimationManager& animationManager,
                          int entityId,
                          const std::string& animId,
                          float frameDuration,
                          bool loop,
                          bool restart,
                          bool flipX) {
    return animationManager.PlayAnimation(entityId, animId, frameDuration, loop, restart, flipX);
}

} // namespace UnitAnimations