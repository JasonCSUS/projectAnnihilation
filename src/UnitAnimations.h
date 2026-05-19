#ifndef UNITANIMATIONS_H
#define UNITANIMATIONS_H

#include "../engine/AnimationManager.h"
#include <string>

namespace UnitAnimations {

// Group ids
inline const std::string GROUP_UNIT = "unit";
inline const std::string GROUP_BUILDING = "building";

// Animation ids
inline const std::string IDLE_UP = "idle_up";
inline const std::string IDLE_RIGHT = "idle_right";
inline const std::string IDLE_DOWN = "idle_down";

inline const std::string MOVE_UP = "move_up";
inline const std::string MOVE_RIGHT = "move_right";
inline const std::string MOVE_DOWN = "move_down";

inline const std::string ATTACK_UP = "attack_up";
inline const std::string ATTACK_RIGHT = "attack_right";
inline const std::string ATTACK_DOWN = "attack_down";

inline const std::string BUILDING_IDLE = "idle";
inline const std::string BUILDING_DEATH = "death";

// Registration
void RegisterDefaultAnimationGroups(AnimationManager& animationManager);

// Binding entities
void BindUnitEntity(AnimationManager& animationManager,
                    int entityId,
                    int sheetId);

void BindBuildingEntity(AnimationManager& animationManager,
                        int entityId,
                        int sheetId);

// External gameplay API
bool SetUnitAnimation(AnimationManager& animationManager,
                      int entityId,
                      const std::string& animId,
                      float frameDuration = 0.20f,
                      bool loop = true,
                      bool restart = false,
                      bool flipX = false);

bool SetBuildingAnimation(AnimationManager& animationManager,
                          int entityId,
                          const std::string& animId,
                          float frameDuration = 0.20f,
                          bool loop = true,
                          bool restart = false,
                          bool flipX = false);

} // namespace UnitAnimations

#endif