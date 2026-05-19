#ifndef UNITVISUALS_H
#define UNITVISUALS_H

#include "../engine/MovementSystem.h"
#include <string>
#include <vector>

struct Entity;
class AnimationManager;
class GameEntityManager;

namespace UnitVisuals {

void BindUnit(AnimationManager& animationManager, int entityId, int sheetId);

std::string ResolveUnitAnimationId(bool isMoving, bool isAttacking, Direction direction);

void ApplyAnimations(std::vector<Entity>& entities,
                     AnimationManager& animationManager,
                     const GameEntityManager& gameEntityManager);

}

#endif