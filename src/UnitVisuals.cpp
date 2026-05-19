#include "UnitVisuals.h"
#include "UnitAnimations.h"
#include "EntityData.h"
#include "GameEntityManager.h"

namespace UnitVisuals {

void BindUnit(AnimationManager& animationManager, int entityId, int sheetId) {
    UnitAnimations::BindUnitEntity(animationManager, entityId, sheetId);
}

std::string ResolveUnitAnimationId(bool isMoving, bool isAttacking, Direction direction) {
    if (isAttacking) {
        switch (direction) {
            case UP:    return UnitAnimations::ATTACK_UP;
            case LEFT:  return UnitAnimations::ATTACK_RIGHT;
            case RIGHT: return UnitAnimations::ATTACK_RIGHT;
            case DOWN:
            default:    return UnitAnimations::ATTACK_DOWN;
        }
    }

    if (isMoving) {
        switch (direction) {
            case UP:    return UnitAnimations::MOVE_UP;
            case LEFT:  return UnitAnimations::MOVE_RIGHT;
            case RIGHT: return UnitAnimations::MOVE_RIGHT;
            case DOWN:
            default:    return UnitAnimations::MOVE_DOWN;
        }
    }

    switch (direction) {
        case UP:    return UnitAnimations::IDLE_UP;
        case LEFT:  return UnitAnimations::IDLE_RIGHT;
        case RIGHT: return UnitAnimations::IDLE_RIGHT;
        case DOWN:
        default:    return UnitAnimations::IDLE_DOWN;
    }
}

void ApplyAnimations(std::vector<Entity>& entities,
                     AnimationManager& animationManager,
                     const GameEntityManager& gameEntityManager) {
    for (auto& entity : entities) {
        if (entity.isDead) {
            continue;
        }

        EntityInfo* info = EntityData::TryGet(entity.id);
        if (!info) {
            continue;
        }

        if (info->isStatic) {
            continue;
        }

        const bool isMoving = entity.pathIndex < entity.path.size();
        const bool isAttacking = (info->state == UNIT_STATE_ATTACKING);
        const Direction direction = gameEntityManager.GetEntityDirection(entity.id);

        const std::string animId = ResolveUnitAnimationId(
            isMoving,
            isAttacking,
            direction
        );

        const float frameDuration = isMoving ? 0.15f : 0.25f;
        const bool flipX = (direction == LEFT);

        UnitAnimations::SetUnitAnimation(
            animationManager,
            entity.id,
            animId,
            frameDuration,
            true,
            false,
            flipX
        );
    }
}

}