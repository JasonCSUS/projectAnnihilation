#ifndef UNITS_H
#define UNITS_H

#include <string>

enum UnitType {
    UNIT1 = 1,
    UNIT2 = 2
};

enum ControllerType {
    PLAYER = 1,
    ENEMY = 2
};

enum UnitState {
    UNIT_STATE_IDLE = 0,
    UNIT_STATE_MOVING = 1,
    UNIT_STATE_CHASING = 2,
    UNIT_STATE_ATTACKING = 3,
    UNIT_STATE_YIELDING = 4,
    UNIT_STATE_STANDBY = 5
};

enum UnitRenderPriority {
    RENDER_BASE = 0,
    RENDER_MASSIVE = 1
};

enum UnitCollisionLayer {
    COLLISION_BASE = 0,
    COLLISION_MASSIVE = 1
};

struct UnitDefinition {
    int unitType = 0;
    int controller = 0;
    int radius = 32;
    float speed = 150.0f;
    float visionRange = 500.0f;
    float attackRange = 300.0f;
    int hp = 20;
    std::string name;

    int renderPriority = RENDER_BASE;
    int collisionLayer = COLLISION_BASE;
    bool heroic = false;
};

namespace Units {
    const UnitDefinition& GetUnitDefinition(int unitType);
    int GetStatePriority(int state);

    // Future hook for JSON loading.
    bool LoadDefinitionsFromJson(const std::string& path);
}

#endif