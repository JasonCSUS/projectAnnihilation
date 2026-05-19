#include "Units.h"

namespace {
    const UnitDefinition unit1Def = {
        UNIT1,              // unitType
        PLAYER,             // controller
        32,                 // radius
        250.0f,             // speed
        500.0f,             // visionRange
        500.0f,             // attackRange
        150,                // hp
        "unit1",            // name
        RENDER_BASE,        // renderPriority
        COLLISION_BASE,     // collisionLayer
        true                // heroic
    };

    const UnitDefinition unit2Def = {
        UNIT2,              // unitType
        ENEMY,              // controller
        32,                 // radius
        150.0f,             // speed
        500.0f,             // visionRange
        300.0f,             // attackRange
        20,                 // hp
        "unit2",            // name
        RENDER_BASE,        // renderPriority
        COLLISION_BASE,     // collisionLayer
        false               // heroic
    };
}

namespace Units {

const UnitDefinition& GetUnitDefinition(int unitType) {
    switch (unitType) {
        case UNIT1: return unit1Def;
        case UNIT2: return unit2Def;
        default:    return unit2Def;
    }
}

int GetStatePriority(int state) {
    switch (state) {
        case UNIT_STATE_ATTACKING: return 5;
        case UNIT_STATE_CHASING:   return 4;
        case UNIT_STATE_MOVING:    return 3;
        case UNIT_STATE_YIELDING:  return 2;
        case UNIT_STATE_STANDBY:   return 1;
        case UNIT_STATE_IDLE:
        default:                   return 0;
    }
}

bool LoadDefinitionsFromJson(const std::string& /*path*/) {
    // Stub for later.
    return false;
}

}