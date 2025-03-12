#ifndef MOVEMENTSYSTEM_H
#define MOVEMENTSYSTEM_H

#include <SDL3/SDL.h>
#include <vector>
#include "Character.h" // For rendering and interacting with characters
#include "AnimationManager.h"
#include "MapLoader.h"

struct Entity; // Forward declaration to avoid circular dependency

enum Direction { UP, DOWN, LEFT, RIGHT };

class MovementSystem {
public:
    void Update(std::vector<Entity>& entities, AnimationManager& animationManager, float deltaTime);
};

#endif
