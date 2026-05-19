#ifndef MOVEMENTSYSTEM_H
#define MOVEMENTSYSTEM_H

class EntityManager;
class AnimationManager;

enum Direction { UP, DOWN, LEFT, RIGHT };

class MovementSystem {
public:
    void Update(EntityManager& entityManager, AnimationManager& animationManager, float deltaTime);
};

#endif
