#ifndef UNITCOLLISIONSYSTEM_H
#define UNITCOLLISIONSYSTEM_H

class EntityManager;
class GameEntityManager;

class UnitCollisionSystem {
public:
    static UnitCollisionSystem& Instance();

    void Update(EntityManager& entityManager,
                GameEntityManager& gameEntityManager,
                float deltaTime);

private:
    UnitCollisionSystem() = default;
    UnitCollisionSystem(const UnitCollisionSystem&) = delete;
    UnitCollisionSystem& operator=(const UnitCollisionSystem&) = delete;
};

#endif
