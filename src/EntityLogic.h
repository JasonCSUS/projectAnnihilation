#ifndef ENTITYLOGIC_H
#define ENTITYLOGIC_H

#include "../engine/EntityManager.h"

class GameEntityManager;

void UpdateEnemyAI(EntityManager& entityManager,
                   GameEntityManager& gameEntityManager,
                   float deltaTime);

#endif