#ifndef ENTITYLOGIC_H
#define ENTITYLOGIC_H

#include "../engine/EntityManager.h"
#include "../engine/MapLoader.h"

// Updates enemy AI using the new navmesh singleton.
void UpdateEnemyAI(EntityManager& entityManager);

#endif // ENTITYLOGIC_H
