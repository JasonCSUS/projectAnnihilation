#ifndef GAMECONTEXT_H
#define GAMECONTEXT_H

#include "../engine/EntityManager.h"
#include "../engine/MapLoader.h"
#include "GameEntityManager.h"
#include "GameHUD.h"
#include "PlayerInput.h"
#include "SelectionState.h"

extern EntityManager entityManager;
extern GameEntityManager gameEntityManager;
extern PlayerInput inputHandler;
extern SelectionState selectionState;
extern MapLoader mapLoader;
extern GameHUD gameHUD;
extern float elapsedTime;

#endif
