#ifndef MINIMAPSECTION_H
#define MINIMAPSECTION_H

#include "GameHUD.h"

class PlayerInput;
class EntityManager;
class GameEntityManager;

class MinimapSection : public GameHUDSection {
public:
    MinimapSection(PlayerInput& inputHandler,
                   EntityManager& entityManager,
                   GameEntityManager& gameEntityManager);

    void Update(float deltaTime) override;

    void Render(SDL_Renderer* renderer,
                const MapLoader& mapLoader,
                const EntityManager& entityManager,
                float cameraX,
                float cameraY,
                int screenW,
                int screenH) override;

    bool HandleEvent(const SDL_Event& event,
                     float& cameraX,
                     float& cameraY,
                     int screenW,
                     int screenH) override;

private:
    bool GetMinimapRect(int screenW, int screenH, SDL_FRect& outRect) const;
    bool PointInsideRect(float px, float py, const SDL_FRect& rect) const;

private:
    PlayerInput& inputHandler;
    EntityManager& entityManagerRef;
    GameEntityManager& gameEntityManagerRef;
    float cachedMapWidth = 4000.0f;
    float cachedMapHeight = 4000.0f;
    bool consumingLeftClick = false;
};

#endif
