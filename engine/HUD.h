#ifndef HUD_H
#define HUD_H

#include <SDL3/SDL.h>

class EntityManager;
class MapLoader;

class HUD {
public:
    virtual ~HUD() = default;

    virtual void Update(float deltaTime) = 0;

    virtual void Render(SDL_Renderer* renderer,
                        const MapLoader& mapLoader,
                        const EntityManager& entityManager,
                        float cameraX,
                        float cameraY,
                        int screenW,
                        int screenH) = 0;

    // Return true if the HUD consumed the event.
    virtual bool HandleEvent(const SDL_Event& event,
                             float& cameraX,
                             float& cameraY,
                             int screenW,
                             int screenH) = 0;
};

#endif // HUD_H