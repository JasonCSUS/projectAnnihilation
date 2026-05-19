#ifndef GAMEHUD_H
#define GAMEHUD_H

#include "../engine/HUD.h"
#include <memory>
#include <vector>

class GameHUDSection {
public:
    virtual ~GameHUDSection() = default;

    virtual void Update(float deltaTime) = 0;

    virtual void Render(SDL_Renderer* renderer,
                        const MapLoader& mapLoader,
                        const EntityManager& entityManager,
                        float cameraX,
                        float cameraY,
                        int screenW,
                        int screenH) = 0;

    // Return true if the section consumed the event.
    virtual bool HandleEvent(const SDL_Event& event,
                             float& cameraX,
                             float& cameraY,
                             int screenW,
                             int screenH) = 0;
};

class GameHUD : public HUD {
public:
    GameHUD() = default;
    ~GameHUD() override = default;

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

    void AddSection(std::unique_ptr<GameHUDSection> section);
    void ClearSections();

private:
    std::vector<std::unique_ptr<GameHUDSection>> sections;
};

#endif // GAMEHUD_H