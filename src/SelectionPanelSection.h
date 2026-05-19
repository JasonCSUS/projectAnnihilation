#ifndef SELECTIONPANELSECTION_H
#define SELECTIONPANELSECTION_H

#include "GameHUD.h"
#include "SelectionState.h"

class SelectionPanelSection : public GameHUDSection {
public:
    explicit SelectionPanelSection(SelectionState& selectionState);

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
    SDL_FRect GetPanelRect(int screenW, int screenH) const;
    void RenderMarquee(SDL_Renderer* renderer) const;

    void RenderSingleSelection(SDL_Renderer* renderer,
                               const EntityManager& entityManager,
                               const SDL_FRect& panelRect) const;

    void RenderMultiSelection(SDL_Renderer* renderer,
                              const EntityManager& entityManager,
                              const SDL_FRect& panelRect,
                              int screenW,
                              int screenH) const;

    SDL_FRect GetSlotRect(const SDL_FRect& panelRect, int visibleIndex) const;
    int FindEntityForSlotClick(float mouseX, float mouseY, int screenW, int screenH) const;
    int FindHoveredEntityId(int screenW, int screenH) const;

private:
    SelectionState& selectionState;
};

#endif
