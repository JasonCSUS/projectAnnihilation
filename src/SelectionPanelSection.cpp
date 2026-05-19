#include "SelectionPanelSection.h"
#include "EntityData.h"

#include "../engine/EntityManager.h"

#include <SDL3/SDL.h>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>

namespace {
constexpr float PANEL_WIDTH = 420.0f;
constexpr float PANEL_HEIGHT = 140.0f;
constexpr float PANEL_MARGIN = 12.0f;
constexpr float ICON_SIZE = 48.0f;
constexpr float ICON_PADDING = 8.0f;
constexpr int GRID_COLUMNS = 6;

void DrawRect(SDL_Renderer* renderer,
              const SDL_FRect& rect,
              Uint8 r, Uint8 g, Uint8 b, Uint8 a,
              bool filled) {
    SDL_SetRenderDrawColor(renderer, r, g, b, a);
    if (filled) {
        SDL_RenderFillRect(renderer, &rect);
    } else {
        SDL_RenderRect(renderer, &rect);
    }
}

void DrawBar(SDL_Renderer* renderer,
             float x, float y, float w, float h,
             float ratio,
             Uint8 bgR, Uint8 bgG, Uint8 bgB,
             Uint8 fgR, Uint8 fgG, Uint8 fgB) {
    SDL_FRect bg{x, y, w, h};
    DrawRect(renderer, bg, bgR, bgG, bgB, 255, true);
    DrawRect(renderer, bg, 255, 255, 255, 255, false);

    const float clamped = std::max(0.0f, std::min(1.0f, ratio));
    SDL_FRect fg{x + 1.0f, y + 1.0f, std::max(0.0f, (w - 2.0f) * clamped), std::max(0.0f, h - 2.0f)};
    DrawRect(renderer, fg, fgR, fgG, fgB, 255, true);
}

void DrawDebugText(SDL_Renderer* renderer, float x, float y, const std::string& text) {
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_RenderDebugText(renderer, x, y, text.c_str());
}

bool PointInRect(float px, float py, const SDL_FRect& rect) {
    return px >= rect.x && px <= rect.x + rect.w &&
           py >= rect.y && py <= rect.y + rect.h;
}

} // namespace

SelectionPanelSection::SelectionPanelSection(SelectionState& selectionStateIn)
    : selectionState(selectionStateIn) {
}

void SelectionPanelSection::Update(float) {
}

SDL_FRect SelectionPanelSection::GetPanelRect(int screenW, int screenH) const {
    SDL_FRect rect{};
    rect.w = PANEL_WIDTH;
    rect.h = PANEL_HEIGHT;
    rect.x = (static_cast<float>(screenW) - rect.w) * 0.5f;
    rect.y = static_cast<float>(screenH) - rect.h - PANEL_MARGIN;
    return rect;
}

SDL_FRect SelectionPanelSection::GetSlotRect(const SDL_FRect& panelRect, int visibleIndex) const {
    const int col = visibleIndex % GRID_COLUMNS;
    const int row = visibleIndex / GRID_COLUMNS;

    SDL_FRect slot{};
    slot.x = panelRect.x + 10.0f + col * (ICON_SIZE + ICON_PADDING);
    slot.y = panelRect.y + 10.0f + row * (ICON_SIZE + ICON_PADDING);
    slot.w = ICON_SIZE;
    slot.h = ICON_SIZE;
    return slot;
}

void SelectionPanelSection::Render(SDL_Renderer* renderer,
                                   const MapLoader&,
                                   const EntityManager& entityManager,
                                   float,
                                   float,
                                   int screenW,
                                   int screenH) {
    RenderMarquee(renderer);

    if (selectionState.Empty()) {
        return;
    }

    const SDL_FRect panelRect = GetPanelRect(screenW, screenH);
    DrawRect(renderer, panelRect, 18, 18, 24, 235, true);
    DrawRect(renderer, panelRect, 140, 140, 160, 255, false);

    if (selectionState.GetSelectedIds().size() <= 1) {
        RenderSingleSelection(renderer, entityManager, panelRect);
    } else {
        RenderMultiSelection(renderer, entityManager, panelRect, screenW, screenH);
    }
}

void SelectionPanelSection::RenderMarquee(SDL_Renderer* renderer) const {
    if (!selectionState.IsMarqueeVisible()) {
        return;
    }

    const SDL_FRect rect = selectionState.GetMarqueeRect();
    DrawRect(renderer, rect, 0, 255, 0, 40, true);
    DrawRect(renderer, rect, 0, 255, 0, 255, false);
}

void SelectionPanelSection::RenderSingleSelection(SDL_Renderer* renderer,
                                                  const EntityManager& entityManager,
                                                  const SDL_FRect& panelRect) const {
    const int entityId = selectionState.GetPrimarySelectedId();
    const Entity* entity = entityManager.GetEntityById(entityId);
    const EntityInfo* info = EntityData::TryGet(entityId);

    if (!entity || entity->isDead || !info) {
        return;
    }

    const SDL_FRect portraitRect{panelRect.x + 10.0f, panelRect.y + 10.0f, 64.0f, 64.0f};
    DrawRect(renderer, portraitRect, 40, 40, 55, 255, true);
    DrawRect(renderer, portraitRect, 200, 200, 220, 255, false);

    DrawDebugText(renderer, panelRect.x + 84.0f, panelRect.y + 10.0f, EntityData::GetDisplayName(*info));

    const float hpRatio = (info->maxHp > 0) ? static_cast<float>(info->hp) / static_cast<float>(info->maxHp) : 0.0f;
    DrawBar(renderer, panelRect.x + 84.0f, panelRect.y + 28.0f, 160.0f, 10.0f, hpRatio, 40, 20, 20, 0, 220, 0);

    const float shieldRatio = (info->maxShield > 0) ? static_cast<float>(info->shield) / static_cast<float>(info->maxShield) : 0.0f;
    DrawBar(renderer, panelRect.x + 84.0f, panelRect.y + 44.0f, 160.0f, 10.0f, shieldRatio, 20, 20, 50, 60, 140, 255);

    std::ostringstream line1;
    line1 << "HP " << info->hp << "/" << info->maxHp
          << "  Shield " << info->shield << "/" << info->maxShield;
    DrawDebugText(renderer, panelRect.x + 10.0f, panelRect.y + 84.0f, line1.str());

    std::ostringstream line2;
    line2 << "Damage " << EntityData::GetDisplayedDamage(*info)
          << "  AtkSpd " << EntityData::GetDisplayedAttackSpeed(*info);
    DrawDebugText(renderer, panelRect.x + 10.0f, panelRect.y + 98.0f, line2.str());

    std::ostringstream line3;
    line3 << "Armor " << EntityData::GetDisplayedArmor(*info)
          << "  ShieldArmor " << EntityData::GetDisplayedShieldArmor(*info)
          << "  Move " << EntityData::GetFinalMoveSpeed(*info);
    DrawDebugText(renderer, panelRect.x + 10.0f, panelRect.y + 112.0f, line3.str());
}

void SelectionPanelSection::RenderMultiSelection(SDL_Renderer* renderer,
                                                 const EntityManager& entityManager,
                                                 const SDL_FRect& panelRect,
                                                 int screenW,
                                                 int screenH) const {
    int visibleIndex = 0;
    for (int entityId : selectionState.GetSelectedIds()) {
        const Entity* entity = entityManager.GetEntityById(entityId);
        const EntityInfo* info = EntityData::TryGet(entityId);
        if (!entity || entity->isDead || !info) {
            continue;
        }

        const SDL_FRect slot = GetSlotRect(panelRect, visibleIndex);
        const bool primary = (entityId == selectionState.GetPrimarySelectedId());

        DrawRect(renderer, slot, 35, 35, 45, 255, true);
        DrawRect(renderer, slot, primary ? 255 : 170, primary ? 255 : 170, primary ? 0 : 170, 255, false);

        const float hpRatio = (info->maxHp > 0) ? static_cast<float>(info->hp) / static_cast<float>(info->maxHp) : 0.0f;
        DrawBar(renderer, slot.x + 4.0f, slot.y + slot.h - 8.0f, slot.w - 8.0f, 4.0f, hpRatio, 40, 20, 20, 0, 220, 0);

        ++visibleIndex;
    }

    const int hoveredId = FindHoveredEntityId(screenW, screenH);
    if (hoveredId >= 0) {
        const EntityInfo* info = EntityData::TryGet(hoveredId);
        if (info) {
            const std::string tooltip = EntityData::GetDisplayName(*info) + "  HP " +
                                        std::to_string(info->hp) + "/" + std::to_string(info->maxHp);
            DrawDebugText(renderer, panelRect.x + 10.0f, panelRect.y + panelRect.h - 14.0f, tooltip);
        }
    }
}

int SelectionPanelSection::FindEntityForSlotClick(float mouseX, float mouseY, int screenW, int screenH) const {
    if (selectionState.GetSelectedIds().size() <= 1) {
        return -1;
    }

    const SDL_FRect panelRect = GetPanelRect(screenW, screenH);

    int visibleIndex = 0;
    for (int entityId : selectionState.GetSelectedIds()) {
        const SDL_FRect slot = GetSlotRect(panelRect, visibleIndex);
        if (PointInRect(mouseX, mouseY, slot)) {
            return entityId;
        }
        ++visibleIndex;
    }

    return -1;
}

int SelectionPanelSection::FindHoveredEntityId(int screenW, int screenH) const {
    float mouseX = 0.0f;
    float mouseY = 0.0f;
    SDL_GetMouseState(&mouseX, &mouseY);
    return FindEntityForSlotClick(mouseX, mouseY, screenW, screenH);
}

bool SelectionPanelSection::HandleEvent(const SDL_Event& event,
                                        float&,
                                        float&,
                                        int screenW,
                                        int screenH) {
    if (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN && event.button.button == SDL_BUTTON_LEFT) {
        const int entityId = FindEntityForSlotClick(static_cast<float>(event.button.x),
                                                    static_cast<float>(event.button.y),
                                                    screenW,
                                                    screenH);
        if (entityId >= 0) {
            selectionState.SetPrimarySelectedId(entityId);
            return true;
        }
    }

    return false;
}
