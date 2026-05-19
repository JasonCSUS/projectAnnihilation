#include "MinimapSection.h"
#include "../engine/EntityManager.h"
#include "../engine/MapLoader.h"
#include "GameEntityManager.h"
#include "PlayerInput.h"
#include "GameMain.h"
#include "Units.h"

#include <algorithm>
#include <cmath>

namespace {
    constexpr float MINIMAP_MARGIN = 16.0f;
    constexpr float BLIP_SIZE = 4.0f;
}

MinimapSection::MinimapSection(PlayerInput& inputHandlerIn,
                               EntityManager& entityManagerIn,
                               GameEntityManager& gameEntityManagerIn)
    : inputHandler(inputHandlerIn),
      entityManagerRef(entityManagerIn),
      gameEntityManagerRef(gameEntityManagerIn) {
}

void MinimapSection::Update(float /*deltaTime*/) {
}

bool MinimapSection::GetMinimapRect(int screenW, int screenH, SDL_FRect& outRect) const {
    const float size = static_cast<float>(screenH) / 5.0f;
    outRect.x = MINIMAP_MARGIN;
    outRect.y = static_cast<float>(screenH) - size - MINIMAP_MARGIN;
    outRect.w = size;
    outRect.h = size;
    return true;
}

bool MinimapSection::PointInsideRect(float px, float py, const SDL_FRect& rect) const {
    return px >= rect.x && px <= rect.x + rect.w &&
           py >= rect.y && py <= rect.y + rect.h;
}

void MinimapSection::Render(SDL_Renderer* renderer,
                            const MapLoader& mapLoader,
                            const EntityManager& entityManager,
                            float cameraX,
                            float cameraY,
                            int screenW,
                            int screenH) {
    SDL_FRect mini;
    GetMinimapRect(screenW, screenH, mini);

    cachedMapWidth = static_cast<float>(std::max(1, mapLoader.GetMapWidth()));
    cachedMapHeight = static_cast<float>(std::max(1, mapLoader.GetMapHeight()));

    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderFillRect(renderer, &mini);

    if (mapLoader.GetMapTexture() != nullptr) {
        SDL_RenderTexture(renderer, mapLoader.GetMapTexture(), nullptr, &mini);
    }

    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_RenderRect(renderer, &mini);

    for (const auto& entity : entityManager.entities) {
        if (entity.isDead) {
            continue;
        }

        const float worldX = entity.position.x;
        const float worldY = entity.position.y;

        const float mx = mini.x + (worldX / cachedMapWidth) * mini.w;
        const float my = mini.y + (worldY / cachedMapHeight) * mini.h;

        SDL_FRect blip = {
            mx - BLIP_SIZE * 0.5f,
            my - BLIP_SIZE * 0.5f,
            BLIP_SIZE,
            BLIP_SIZE
        };

        const int controller = gameEntityManagerRef.GetEntityController(entity.id);
        if (controller == PLAYER) {
            SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
        } else {
            SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
        }

        SDL_RenderFillRect(renderer, &blip);
    }

    float camMiniX = mini.x + (cameraX / cachedMapWidth) * mini.w;
    float camMiniY = mini.y + (cameraY / cachedMapHeight) * mini.h;
    float camMiniW = (static_cast<float>(screenW) / cachedMapWidth) * mini.w;
    float camMiniH = (static_cast<float>(screenH) / cachedMapHeight) * mini.h;

    camMiniW = std::min(camMiniW, mini.w);
    camMiniH = std::min(camMiniH, mini.h);

    camMiniX = std::max(mini.x, std::min(camMiniX, mini.x + mini.w - camMiniW));
    camMiniY = std::max(mini.y, std::min(camMiniY, mini.y + mini.h - camMiniH));

    SDL_FRect camRect = {
        camMiniX,
        camMiniY,
        camMiniW,
        camMiniH
    };

    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_RenderRect(renderer, &camRect);
}

bool MinimapSection::HandleEvent(const SDL_Event& event,
                                 float& cameraX,
                                 float& cameraY,
                                 int screenW,
                                 int screenH) {
    SDL_FRect mini;
    GetMinimapRect(screenW, screenH, mini);

    if (event.type == SDL_EVENT_MOUSE_BUTTON_UP && event.button.button == SDL_BUTTON_LEFT && consumingLeftClick) {
        consumingLeftClick = false;
        inputHandler.SetIgnoreNextLeftRelease(true);
        return true;
    }

    if (event.type != SDL_EVENT_MOUSE_BUTTON_DOWN) {
        return false;
    }

    const float mx = static_cast<float>(event.button.x);
    const float my = static_cast<float>(event.button.y);

    if (!PointInsideRect(mx, my, mini)) {
        return false;
    }

    const float u = (mx - mini.x) / mini.w;
    const float v = (my - mini.y) / mini.h;

    const float worldX = u * cachedMapWidth;
    const float worldY = v * cachedMapHeight;

    if (event.button.button == SDL_BUTTON_LEFT) {
        consumingLeftClick = true;
        inputHandler.PanCameraToWorld(
            worldX,
            worldY,
            cameraX,
            cameraY,
            screenW,
            screenH,
            cachedMapWidth,
            cachedMapHeight
        );
        return true;
    }

    if (event.button.button == SDL_BUTTON_RIGHT) {
        inputHandler.MoveSelectedToWorld(entityManagerRef, worldX, worldY);
        return true;
    }

    return false;
}
