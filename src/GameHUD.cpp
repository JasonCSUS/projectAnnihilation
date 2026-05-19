#include "GameHUD.h"

void GameHUD::Update(float deltaTime) {
    for (auto& section : sections) {
        if (section) {
            section->Update(deltaTime);
        }
    }
}

void GameHUD::Render(SDL_Renderer* renderer,
                     const MapLoader& mapLoader,
                     const EntityManager& entityManager,
                     float cameraX,
                     float cameraY,
                     int screenW,
                     int screenH) {
    for (auto& section : sections) {
        if (section) {
            section->Render(renderer, mapLoader, entityManager, cameraX, cameraY, screenW, screenH);
        }
    }
}

bool GameHUD::HandleEvent(const SDL_Event& event,
                          float& cameraX,
                          float& cameraY,
                          int screenW,
                          int screenH) {
    // Let later-added sections get first crack if you want topmost UI first.
    // For now, iterate in reverse so newer sections can sit "on top."
    for (auto it = sections.rbegin(); it != sections.rend(); ++it) {
        if (*it && (*it)->HandleEvent(event, cameraX, cameraY, screenW, screenH)) {
            return true;
        }
    }

    return false;
}

void GameHUD::AddSection(std::unique_ptr<GameHUDSection> section) {
    if (section) {
        sections.push_back(std::move(section));
    }
}

void GameHUD::ClearSections() {
    sections.clear();
}