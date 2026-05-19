#include "EntityManager.h"

#include <SDL3/SDL.h>
#include <algorithm>
#include <iostream>

Entity& EntityManager::AddEntity(int radius,
                                 const SDL_FRect& position,
                                 float speed) {
    static int nextID = 1;

    Entity entity{};
    entity.id = nextID++;
    entity.radius = radius;
    entity.position = position;
    entity.speed = speed;

    entities.push_back(std::move(entity));
    entityIndex[entities.back().id] = entities.size() - 1;
    return entities.back();
}

void EntityManager::RemoveDeadEntities() {
    size_t i = 0;
    while (i < entities.size()) {
        if (entities[i].isDead) {
            animationManager.RemoveEntityAnim(entities[i].id);
            if (i + 1 < entities.size()) {
                entities[i] = std::move(entities.back());
            }
            entities.pop_back();
        } else {
            ++i;
        }
    }
    entityIndex.clear();
    entityIndex.reserve(entities.size());
    for (size_t j = 0; j < entities.size(); ++j) {
        entityIndex[entities[j].id] = j;
    }
}

Entity* EntityManager::GetEntityById(int id) {
    const auto it = entityIndex.find(id);
    if (it == entityIndex.end()) return nullptr;
    return &entities[it->second];
}

const Entity* EntityManager::GetEntityById(int id) const {
    const auto it = entityIndex.find(id);
    if (it == entityIndex.end()) return nullptr;
    return &entities[it->second];
}

void EntityManager::RenderEntities(SDL_Renderer* renderer,
                                   float& cameraX,
                                   float& cameraY,
                                   float& cameraW,
                                   float& cameraH,
                                   float /*deltaTime*/) {
    renderScratch.clear();
    renderScratch.reserve(entities.size());

    for (auto& entity : entities) {
        if (!entity.isDead) {
            renderScratch.push_back(&entity);
        }
    }

    std::stable_sort(renderScratch.begin(), renderScratch.end(),
        [](const Entity* a, const Entity* b) {
            if (a->renderPriority != b->renderPriority) {
                return a->renderPriority < b->renderPriority;
            }
            return a->position.y < b->position.y;
        });

    for (Entity* entity : renderScratch) {
        int spriteW = 0;
        int spriteH = 0;
        if (!animationManager.GetCurrentFrameDrawSize(entity->id, spriteW, spriteH)) {
            continue;
        }

        const int renderX = static_cast<int>(entity->position.x - cameraX);
        const int renderY = static_cast<int>(entity->position.y - cameraY);

        if (renderX + spriteW < 0 || renderX > cameraW ||
            renderY + spriteH < 0 || renderY > cameraH) {
            continue;
        }

        animationManager.RenderEntity(renderer, entity->id, renderX, renderY);
    }
}