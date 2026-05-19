#include "GameEntityManager.h"

#include "../engine/AnimationManager.h"
#include "../engine/MapLoader.h"
#include "../engine/NavigationSystem.h"
#include "../engine/Rigidbodies.h"

#include "UnitAnimations.h"
#include "UnitVisuals.h"
#include "EntityData.h"

#include <cmath>
#include <iostream>

void GameEntityManager::Clear() {
    definitions.clear();
    visuals.clear();
    directions.clear();
    nextSheetId = 1;
}

namespace {
    void ComputeRenderSizeFromRadius(int spriteX,
                                     int spriteY,
                                     int radius,
                                     int& outW,
                                     int& outH) {
        spriteX = std::max(1, spriteX);
        spriteY = std::max(1, spriteY);
        radius = std::max(1, radius);

        const int targetMaxSide = radius * 2;
        const int sourceMaxSide = std::max(spriteX, spriteY);
        const float scale = static_cast<float>(targetMaxSide) / static_cast<float>(sourceMaxSide);

        outW = std::max(1, static_cast<int>(std::round(spriteX * scale)));
        outH = std::max(1, static_cast<int>(std::round(spriteY * scale)));
    }
}

bool GameEntityManager::RegisterSheetForEntity(EntityManager& entityManager,
                                               MapLoader& mapLoader,
                                               SDL_Renderer* renderer,
                                               const std::string& entityName,
                                               const std::string& spritePath,
                                               int cellWidth,
                                               int cellHeight,
                                               int rows,
                                               int cols) {
    if (entityName.empty() || spritePath.empty() || !renderer) {
        return false;
    }

    SDL_Texture* texture = mapLoader.LoadTexture(spritePath, renderer);
    if (!texture) {
        std::cerr << "Failed to load texture for entity '" << entityName
                  << "' from path: " << spritePath << std::endl;
        return false;
    }

    const int sheetId = nextSheetId++;
    if (!entityManager.GetAnimationManager().CreateSheet(
            sheetId,
            entityName,
            texture,
            cellWidth,
            cellHeight,
            rows,
            cols)) {
        std::cerr << "Failed to create animation sheet for entity '" << entityName << "'\n";
        return false;
    }

    RegisteredVisual visual;
    visual.sheetId = sheetId;
    visual.isBuilding = (rows == 1);
    visuals[entityName] = visual;
    return true;
}

void GameEntityManager::RegisterDefinition(const GameEntityDefinition& def) {
    if (def.name.empty()) {
        return;
    }
    definitions[def.name] = def;
}

const GameEntityDefinition* GameEntityManager::TryGetDefinition(const std::string& entityName) const {
    auto it = definitions.find(entityName);
    if (it == definitions.end()) {
        return nullptr;
    }
    return &it->second;
}

int GameEntityManager::SpawnUnit(EntityManager& entityManager,
                                 const std::string& entityName,
                                 float x,
                                 float y,
                                 int renderPriority) {
    const GameEntityDefinition* def = TryGetDefinition(entityName);
    auto visIt = visuals.find(entityName);

    if (!def || visIt == visuals.end()) {
        std::cerr << "SpawnUnit failed for entity '" << entityName
                  << "' (missing definition or sheet)\n";
        return -1;
    }

    int renderW = 0;
    int renderH = 0;
    ComputeRenderSizeFromRadius(def->spriteX, def->spriteY, def->radius, renderW, renderH);

    SDL_FRect pos{
        x,
        y,
        static_cast<float>(renderW),
        static_cast<float>(renderH)
    };

    Entity& entity = entityManager.AddEntity(def->radius, pos, def->moveSpeed);
    entity.renderPriority = renderPriority;

    directions[entity.id] = DOWN;

    UnitVisuals::BindUnit(
        entityManager.GetAnimationManager(),
        entity.id,
        visIt->second.sheetId
    );

    entityManager.GetAnimationManager().SetRenderSize(entity.id, renderW, renderH);

    return entity.id;
}

int GameEntityManager::SpawnBuilding(EntityManager& entityManager,
                                     const std::string& entityName,
                                     float x,
                                     float y,
                                     int renderPriority) {
    const GameEntityDefinition* def = TryGetDefinition(entityName);
    auto visIt = visuals.find(entityName);

    if (!def || visIt == visuals.end()) {
        std::cerr << "SpawnBuilding failed for entity '" << entityName
                  << "' (missing definition or sheet)\n";
        return -1;
    }

    int renderW = 0;
    int renderH = 0;
    ComputeRenderSizeFromRadius(def->spriteX, def->spriteY, def->radius, renderW, renderH);

    SDL_FRect pos{
        x,
        y,
        static_cast<float>(renderW),
        static_cast<float>(renderH)
    };

    Entity& entity = entityManager.AddEntity(def->radius, pos, 0.0f);
    entity.renderPriority = renderPriority;

    directions[entity.id] = DOWN;

    UnitAnimations::BindBuildingEntity(
        entityManager.GetAnimationManager(),
        entity.id,
        visIt->second.sheetId
    );

    entityManager.GetAnimationManager().SetRenderSize(entity.id, renderW, renderH);

    return entity.id;
}

void GameEntityManager::KillEntity(EntityManager& entityManager, int entityId) {
    Entity* entity = entityManager.GetEntityById(entityId);
    if (entity) {
        entity->isDead = true;
        directions.erase(entityId);
    }
}

void GameEntityManager::SetEntityDirection(int entityId, Direction direction) {
    directions[entityId] = direction;
}

Direction GameEntityManager::GetEntityDirection(int entityId) const {
    auto it = directions.find(entityId);
    if (it == directions.end()) {
        return DOWN;
    }
    return it->second;
}

void GameEntityManager::Update(EntityManager& entityManager, float deltaTime) {
    std::unordered_map<int, RigidbodySystem::RigidbodyDesc> bodyOverrides;
    bodyOverrides.reserve(entityManager.entities.size());
    for (const auto& entity : entityManager.entities) {
        const EntityInfo* info = EntityData::TryGet(entity.id);
        RigidbodySystem::RigidbodyDesc desc{};
        desc.enabled = !entity.isDead;
        desc.bodyType = RigidbodySystem::BodyType::Circle;
        if (info && info->massive) {
            desc.layer = 2;
            desc.collisionMask = (1u << 2);
        } else if (info && info->heroic) {
            desc.layer = 1;
            desc.collisionMask = (1u << 1);
        } else {
            desc.layer = 0;
            desc.collisionMask = (1u << 0) | (1u << 1);
        }
        desc.entityCollision = !(info && info->isStatic && entity.radius <= 0);
        desc.mapCollision = true;
        desc.allowOutOfBounds = false;
        desc.immovable = info && info->isStatic;
        desc.radius = static_cast<float>(entity.radius);
        bodyOverrides[entity.id] = desc;
    }
    RigidbodySystem::SyncFromEntities(entityManager, &bodyOverrides);

    NavigationSystem::Instance().Update(entityManager, deltaTime);
    movementSystem.Update(entityManager, entityManager.GetAnimationManager(), deltaTime);

    for (auto& entity : entityManager.entities) {
        if (entity.isDead) {
            continue;
        }

        if (entity.pathIndex < entity.path.size()) {
            const Vec2& next = entity.path[entity.pathIndex];
            const float dx = next.x - entity.position.x;
            const float dy = next.y - entity.position.y;

            if (std::fabs(dx) > std::fabs(dy)) {
                directions[entity.id] = (dx < 0.0f) ? LEFT : RIGHT;
            } else if (std::fabs(dy) > 0.001f) {
                directions[entity.id] = (dy < 0.0f) ? UP : DOWN;
            }
        }
    }

    entityManager.GetAnimationManager().UpdateAnimations(deltaTime);
    UnitVisuals::ApplyAnimations(entityManager.entities, entityManager.GetAnimationManager(), *this);
}

bool GameEntityManager::IsEntityStatic(int entityId) const {
    const EntityInfo* info = EntityData::TryGet(entityId);
    return info ? info->isStatic : false;
}

bool GameEntityManager::IsEntityHeroic(int entityId) const {
    const EntityInfo* info = EntityData::TryGet(entityId);
    return info ? info->heroic : false;
}

bool GameEntityManager::IsEntityMassive(int entityId) const {
    const EntityInfo* info = EntityData::TryGet(entityId);
    return info ? info->massive : false;
}

int GameEntityManager::GetEntityController(int entityId) const {
    const EntityInfo* info = EntityData::TryGet(entityId);
    return info ? info->controller : -1;
}

bool GameEntityManager::ShouldEntitiesCollide(int entityIdA, int entityIdB) const {
    const EntityInfo* a = EntityData::TryGet(entityIdA);
    const EntityInfo* b = EntityData::TryGet(entityIdB);
    if (!a || !b) {
        return false;
    }
    const auto layerFor = [](const EntityInfo* info) {
        if (!info) return 0;
        if (info->massive) return 2;
        if (info->heroic) return 1;
        return 0;
    };

    const int layerA = layerFor(a);
    const int layerB = layerFor(b);
    const unsigned int maskA = (layerA == 2) ? (1u << 2) : (layerA == 1 ? (1u << 1) : ((1u << 0) | (1u << 1)));
    const unsigned int maskB = (layerB == 2) ? (1u << 2) : (layerB == 1 ? (1u << 1) : ((1u << 0) | (1u << 1)));
    return (maskA & (1u << layerB)) != 0u && (maskB & (1u << layerA)) != 0u;
}
