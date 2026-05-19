#include "Rigidbodies.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace RigidbodySystem {
namespace {

constexpr float GRID_CELL_SIZE = 96.0f;
constexpr float MAP_COLLISION_MARGIN = 1.0f;

using GridKey = long long;

struct Registry {
    std::unordered_map<int, BodyState> bodies;
    std::unordered_map<int, std::unordered_map<GridKey, std::vector<int>>> gridsByLayer;
    std::unordered_map<int, CollisionReport> lastReports;
} g_registry;

GridKey MakeKey(int cx, int cy) {
    return (static_cast<long long>(cx) << 32) ^ static_cast<unsigned int>(cy);
}

int CellX(float x) {
    return static_cast<int>(std::floor(x / GRID_CELL_SIZE));
}

int CellY(float y) {
    return static_cast<int>(std::floor(y / GRID_CELL_SIZE));
}

void InsertIntoGrid(const BodyState& body) {
    auto& grid = g_registry.gridsByLayer[body.desc.layer];
    const int minCellX = CellX(body.x - body.desc.radius);
    const int maxCellX = CellX(body.x + body.desc.radius);
    const int minCellY = CellY(body.y - body.desc.radius);
    const int maxCellY = CellY(body.y + body.desc.radius);

    for (int cy = minCellY; cy <= maxCellY; ++cy) {
        for (int cx = minCellX; cx <= maxCellX; ++cx) {
            grid[MakeKey(cx, cy)].push_back(body.entityId);
        }
    }
}

void RemoveFromGrid(int layer, float x, float y, float radius, int entityId) {
    auto layerIt = g_registry.gridsByLayer.find(layer);
    if (layerIt == g_registry.gridsByLayer.end()) return;
    auto& grid = layerIt->second;

    const int minCellX = CellX(x - radius);
    const int maxCellX = CellX(x + radius);
    const int minCellY = CellY(y - radius);
    const int maxCellY = CellY(y + radius);

    for (int cy = minCellY; cy <= maxCellY; ++cy) {
        for (int cx = minCellX; cx <= maxCellX; ++cx) {
            auto cellIt = grid.find(MakeKey(cx, cy));
            if (cellIt == grid.end()) continue;
            auto& vec = cellIt->second;
            vec.erase(std::remove(vec.begin(), vec.end(), entityId), vec.end());
            if (vec.empty()) grid.erase(cellIt);
        }
    }
}

const BodyState* FindBody(int entityId) {
    auto it = g_registry.bodies.find(entityId);
    if (it == g_registry.bodies.end()) {
        return nullptr;
    }
    return &it->second;
}

bool LayersCanCollide(const RigidbodyDesc& a, const RigidbodyDesc& b) {
    if (a.layer < 0 || b.layer < 0) {
        return false;
    }
    const unsigned int otherALayerBit = (b.layer >= 32) ? 0u : (1u << static_cast<unsigned int>(b.layer));
    const unsigned int otherBLayerBit = (a.layer >= 32) ? 0u : (1u << static_cast<unsigned int>(a.layer));
    return (a.collisionMask & otherALayerBit) != 0u &&
           (b.collisionMask & otherBLayerBit) != 0u;
}

bool CheckCircleOverlap(float ax,
                        float ay,
                        float ar,
                        float bx,
                        float by,
                        float br,
                        float* outNormalX,
                        float* outNormalY,
                        float* outOverlap) {
    const float dx = ax - bx;
    const float dy = ay - by;
    const float combined = ar + br;
    const float distSq = dx * dx + dy * dy;
    if (distSq >= combined * combined) {
        return false;
    }

    float nx = 1.0f;
    float ny = 0.0f;
    float dist = 0.0f;

    if (distSq > 0.0001f) {
        dist = std::sqrt(distSq);
        nx = dx / dist;
        ny = dy / dist;
    }

    if (outNormalX) *outNormalX = nx;
    if (outNormalY) *outNormalY = ny;
    if (outOverlap) *outOverlap = combined - dist;
    return true;
}

bool WouldHitMap(const BodyState& body, float x, float y, CollisionHit* outHit) {
    if (!body.desc.mapCollision) {
        return false;
    }

    float pushX = 0.0f;
    float pushY = 0.0f;
    const bool hit = NavMesh::Instance().ResolveSoftCollision(
        {static_cast<int>(std::lround(x)), static_cast<int>(std::lround(y))},
        body.desc.radius + MAP_COLLISION_MARGIN,
        pushX,
        pushY);

    if (hit && outHit) {
        outHit->kind = CollisionKind::Map;
        const float lenSq = pushX * pushX + pushY * pushY;
        if (lenSq > 0.0001f) {
            const float len = std::sqrt(lenSq);
            outHit->normalX = pushX / len;
            outHit->normalY = pushY / len;
            outHit->overlap = len;
        }
    }
    return hit;
}

bool IsBodyOutOfBounds(const BodyState& body, float x, float y) {
    if (body.desc.allowOutOfBounds) {
        return false;
    }
    return !NavMesh::Instance().IsPointWalkable(
        {static_cast<int>(std::lround(x)), static_cast<int>(std::lround(y))},
        NavMesh::QuantizeClearanceBucket(static_cast<int>(std::ceil(body.desc.radius))));
}

std::vector<int> QueryNearbyCandidateIds(const BodyState& body, float x, float y, float extraRadius) {
    std::vector<int> result;
    std::unordered_set<int> unique;
    const float queryRadius = body.desc.radius + std::max(0.0f, extraRadius);
    const int minCellX = CellX(x - queryRadius);
    const int maxCellX = CellX(x + queryRadius);
    const int minCellY = CellY(y - queryRadius);
    const int maxCellY = CellY(y + queryRadius);

    for (const auto& [layer, grid] : g_registry.gridsByLayer) {
        const unsigned int layerBit = (layer >= 32) ? 0u : (1u << static_cast<unsigned int>(layer));
        if ((body.desc.collisionMask & layerBit) == 0u) {
            continue;
        }
        for (int cy = minCellY; cy <= maxCellY; ++cy) {
            for (int cx = minCellX; cx <= maxCellX; ++cx) {
                const auto cellIt = grid.find(MakeKey(cx, cy));
                if (cellIt == grid.end()) {
                    continue;
                }
                for (int otherId : cellIt->second) {
                    if (otherId != body.entityId) {
                        unique.insert(otherId);
                    }
                }
            }
        }
    }

    result.assign(unique.begin(), unique.end());
    return result;
}

} // namespace

void Clear() {
    g_registry.bodies.clear();
    g_registry.gridsByLayer.clear();
    g_registry.lastReports.clear();
}

void RegisterOrUpdateBody(int entityId,
                          const RigidbodyDesc& desc,
                          float x,
                          float y,
                          bool moving,
                          bool idle) {
    BodyState& body = g_registry.bodies[entityId];

    const bool wasNew      = (body.entityId < 0);
    const bool oldEnabled  = body.desc.enabled;
    const int  oldLayer    = body.desc.layer;
    const float oldX       = body.x;
    const float oldY       = body.y;
    const float oldRadius  = body.desc.radius;

    body.entityId  = entityId;
    body.previousX = body.x;
    body.previousY = body.y;
    body.desc      = desc;
    body.x         = x;
    body.y         = y;
    body.moving    = moving;
    body.idle      = idle;

    const bool gridChanged = wasNew
        || oldEnabled  != desc.enabled
        || oldLayer    != desc.layer
        || oldX        != x
        || oldY        != y
        || oldRadius   != desc.radius;

    if (!gridChanged) return;

    if (oldEnabled && !wasNew) {
        RemoveFromGrid(oldLayer, oldX, oldY, oldRadius, entityId);
    }
    if (desc.enabled) {
        InsertIntoGrid(body);
    }
}

void RemoveBody(int entityId) {
    auto it = g_registry.bodies.find(entityId);
    if (it == g_registry.bodies.end()) return;
    const BodyState& body = it->second;
    if (body.desc.enabled) {
        RemoveFromGrid(body.desc.layer, body.x, body.y, body.desc.radius, entityId);
    }
    g_registry.bodies.erase(it);
    g_registry.lastReports.erase(entityId);
}

bool HasBody(int entityId) {
    return g_registry.bodies.find(entityId) != g_registry.bodies.end();
}

const BodyState* GetBody(int entityId) {
    return FindBody(entityId);
}

void SyncFromEntities(const EntityManager& entityManager,
                      const std::unordered_map<int, RigidbodyDesc>* overrides) {
    std::unordered_set<int> liveIds;
    for (const Entity& entity : entityManager.entities) {
        liveIds.insert(entity.id);

        RigidbodyDesc desc{};
        desc.enabled = !entity.isDead;
        desc.bodyType = BodyType::Circle;
        desc.layer = 0;
        desc.collisionMask = 1u;
        desc.entityCollision = true;
        desc.mapCollision = true;
        desc.allowOutOfBounds = false;
        desc.immovable = false;
        desc.radius = static_cast<float>(entity.radius);

        if (overrides) {
            auto overrideIt = overrides->find(entity.id);
            if (overrideIt != overrides->end()) {
                desc = overrideIt->second;
            }
        }

        RegisterOrUpdateBody(entity.id,
                             desc,
                             entity.position.x,
                             entity.position.y,
                             entity.pathIndex < entity.path.size(),
                             entity.pathIndex >= entity.path.size());
    }

    std::vector<int> toRemove;
    for (const auto& [entityId, body] : g_registry.bodies) {
        (void)body;
        if (liveIds.find(entityId) == liveIds.end()) {
            toRemove.push_back(entityId);
        }
    }
    for (int entityId : toRemove) {
        RemoveBody(entityId);
    }
}

CollisionReport QueryAtPosition(int entityId, float x, float y) {
    CollisionReport report{};
    const BodyState* body = FindBody(entityId);
    if (!body || !body->desc.enabled) {
        return report;
    }

    if (IsBodyOutOfBounds(*body, x, y)) {
        report.outOfBounds = true;
        CollisionHit hit{};
        hit.kind = CollisionKind::OutOfBounds;
        report.hits.push_back(hit);
    }

    CollisionHit mapHit{};
    if (WouldHitMap(*body, x, y, &mapHit)) {
        report.blockedByMap = true;
        report.hits.push_back(mapHit);
    }

    if (body->desc.entityCollision) {
        const std::vector<int> candidates = QueryNearbyCandidateIds(*body, x, y, body->desc.radius);
        for (int otherId : candidates) {
            const BodyState* other = FindBody(otherId);
            if (!other || !other->desc.enabled || !other->desc.entityCollision) {
                continue;
            }
            if (!LayersCanCollide(body->desc, other->desc)) {
                continue;
            }

            CollisionHit hit{};
            if (CheckCircleOverlap(x,
                                   y,
                                   body->desc.radius,
                                   other->x,
                                   other->y,
                                   other->desc.radius,
                                   &hit.normalX,
                                   &hit.normalY,
                                   &hit.overlap)) {
                hit.kind = CollisionKind::Entity;
                hit.otherEntityId = otherId;
                report.blockedByEntity = true;
                report.hits.push_back(hit);
            }
        }
    }

    return report;
}

// Like QueryAtPosition but skips the IsBodyOutOfBounds check.
// Use for per-frame movement: a unit touching a wall mid-slide is legitimately
// near the clearance boundary and must not be treated as out-of-bounds.
CollisionReport QueryAtPositionForMovement(int entityId, float x, float y) {
    CollisionReport report{};
    const BodyState* body = FindBody(entityId);
    if (!body || !body->desc.enabled) {
        return report;
    }

    CollisionHit mapHit{};
    if (WouldHitMap(*body, x, y, &mapHit)) {
        report.blockedByMap = true;
        report.hits.push_back(mapHit);
    }

    if (body->desc.entityCollision) {
        const std::vector<int> candidates = QueryNearbyCandidateIds(*body, x, y, body->desc.radius);
        for (int otherId : candidates) {
            const BodyState* other = FindBody(otherId);
            if (!other || !other->desc.enabled || !other->desc.entityCollision) {
                continue;
            }
            if (!LayersCanCollide(body->desc, other->desc)) {
                continue;
            }

            CollisionHit hit{};
            if (CheckCircleOverlap(x,
                                   y,
                                   body->desc.radius,
                                   other->x,
                                   other->y,
                                   other->desc.radius,
                                   &hit.normalX,
                                   &hit.normalY,
                                   &hit.overlap)) {
                hit.kind = CollisionKind::Entity;
                hit.otherEntityId = otherId;
                report.blockedByEntity = true;
                report.hits.push_back(hit);
            }
        }
    }

    return report;
}

MoveResult TestMove(const MoveQuery& query) {
    MoveResult result{};
    result.acceptedX = query.toX;
    result.acceptedY = query.toY;

    const BodyState* body = FindBody(query.entityId);
    if (!body || !body->desc.enabled) {
        return result;
    }

    result.report = QueryAtPosition(query.entityId, query.toX, query.toY);
    result.blockedByMap = result.report.blockedByMap || result.report.outOfBounds;
    result.blockedByEntity = result.report.blockedByEntity;
    result.blocked = result.blockedByMap || result.blockedByEntity;

    if (result.blocked) {
        result.acceptedX = query.fromX;
        result.acceptedY = query.fromY;
    }

    return result;
}

// TestMove variant for per-frame movement: uses QueryAtPositionForMovement so
// the out-of-bounds check is excluded. Wall proximity still blocks via WouldHitMap
// and provides slide normals; only the NavMesh walkability gate is bypassed.
MoveResult TestMoveForMovement(const MoveQuery& query) {
    MoveResult result{};
    result.acceptedX = query.toX;
    result.acceptedY = query.toY;

    const BodyState* body = FindBody(query.entityId);
    if (!body || !body->desc.enabled) {
        return result;
    }

    result.report = QueryAtPositionForMovement(query.entityId, query.toX, query.toY);
    result.blockedByMap = result.report.blockedByMap;
    result.blockedByEntity = result.report.blockedByEntity;
    result.blocked = result.blockedByMap || result.blockedByEntity;

    if (result.blocked) {
        result.acceptedX = query.fromX;
        result.acceptedY = query.fromY;
    }

    return result;
}

bool WouldCollideWithMap(int entityId, float x, float y) {
    const BodyState* body = FindBody(entityId);
    if (!body || !body->desc.enabled) {
        return false;
    }
    if (IsBodyOutOfBounds(*body, x, y)) {
        return true;
    }
    return WouldHitMap(*body, x, y, nullptr);
}

bool WouldCollideWithEntities(int entityId, float x, float y, CollisionReport* outReport) {
    const BodyState* body = FindBody(entityId);
    if (!body || !body->desc.enabled || !body->desc.entityCollision) {
        if (outReport) {
            *outReport = CollisionReport{};
        }
        return false;
    }

    CollisionReport report = QueryAtPosition(entityId, x, y);
    if (outReport) {
        *outReport = report;
    }
    return report.blockedByEntity;
}

bool IsPointTraversable(int entityId, const Vec2& point, int clearanceBucket) {
    const BodyState* body = FindBody(entityId);
    if (!body || !body->desc.enabled) {
        return false;
    }
    if (!body->desc.allowOutOfBounds && !NavMesh::Instance().IsPointWalkable(point, clearanceBucket)) {
        return false;
    }
    if (body->desc.mapCollision && WouldCollideWithMap(entityId, static_cast<float>(point.x), static_cast<float>(point.y))) {
        return false;
    }
    return true;
}

bool HasTraversalLineOfSight(int entityId,
                             const Vec2& a,
                             const Vec2& b,
                             int clearanceBucket) {
    const BodyState* body = FindBody(entityId);
    if (!body || !body->desc.enabled) {
        return false;
    }
    if (!body->desc.allowOutOfBounds) {
        if (!NavMesh::Instance().IsPointWalkable(a, clearanceBucket) ||
            !NavMesh::Instance().IsPointWalkable(b, clearanceBucket)) {
            return false;
        }
    }
    return NavMesh::Instance().HasLineOfSight(a, b, clearanceBucket);
}

Vec2 ClampPointToTraversal(int entityId, const Vec2& point, int clearanceBucket) {
    const BodyState* body = FindBody(entityId);
    if (!body || !body->desc.enabled || body->desc.allowOutOfBounds) {
        return point;
    }
    return NavMesh::Instance().ClampToNavMesh(point, clearanceBucket);
}

std::vector<int> QueryNearbySameLayer(int entityId, float x, float y, float extraRadius) {
    const BodyState* body = FindBody(entityId);
    if (!body || !body->desc.enabled) {
        return {};
    }
    return QueryNearbyCandidateIds(*body, x, y, extraRadius);
}

void ClearLastReports() {
    g_registry.lastReports.clear();
}

void ClearEntityReport(int entityId) {
    g_registry.lastReports.erase(entityId);
}

void StoreLastReport(int entityId, const CollisionReport& report) {
    g_registry.lastReports[entityId] = report;
}

const CollisionReport* GetLastReport(int entityId) {
    auto it = g_registry.lastReports.find(entityId);
    if (it == g_registry.lastReports.end()) {
        return nullptr;
    }
    return &it->second;
}

} // namespace RigidbodySystem
