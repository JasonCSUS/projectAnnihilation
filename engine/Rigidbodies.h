#ifndef RIGIDBODIES_H
#define RIGIDBODIES_H

#include <SDL3/SDL.h>

#include <unordered_map>
#include <vector>
#include <cstdint>

#include "EntityManager.h"
#include "NavMesh.h"

namespace RigidbodySystem {

enum class BodyType {
    None,
    Circle
};

enum class CollisionKind {
    None,
    Entity,
    Map,
    OutOfBounds
};

struct RigidbodyDesc {
    bool enabled = true;
    BodyType bodyType = BodyType::Circle;
    int layer = 0;
    unsigned int collisionMask = 1u;
    bool entityCollision = true;
    bool mapCollision = true;
    bool allowOutOfBounds = false;
    bool immovable = false;
    float radius = 0.0f;
};

struct BodyState {
    int entityId = -1;
    RigidbodyDesc desc{};
    float x = 0.0f;
    float y = 0.0f;
    float previousX = 0.0f;
    float previousY = 0.0f;
    bool moving = false;
    bool idle = true;
};

struct CollisionHit {
    CollisionKind kind = CollisionKind::None;
    int otherEntityId = -1;
    float normalX = 0.0f;
    float normalY = 0.0f;
    float overlap = 0.0f;
};

struct CollisionReport {
    bool blockedByMap = false;
    bool blockedByEntity = false;
    bool outOfBounds = false;
    std::vector<CollisionHit> hits;
};

struct MoveQuery {
    int entityId = -1;
    float fromX = 0.0f;
    float fromY = 0.0f;
    float toX = 0.0f;
    float toY = 0.0f;
};

struct MoveResult {
    float acceptedX = 0.0f;
    float acceptedY = 0.0f;
    bool blocked = false;
    bool blockedByMap = false;
    bool blockedByEntity = false;
    CollisionReport report{};
};

void Clear();

void RegisterOrUpdateBody(int entityId,
                          const RigidbodyDesc& desc,
                          float x,
                          float y,
                          bool moving,
                          bool idle);

void RemoveBody(int entityId);

bool HasBody(int entityId);
const BodyState* GetBody(int entityId);

void SyncFromEntities(const EntityManager& entityManager,
                      const std::unordered_map<int, RigidbodyDesc>* overrides = nullptr);

CollisionReport QueryAtPosition(int entityId, float x, float y);
CollisionReport QueryAtPositionForMovement(int entityId, float x, float y);
MoveResult TestMove(const MoveQuery& query);
MoveResult TestMoveForMovement(const MoveQuery& query);

bool WouldCollideWithMap(int entityId, float x, float y);
bool WouldCollideWithEntities(int entityId, float x, float y, CollisionReport* outReport = nullptr);

bool IsPointTraversable(int entityId, const Vec2& point, int clearanceBucket);
bool HasTraversalLineOfSight(int entityId,
                             const Vec2& a,
                             const Vec2& b,
                             int clearanceBucket);
Vec2 ClampPointToTraversal(int entityId, const Vec2& point, int clearanceBucket);

std::vector<int> QueryNearbySameLayer(int entityId, float x, float y, float extraRadius = 0.0f);

void ClearLastReports();
void ClearEntityReport(int entityId);
void StoreLastReport(int entityId, const CollisionReport& report);
const CollisionReport* GetLastReport(int entityId);

} // namespace RigidbodySystem

#endif
