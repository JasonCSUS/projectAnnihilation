#ifndef NAVIGATIONREGIONGRAPH_H
#define NAVIGATIONREGIONGRAPH_H

#include "NavMesh.h"
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

struct NavigationRegionNode {
    std::string label;
    std::string role;
    Vec2 center{};
    std::vector<std::string> neighbors;
    std::vector<int> polygonIds;
};

struct NavigationBlockerInfo {
    std::string label;
    std::string toggleId;
    std::string blockerType;
    std::string owningRegion;
    std::vector<int> blockedPolygons;
};

class NavigationRegionGraph {
public:
    bool LoadFromNavJson(const std::string& navJsonPath);
    void Clear();

    const NavigationRegionNode* FindRegion(const std::string& label) const;
    const NavigationBlockerInfo* FindBlocker(const std::string& toggleId) const;

    const std::vector<NavigationRegionNode>& GetRegions() const { return regions; }
    const std::vector<NavigationBlockerInfo>& GetBlockers() const { return blockers; }

    std::string GetPrimaryRegionForPoly(int polyId) const;
    std::vector<std::string> GetReachableRegions(const std::string& startRegion,
                                                 uint64_t stateKey) const;

    uint64_t ComputeStateKey(const std::unordered_map<std::string, bool>& blockerEnabled) const;
    std::unordered_map<std::string, bool> DecodeStateKey(uint64_t stateKey) const;

    bool IsRoomClearState(const std::string& regionLabel, uint64_t stateKey) const;
    bool IsBlockerOpen(const std::string& toggleId, uint64_t stateKey) const;

private:
    std::vector<std::string> GetFilteredNeighbors(const NavigationRegionNode& region,
                                                  uint64_t stateKey) const;

private:
    std::vector<NavigationRegionNode> regions;
    std::vector<NavigationBlockerInfo> blockers;
    std::unordered_map<std::string, int> regionIndexByLabel;
    std::unordered_map<std::string, int> blockerIndexByToggleId;
    std::unordered_map<int, std::string> polyToPrimaryRegion;
    std::vector<std::string> blockerBitOrder;
    std::unordered_map<std::string, int> blockerBitIndex;
};

#endif
