#include "NavigationRegionGraph.h"
#include <fstream>
#include <queue>
#include <regex>

namespace {
bool LoadTextFile(const std::string& path, std::string& outText) {
    std::ifstream in(path);
    if (!in) return false;
    outText.assign((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    return true;
}

bool ExtractSectionArray(const std::string& json, const std::string& key, std::string& outArrayBody) {
    const std::string token = "\""+key+"\"";
    const size_t keyPos = json.find(token);
    if (keyPos == std::string::npos) return false;
    const size_t open = json.find('[', keyPos);
    if (open == std::string::npos) return false;
    int depth = 0;
    for (size_t i = open; i < json.size(); ++i) {
        if (json[i] == '[') depth++;
        else if (json[i] == ']') {
            depth--;
            if (depth == 0) {
                outArrayBody = json.substr(open + 1, i - open - 1);
                return true;
            }
        }
    }
    return false;
}

std::vector<std::string> SplitTopLevelObjects(const std::string& arrayBody) {
    std::vector<std::string> objects;
    int depth = 0;
    size_t start = std::string::npos;
    for (size_t i = 0; i < arrayBody.size(); ++i) {
        if (arrayBody[i] == '{') {
            if (depth == 0) start = i;
            depth++;
        } else if (arrayBody[i] == '}') {
            depth--;
            if (depth == 0 && start != std::string::npos) {
                objects.push_back(arrayBody.substr(start, i - start + 1));
                start = std::string::npos;
            }
        }
    }
    return objects;
}

bool ExtractStringField(const std::string& text, const std::string& key, std::string& out) {
    const std::regex re("\\\""+key+"\\\"\\s*:\\s*\\\"([^\\\"]*)\\\"");
    std::smatch m;
    if (std::regex_search(text, m, re)) { out = m[1].str(); return true; }
    return false;
}

bool ExtractStringArrayField(const std::string& text, const std::string& key, std::vector<std::string>& out) {
    const std::string token = "\""+key+"\"";
    const size_t keyPos = text.find(token);
    if (keyPos == std::string::npos) return false;
    const size_t open = text.find('[', keyPos);
    if (open == std::string::npos) return false;
    int depth = 0;
    size_t close = std::string::npos;
    for (size_t i = open; i < text.size(); ++i) {
        if (text[i] == '[') depth++;
        else if (text[i] == ']') {
            depth--;
            if (depth == 0) { close = i; break; }
        }
    }
    if (close == std::string::npos) return false;
    out.clear();
    const std::string body = text.substr(open + 1, close - open - 1);
    const std::regex re("\\\"([^\\\"]+)\\\"");
    for (auto it = std::sregex_iterator(body.begin(), body.end(), re); it != std::sregex_iterator(); ++it) {
        out.push_back((*it)[1].str());
    }
    return true;
}

bool ExtractIntArrayField(const std::string& text, const std::string& key, std::vector<int>& out) {
    const std::string token = "\""+key+"\"";
    const size_t keyPos = text.find(token);
    if (keyPos == std::string::npos) return false;
    const size_t open = text.find('[', keyPos);
    if (open == std::string::npos) return false;
    int depth = 0;
    size_t close = std::string::npos;
    for (size_t i = open; i < text.size(); ++i) {
        if (text[i] == '[') depth++;
        else if (text[i] == ']') {
            depth--;
            if (depth == 0) { close = i; break; }
        }
    }
    if (close == std::string::npos) return false;
    out.clear();
    const std::string body = text.substr(open + 1, close - open - 1);
    const std::regex re("(-?\\d+)");
    for (auto it = std::sregex_iterator(body.begin(), body.end(), re); it != std::sregex_iterator(); ++it) {
        out.push_back(std::stoi((*it)[1].str()));
    }
    return true;
}

bool ExtractCenterField(const std::string& text, Vec2& out) {
    const std::regex re("\\\"center\\\"\\s*:\\s*\\[\\s*(-?\\d+)\\s*,\\s*(-?\\d+)\\s*\\]");
    std::smatch m;
    if (!std::regex_search(text, m, re)) return false;
    out.x = std::stoi(m[1].str());
    out.y = std::stoi(m[2].str());
    return true;
}
}

void NavigationRegionGraph::Clear() {
    regions.clear(); blockers.clear(); regionIndexByLabel.clear(); blockerIndexByToggleId.clear();
    polyToPrimaryRegion.clear(); blockerBitOrder.clear(); blockerBitIndex.clear();
}

bool NavigationRegionGraph::LoadFromNavJson(const std::string& navJsonPath) {
    Clear();
    std::string json;
    if (!LoadTextFile(navJsonPath, json)) return false;

    std::string regionsBody;
    if (ExtractSectionArray(json, "regions", regionsBody)) {
        for (const std::string& obj : SplitTopLevelObjects(regionsBody)) {
            NavigationRegionNode region;
            if (!ExtractStringField(obj, "label", region.label) || region.label.empty()) continue;
            ExtractStringField(obj, "role", region.role);
            ExtractCenterField(obj, region.center);
            ExtractStringArrayField(obj, "neighbors", region.neighbors);
            ExtractIntArrayField(obj, "polygon_ids", region.polygonIds);
            regionIndexByLabel[region.label] = static_cast<int>(regions.size());
            for (int polyId : region.polygonIds) {
                if (!polyToPrimaryRegion.count(polyId)) polyToPrimaryRegion[polyId] = region.label;
            }
            regions.push_back(std::move(region));
        }
    }

    std::string blockersBody;
    if (ExtractSectionArray(json, "runtime_blockers", blockersBody)) {
        for (const std::string& obj : SplitTopLevelObjects(blockersBody)) {
            NavigationBlockerInfo blocker;
            ExtractStringField(obj, "label", blocker.label);
            ExtractStringField(obj, "toggle_id", blocker.toggleId);
            ExtractStringField(obj, "blocker_type", blocker.blockerType);
            ExtractStringField(obj, "owning_region", blocker.owningRegion);
            ExtractIntArrayField(obj, "cell_ids", blocker.blockedPolygons);
            if (blocker.toggleId.empty()) blocker.toggleId = blocker.label;
            if (blocker.blockerType.empty()) blocker.blockerType = "unknown";
            blockerIndexByToggleId[blocker.toggleId] = static_cast<int>(blockers.size());
            blockerBitIndex[blocker.toggleId] = static_cast<int>(blockerBitOrder.size());
            blockerBitOrder.push_back(blocker.toggleId);
            blockers.push_back(std::move(blocker));
        }
    }

    std::string cellsBody;
    if (ExtractSectionArray(json, "cells", cellsBody)) {
        int polyId = 0;
        for (const std::string& obj : SplitTopLevelObjects(cellsBody)) {
            std::vector<std::string> labels;
            if (ExtractStringArrayField(obj, "source_region_labels", labels) && !labels.empty()) {
                polyToPrimaryRegion[polyId] = labels.front();
            }
            ++polyId;
        }
    }

    return !regions.empty();
}

const NavigationRegionNode* NavigationRegionGraph::FindRegion(const std::string& label) const {
    auto it = regionIndexByLabel.find(label);
    if (it == regionIndexByLabel.end()) return nullptr;
    return &regions[it->second];
}

const NavigationBlockerInfo* NavigationRegionGraph::FindBlocker(const std::string& toggleId) const {
    auto it = blockerIndexByToggleId.find(toggleId);
    if (it == blockerIndexByToggleId.end()) return nullptr;
    return &blockers[it->second];
}

std::string NavigationRegionGraph::GetPrimaryRegionForPoly(int polyId) const {
    auto it = polyToPrimaryRegion.find(polyId);
    if (it == polyToPrimaryRegion.end()) return {};
    return it->second;
}

uint64_t NavigationRegionGraph::ComputeStateKey(const std::unordered_map<std::string, bool>& blockerEnabled) const {
    uint64_t key = 0;
    for (const auto& toggleId : blockerBitOrder) {
        auto bitIt = blockerBitIndex.find(toggleId);
        auto stateIt = blockerEnabled.find(toggleId);
        const bool enabled = stateIt != blockerEnabled.end() ? stateIt->second : true;
        if (enabled) key |= (uint64_t{1} << bitIt->second);
    }
    return key;
}

std::unordered_map<std::string, bool> NavigationRegionGraph::DecodeStateKey(uint64_t stateKey) const {
    std::unordered_map<std::string, bool> out;
    for (const auto& toggleId : blockerBitOrder) {
        const int bit = blockerBitIndex.at(toggleId);
        out[toggleId] = ((stateKey >> bit) & 1ULL) != 0ULL;
    }
    return out;
}

bool NavigationRegionGraph::IsRoomClearState(const std::string& regionLabel, uint64_t stateKey) const {
    for (const auto& blocker : blockers) {
        if (blocker.owningRegion != regionLabel) continue;
        if (blocker.blockerType != "spawner") continue;
        if (!IsBlockerOpen(blocker.toggleId, stateKey)) return false;
    }
    return true;
}

bool NavigationRegionGraph::IsBlockerOpen(const std::string& toggleId, uint64_t stateKey) const {
    auto it = blockerBitIndex.find(toggleId);
    if (it == blockerBitIndex.end()) return true;
    const bool enabled = ((stateKey >> it->second) & 1ULL) != 0ULL;
    return !enabled;
}

std::vector<std::string> NavigationRegionGraph::GetFilteredNeighbors(const NavigationRegionNode& region,
                                                                     uint64_t stateKey) const {
    std::vector<std::string> out;
    for (const std::string& neighbor : region.neighbors) {
        bool blocked = false;
        for (const auto& blocker : blockers) {
            if (blocker.blockerType != "blockade") continue;
            if (blocker.owningRegion != region.label && blocker.owningRegion != neighbor) continue;
            if (!IsBlockerOpen(blocker.toggleId, stateKey)) { blocked = true; break; }
        }
        if (!blocked) out.push_back(neighbor);
    }
    return out;
}

std::vector<std::string> NavigationRegionGraph::GetReachableRegions(const std::string& startRegion,
                                                                    uint64_t stateKey) const {
    std::vector<std::string> result;
    const NavigationRegionNode* start = FindRegion(startRegion);
    if (!start) return result;

    std::queue<std::string> q;
    std::unordered_map<std::string, bool> visited;
    q.push(startRegion);
    visited[startRegion] = true;

    while (!q.empty()) {
        std::string current = q.front();
        q.pop();
        result.push_back(current);
        const NavigationRegionNode* region = FindRegion(current);
        if (!region) continue;

        for (const std::string& next : GetFilteredNeighbors(*region, stateKey)) {
            if (visited[next]) continue;
            visited[next] = true;
            q.push(next);
        }
    }
    return result;
}
