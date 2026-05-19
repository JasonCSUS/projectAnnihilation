#include "NavPriorDB.h"

#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

namespace {
    bool ExtractSectionArray(const std::string& json, const std::string& key, std::string& outArrayBody) {
        const std::string token = "\"" + key + "\"";
        const size_t keyPos = json.find(token);
        if (keyPos == std::string::npos) return false;

        const size_t bracketStart = json.find('[', keyPos);
        if (bracketStart == std::string::npos) return false;

        int depth = 0;
        for (size_t i = bracketStart; i < json.size(); ++i) {
            if (json[i] == '[') depth++;
            else if (json[i] == ']') {
                depth--;
                if (depth == 0) {
                    outArrayBody = json.substr(bracketStart + 1, i - bracketStart - 1);
                    return true;
                }
            }
        }
        return false;
    }

    std::vector<std::string> SplitTopLevelObjects(const std::string& arrayBody) {
        std::vector<std::string> objects;
        int braceDepth = 0;
        size_t objStart = std::string::npos;

        for (size_t i = 0; i < arrayBody.size(); ++i) {
            if (arrayBody[i] == '{') {
                if (braceDepth == 0) objStart = i;
                braceDepth++;
            } else if (arrayBody[i] == '}') {
                braceDepth--;
                if (braceDepth == 0 && objStart != std::string::npos) {
                    objects.push_back(arrayBody.substr(objStart, i - objStart + 1));
                    objStart = std::string::npos;
                }
            }
        }

        return objects;
    }

    bool ExtractInt(const std::string& objectText, const std::string& key, int& outValue) {
        const std::string token = "\"" + key + "\"";
        const size_t keyPos = objectText.find(token);
        if (keyPos == std::string::npos) return false;
        const size_t colon = objectText.find(':', keyPos);
        if (colon == std::string::npos) return false;

        size_t start = colon + 1;
        while (start < objectText.size() && (objectText[start] == ' ' || objectText[start] == '\n' || objectText[start] == '\r' || objectText[start] == '\t')) {
            ++start;
        }
        size_t end = start;
        while (end < objectText.size() && (objectText[end] == '-' || (objectText[end] >= '0' && objectText[end] <= '9'))) {
            ++end;
        }
        if (end == start) return false;

        outValue = std::stoi(objectText.substr(start, end - start));
        return true;
    }

    bool ExtractFloat(const std::string& objectText, const std::string& key, float& outValue) {
        const std::string token = "\"" + key + "\"";
        const size_t keyPos = objectText.find(token);
        if (keyPos == std::string::npos) return false;
        const size_t colon = objectText.find(':', keyPos);
        if (colon == std::string::npos) return false;

        size_t start = colon + 1;
        while (start < objectText.size() && (objectText[start] == ' ' || objectText[start] == '\n' || objectText[start] == '\r' || objectText[start] == '\t')) {
            ++start;
        }
        size_t end = start;
        while (end < objectText.size() && (objectText[end] == '-' || objectText[end] == '+' || objectText[end] == '.' || (objectText[end] >= '0' && objectText[end] <= '9') || objectText[end] == 'e' || objectText[end] == 'E')) {
            ++end;
        }
        if (end == start) return false;

        outValue = std::stof(objectText.substr(start, end - start));
        return true;
    }
}

NavPriorDB& NavPriorDB::Instance() {
    static NavPriorDB instance;
    return instance;
}

long long NavPriorDB::MakeEdgeKey(int fromPoly, int toPoly) {
    return (static_cast<long long>(fromPoly) << 32) ^ static_cast<unsigned int>(toPoly);
}

void NavPriorDB::Clear() {
    goalTransitionBonusesByBucket.clear();
    portalLengthHintsByBucket.clear();
}

bool NavPriorDB::LoadFromFile(const std::string& filename) {
    Clear();

    std::ifstream in(filename);
    if (!in) {
        std::cerr << "NavPriorDB: failed to open prior file: " << filename << "\n";
        return false;
    }

    std::string json((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());

    std::string metricsBody;
    if (ExtractSectionArray(json, "edge_metrics", metricsBody)) {
        const std::vector<std::string> metricObjects = SplitTopLevelObjects(metricsBody);
        for (const std::string& obj : metricObjects) {
            int bucket = 0;
            int from = -1;
            int to = -1;
            float portalLen = 0.0f;
            ExtractInt(obj, "bucket", bucket);
            if (!ExtractInt(obj, "from", from)) continue;
            if (!ExtractInt(obj, "to", to)) continue;
            ExtractFloat(obj, "portal_len", portalLen);
            portalLengthHintsByBucket[bucket][MakeEdgeKey(from, to)] = portalLen;
        }
    }

    std::string goalsBody;
    if (!ExtractSectionArray(json, "goal_priors", goalsBody)) {
        return true;
    }

    const std::vector<std::string> goalObjects = SplitTopLevelObjects(goalsBody);
    for (const std::string& goalObj : goalObjects) {
        int bucket = 0;
        int goalPoly = -1;
        ExtractInt(goalObj, "bucket", bucket);
        if (!ExtractInt(goalObj, "goal_poly", goalPoly)) {
            continue;
        }

        std::string transitionsBody;
        if (!ExtractSectionArray(goalObj, "transitions", transitionsBody)) {
            continue;
        }

        const std::vector<std::string> transitionObjects = SplitTopLevelObjects(transitionsBody);
        for (const std::string& transObj : transitionObjects) {
            int from = -1;
            int to = -1;
            float bonus = 0.0f;
            if (!ExtractInt(transObj, "from", from)) continue;
            if (!ExtractInt(transObj, "to", to)) continue;
            if (!ExtractFloat(transObj, "bonus", bonus)) continue;

            goalTransitionBonusesByBucket[bucket][goalPoly][MakeEdgeKey(from, to)] = bonus;
        }
    }

    return true;
}

float NavPriorDB::GetTransitionBonus(int goalPoly, int fromPoly, int toPoly, int bucket) const {
    const long long edgeKey = MakeEdgeKey(fromPoly, toPoly);

    auto bucketIt = goalTransitionBonusesByBucket.find(bucket);
    if (bucketIt != goalTransitionBonusesByBucket.end()) {
        auto goalIt = bucketIt->second.find(goalPoly);
        if (goalIt != bucketIt->second.end()) {
            auto edgeIt = goalIt->second.find(edgeKey);
            if (edgeIt != goalIt->second.end()) {
                return edgeIt->second;
            }
        }
    }

    bucketIt = goalTransitionBonusesByBucket.find(0);
    if (bucketIt != goalTransitionBonusesByBucket.end()) {
        auto goalIt = bucketIt->second.find(goalPoly);
        if (goalIt != bucketIt->second.end()) {
            auto edgeIt = goalIt->second.find(edgeKey);
            if (edgeIt != goalIt->second.end()) {
                return edgeIt->second;
            }
        }
    }

    return 0.0f;
}

float NavPriorDB::GetPortalLengthHint(int fromPoly, int toPoly, int bucket) const {
    const long long edgeKey = MakeEdgeKey(fromPoly, toPoly);

    auto bucketIt = portalLengthHintsByBucket.find(bucket);
    if (bucketIt != portalLengthHintsByBucket.end()) {
        auto it = bucketIt->second.find(edgeKey);
        if (it != bucketIt->second.end()) {
            return it->second;
        }
    }

    bucketIt = portalLengthHintsByBucket.find(0);
    if (bucketIt != portalLengthHintsByBucket.end()) {
        auto it = bucketIt->second.find(edgeKey);
        if (it != bucketIt->second.end()) {
            return it->second;
        }
    }

    return 0.0f;
}

bool NavPriorDB::HasAnyPriors() const {
    for (const auto& [bucket, byGoal] : goalTransitionBonusesByBucket) {
        (void)bucket;
        if (!byGoal.empty()) {
            return true;
        }
    }
    return false;
}
