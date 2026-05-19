#ifndef NAVPRIORDATABASE_H
#define NAVPRIORDATABASE_H

#include <string>
#include <unordered_map>
#include <vector>

struct GoalTransitionPrior {
    int fromPoly = -1;
    int toPoly = -1;
    float bonus = 0.0f;
};

class NavPriorDB {
public:
    static NavPriorDB& Instance();

    bool LoadFromFile(const std::string& filename);
    void Clear();

    float GetTransitionBonus(int goalPoly, int fromPoly, int toPoly, int bucket = 0) const;
    float GetPortalLengthHint(int fromPoly, int toPoly, int bucket = 0) const;
    bool HasAnyPriors() const;

private:
    NavPriorDB() = default;
    NavPriorDB(const NavPriorDB&) = delete;
    NavPriorDB& operator=(const NavPriorDB&) = delete;

    static long long MakeEdgeKey(int fromPoly, int toPoly);

private:
    std::unordered_map<int, std::unordered_map<int, std::unordered_map<long long, float>>> goalTransitionBonusesByBucket;
    std::unordered_map<int, std::unordered_map<long long, float>> portalLengthHintsByBucket;
};

#endif
