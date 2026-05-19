#ifndef NAVIGATIONTRAINING_H
#define NAVIGATIONTRAINING_H

#include "NavigationLocalRoutes.h"
#include "NavigationMacroRoutes.h"
#include "NavigationRegionGraph.h"
#include "NavigationStateCache.h"
#include <string>
#include <vector>

class NavigationTraining {
public:
    bool BuildStartupCaches(const std::string& navJsonPath,
                            const std::string& targetRegionLabel,
                            const std::vector<int>& bucketRadii,
                            const std::string& cacheFilePath);

    bool LoadStartupCaches(const std::string& navJsonPath,
                           const std::string& targetRegionLabel,
                           const std::vector<int>& bucketRadii,
                           const std::string& cacheFilePath);

    bool EnsureStartupCaches(const std::string& navJsonPath,
                             const std::string& targetRegionLabel,
                             const std::vector<int>& bucketRadii,
                             const std::string& cacheFilePath);

    const NavigationRegionGraph& GetRegionGraph() const { return regionGraph; }
    const NavigationMacroRoutes& GetMacroRoutes() const { return macroRoutes; }
    const NavigationStateCache& GetStateCache() const { return stateCache; }

private:
    void EnumerateRegionStateKeys(const std::string& regionLabel,
                                  std::vector<uint64_t>& outStateKeys) const;
    void BuildExitAnchorsForRegion(const std::string& regionLabel,
                                   int clearanceBucket,
                                   std::vector<NavigationAnchor>& outAnchors) const;

private:
    NavigationRegionGraph regionGraph;
    NavigationMacroRoutes macroRoutes;
    NavigationLocalRoutes localRoutes;
    NavigationStateCache stateCache;
};

#endif
