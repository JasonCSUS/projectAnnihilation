#ifndef NAVIGATIONLOCALROUTES_H
#define NAVIGATIONLOCALROUTES_H

#include "NavigationRegionGraph.h"
#include "NavigationStateCache.h"

class NavigationLocalRoutes {
public:
    bool BuildShortestAttachmentRoute(int startPoly,
                                      const NavigationAnchor& anchor,
                                      int clearanceBucket,
                                      NavigationSubCacheRoute& outRoute);

    bool BuildRegionStateCache(const NavigationRegionGraph& regionGraph,
                               const std::string& regionLabel,
                               uint64_t stateKey,
                               int clearanceBucket,
                               const std::vector<NavigationAnchor>& exitAnchors,
                               NavigationRegionStateSubCache& outCache);
};

#endif
