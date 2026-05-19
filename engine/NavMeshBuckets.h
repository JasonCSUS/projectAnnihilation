#ifndef NAVMESHBUCKETS_H
#define NAVMESHBUCKETS_H

#include "NavMesh.h"

#include <memory>
#include <vector>

class NavMeshBuckets {
public:
    static const NavMesh::BucketView* GetBucketView(
        const NavMesh& navMesh,
        int clearanceBucket,
        std::shared_ptr<const std::vector<NavMesh::BucketView>>& outSnapshot);

    static void BuildRuntimeStateFromCurrentData(
        const NavMesh& navMesh,
        std::vector<uint8_t>& outPolygonEnabled,
        std::vector<NavWallSegment>& outActiveDynamicWalls,
        std::vector<NavMesh::BucketView>& outBucketViews);

    static void BucketWorkerLoop(NavMesh& navMesh);
};

#endif
