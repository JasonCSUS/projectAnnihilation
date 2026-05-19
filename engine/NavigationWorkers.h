#ifndef NAVIGATIONWORKERS_H
#define NAVIGATIONWORKERS_H

#include "NavMesh.h"

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <thread>
#include <unordered_set>
#include <vector>

struct NavigationWorkRequest {
    int entityId = -1;
    int startPoly = -1;
    int goalPoly = -1;
    int clearanceBucket = 0;
    int startX = 0;
    int startY = 0;
    int goalX = 0;
    int goalY = 0;
    uint32_t generation = 0;
};

struct NavigationWorkResult {
    int entityId = -1;
    int startPoly = -1;
    int goalPoly = -1;
    int clearanceBucket = 0;
    int startX = 0;
    int startY = 0;
    int goalX = 0;
    int goalY = 0;
    bool success = false;
    std::vector<int> polyPath;
    std::vector<Vec2> route;
    uint32_t generation = 0;
};

class NavigationWorkers {
public:
    NavigationWorkers();
    ~NavigationWorkers();

    void Start(int workerCount);
    void Stop();

    bool Enqueue(const NavigationWorkRequest& request);
    bool TryPopResult(NavigationWorkResult& outResult);

    bool HasPendingForEntity(int entityId) const;

private:
    void WorkerLoop();

private:
    mutable std::mutex mutex;
    std::condition_variable cv;
    std::deque<NavigationWorkRequest> requests;
    std::deque<NavigationWorkResult> results;
    std::unordered_set<int> pendingEntities;
    std::vector<std::thread> workers;
    bool stopFlag = false;
    bool started = false;
};

#endif
