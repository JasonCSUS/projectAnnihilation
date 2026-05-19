#include "NavigationWorkers.h"

#include "NavMesh.h"

#include <algorithm>

NavigationWorkers::NavigationWorkers() = default;

NavigationWorkers::~NavigationWorkers() {
    Stop();
}

void NavigationWorkers::Start(int workerCount) {
    std::lock_guard<std::mutex> lock(mutex);
    if (started) {
        return;
    }

    stopFlag = false;
    started = true;

    const int count = std::max(1, workerCount);
    for (int i = 0; i < count; ++i) {
        workers.emplace_back(&NavigationWorkers::WorkerLoop, this);
    }
}

void NavigationWorkers::Stop() {
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (!started) {
            return;
        }
        stopFlag = true;
    }

    cv.notify_all();

    for (auto& worker : workers) {
        if (worker.joinable()) {
            worker.join();
        }
    }

    workers.clear();

    std::lock_guard<std::mutex> lock(mutex);
    requests.clear();
    results.clear();
    pendingEntities.clear();
    stopFlag = false;
    started = false;
}

bool NavigationWorkers::Enqueue(const NavigationWorkRequest& request) {
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (pendingEntities.find(request.entityId) != pendingEntities.end()) {
            return false;
        }

        pendingEntities.insert(request.entityId);
        requests.push_back(request);
    }

    cv.notify_one();
    return true;
}

bool NavigationWorkers::TryPopResult(NavigationWorkResult& outResult) {
    std::lock_guard<std::mutex> lock(mutex);
    if (results.empty()) {
        return false;
    }

    outResult = std::move(results.front());
    results.pop_front();
    return true;
}

bool NavigationWorkers::HasPendingForEntity(int entityId) const {
    std::lock_guard<std::mutex> lock(mutex);
    return pendingEntities.find(entityId) != pendingEntities.end();
}

void NavigationWorkers::WorkerLoop() {
    while (true) {
        NavigationWorkRequest request;

        {
            std::unique_lock<std::mutex> lock(mutex);
            cv.wait(lock, [&]() { return stopFlag || !requests.empty(); });

            if (stopFlag && requests.empty()) {
                return;
            }

            request = requests.front();
            requests.pop_front();
        }

        NavigationWorkResult result;
        result.entityId = request.entityId;
        result.startPoly = request.startPoly;
        result.goalPoly = request.goalPoly;
        result.clearanceBucket = request.clearanceBucket;
        result.startX = request.startX;
        result.startY = request.startY;
        result.goalX = request.goalX;
        result.goalY = request.goalY;
        result.generation = request.generation;

        const Vec2 startPoint{request.startX, request.startY};
        const Vec2 goalPoint{request.goalX, request.goalY};

        result.polyPath = NavMesh::Instance().FindPath(
            request.startPoly,
            request.goalPoly,
            request.clearanceBucket
        );

        if (!result.polyPath.empty()) {
            result.route = NavMesh::Instance().FunnelPath(
                result.polyPath,
                startPoint,
                goalPoint,
                request.clearanceBucket
            );
            result.success = !result.route.empty();
        }

        {
            std::lock_guard<std::mutex> lock(mutex);
            pendingEntities.erase(request.entityId);
            results.push_back(std::move(result));
        }
    }
}
