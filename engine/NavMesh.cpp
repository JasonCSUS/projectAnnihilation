#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "NavMesh.h"
#include <fstream>
#include <iostream>
#include <queue>
#include <cmath>
#include <limits>
#include <algorithm>
#include <SDL3/SDL.h>

//------------------------------------------------------------
// Basic Utility Functions
//------------------------------------------------------------

static bool PointInPolygon(const Vec2& p, const std::vector<Vec2>& vertices) {
    bool inside = false;
    const int n = static_cast<int>(vertices.size());

    for (int i = 0, j = n - 1; i < n; j = i++) {
        if (((vertices[i].y > p.y) != (vertices[j].y > p.y)) &&
            (p.x < (vertices[j].x - vertices[i].x) * (p.y - vertices[i].y) /
                   static_cast<double>(vertices[j].y - vertices[i].y) + vertices[i].x)) {
            inside = !inside;
        }
    }

    return inside;
}

static Vec2 ComputeCentroid(const std::vector<Vec2>& vertices) {
    long long sumX = 0;
    long long sumY = 0;

    for (const auto& v : vertices) {
        sumX += v.x;
        sumY += v.y;
    }

    const int count = static_cast<int>(vertices.size());
    if (count == 0) {
        return {0, 0};
    }

    return {
        static_cast<int>(sumX / count),
        static_cast<int>(sumY / count)
    };
}

static double Distance(const Vec2& a, const Vec2& b) {
    const int dx = a.x - b.x;
    const int dy = a.y - b.y;
    return std::sqrt(static_cast<double>(dx * dx + dy * dy));
}

static double DistanceSquared(const Vec2& a, const Vec2& b) {
    const double dx = static_cast<double>(a.x - b.x);
    const double dy = static_cast<double>(a.y - b.y);
    return dx * dx + dy * dy;
}

static int TriArea2(const Vec2& a, const Vec2& b, const Vec2& c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

static Vec2 ClampPointToPolygon(const Vec2& pt, const std::vector<Vec2>& poly) {
    double bestDist = std::numeric_limits<double>::max();
    Vec2 bestPt = pt;

    const size_t n = poly.size();
    for (size_t i = 0; i < n; ++i) {
        const Vec2 A = poly[i];
        const Vec2 B = poly[(i + 1) % n];

        const int abx = B.x - A.x;
        const int aby = B.y - A.y;
        const double abSq = static_cast<double>(abx * abx + aby * aby);
        if (abSq == 0.0) {
            continue;
        }

        double t = ((pt.x - A.x) * abx + (pt.y - A.y) * aby) / abSq;
        t = std::max(0.0, std::min(1.0, t));

        const Vec2 proj = {
            A.x + static_cast<int>(std::lround(abx * t)),
            A.y + static_cast<int>(std::lround(aby * t))
        };

        const double d = DistanceSquared(pt, proj);
        if (d < bestDist) {
            bestDist = d;
            bestPt = proj;
        }
    }

    return bestPt;
}

static bool PointInCorridor(const Vec2& p,
                            const std::vector<int>& polyPath,
                            const std::vector<NavPolygon>& polygons) {
    for (int polyIndex : polyPath) {
        if (polyIndex >= 0 &&
            polyIndex < static_cast<int>(polygons.size()) &&
            PointInPolygon(p, polygons[polyIndex].vertices)) {
            return true;
        }
    }
    return false;
}

static bool IsSegmentInsideCorridorSampled(const Vec2& a,
                                           const Vec2& b,
                                           const std::vector<int>& polyPath,
                                           const std::vector<NavPolygon>& polygons) {
    if (!PointInCorridor(a, polyPath, polygons) || !PointInCorridor(b, polyPath, polygons)) {
        return false;
    }

    const double dx = static_cast<double>(b.x - a.x);
    const double dy = static_cast<double>(b.y - a.y);
    const double length = std::sqrt(dx * dx + dy * dy);

    if (length <= 1.0) {
        return true;
    }

    const double stepSize = 8.0;
    const int steps = std::max(1, static_cast<int>(std::ceil(length / stepSize)));

    for (int i = 1; i < steps; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(steps);
        const Vec2 sample = {
            static_cast<int>(std::lround(a.x + dx * t)),
            static_cast<int>(std::lround(a.y + dy * t))
        };

        if (!PointInCorridor(sample, polyPath, polygons)) {
            return false;
        }
    }

    return true;
}

static bool IsPathInsideCorridor(const std::vector<Vec2>& path,
                                 const std::vector<int>& polyPath,
                                 const std::vector<NavPolygon>& polygons) {
    if (path.empty()) {
        return false;
    }

    if (path.size() == 1) {
        return PointInCorridor(path[0], polyPath, polygons);
    }

    for (size_t i = 0; i + 1 < path.size(); ++i) {
        if (!IsSegmentInsideCorridorSampled(path[i], path[i + 1], polyPath, polygons)) {
            return false;
        }
    }

    return true;
}

static Vec2 ClosestPointOnSegmentToPoint(const Vec2& segA, const Vec2& segB, const Vec2& p) {
    const double abx = static_cast<double>(segB.x - segA.x);
    const double aby = static_cast<double>(segB.y - segA.y);
    const double abSq = abx * abx + aby * aby;

    if (abSq <= 1e-9) {
        return segA;
    }

    const double apx = static_cast<double>(p.x - segA.x);
    const double apy = static_cast<double>(p.y - segA.y);
    double t = (apx * abx + apy * aby) / abSq;
    t = std::max(0.0, std::min(1.0, t));

    return {
        static_cast<int>(std::lround(segA.x + abx * t)),
        static_cast<int>(std::lround(segA.y + aby * t))
    };
}

static std::vector<Vec2> BuildSafePortalPath(const std::vector<Portal>& portals) {
    std::vector<Vec2> safePath;
    if (portals.empty()) {
        return safePath;
    }

    safePath.reserve(portals.size() + 1);
    safePath.push_back(portals.front().left);

    for (size_t i = 1; i + 1 < portals.size(); ++i) {
        const Portal& p = portals[i];
        const Vec2 midpoint = {
            (p.left.x + p.right.x) / 2,
            (p.left.y + p.right.y) / 2
        };

        if (safePath.empty() ||
            safePath.back().x != midpoint.x ||
            safePath.back().y != midpoint.y) {
            safePath.push_back(midpoint);
        }
    }

    if (safePath.empty() ||
        safePath.back().x != portals.back().left.x ||
        safePath.back().y != portals.back().left.y) {
        safePath.push_back(portals.back().left);
    }

    return safePath;
}

static std::vector<Portal> BuildPortals(const std::vector<int>& polyPath,
                                        const std::vector<NavPolygon>& polygons,
                                        const Vec2& start,
                                        const Vec2& goal) {
    std::vector<Portal> portals;
    if (polyPath.empty()) {
        return portals;
    }

    portals.reserve(polyPath.size() + 1);
    portals.push_back({start, start});

    for (size_t i = 0; i + 1 < polyPath.size(); ++i) {
        const int currIdx = polyPath[i];
        const int nextIdx = polyPath[i + 1];

        if (currIdx < 0 || nextIdx < 0 ||
            currIdx >= static_cast<int>(polygons.size()) ||
            nextIdx >= static_cast<int>(polygons.size())) {
            continue;
        }

        const NavPolygon& A = polygons[currIdx];
        const NavPolygon& B = polygons[nextIdx];

        bool found = false;
        Vec2 sharedA{};
        Vec2 sharedB{};

        for (size_t j = 0; j < A.vertices.size(); ++j) {
            const Vec2 a1 = A.vertices[j];
            const Vec2 a2 = A.vertices[(j + 1) % A.vertices.size()];

            bool inB1 = false;
            bool inB2 = false;

            for (const auto& v : B.vertices) {
                if (v.x == a1.x && v.y == a1.y) inB1 = true;
                if (v.x == a2.x && v.y == a2.y) inB2 = true;
            }

            if (inB1 && inB2) {
                sharedA = a1;
                sharedB = a2;
                found = true;
                break;
            }
        }

        if (!found) {
            continue;
        }

        const Vec2 currCenter = ComputeCentroid(A.vertices);
        const Vec2 nextCenter = ComputeCentroid(B.vertices);

        if (TriArea2(currCenter, nextCenter, sharedA) > 0) {
            portals.push_back({sharedB, sharedA});
        } else {
            portals.push_back({sharedA, sharedB});
        }
    }

    portals.push_back({goal, goal});
    return portals;
}

static std::vector<Vec2> BuildProjectedPortalPath(const std::vector<Portal>& portals,
                                                  const Vec2& goal) {
    std::vector<Vec2> path;
    if (portals.empty()) {
        return path;
    }

    path.reserve(portals.size() + 1);
    path.push_back(portals.front().left); // start

    // For each real portal, choose the point on that portal segment
    // that is closest to the final goal.
    for (size_t i = 1; i + 1 < portals.size(); ++i) {
        const Portal& p = portals[i];
        const Vec2 projected = ClosestPointOnSegmentToPoint(p.left, p.right, goal);

        if (path.empty() ||
            path.back().x != projected.x ||
            path.back().y != projected.y) {
            path.push_back(projected);
        }
    }

    if (path.empty() ||
        path.back().x != portals.back().left.x ||
        path.back().y != portals.back().left.y) {
        path.push_back(portals.back().left); // goal
    }

    return path;
}

//------------------------------------------------------------
// NavMesh Public Functions
//------------------------------------------------------------

NavMesh& NavMesh::Instance() {
    static NavMesh instance;
    return instance;
}

bool NavMesh::LoadFromFile(const std::string& filename) {
    Clear();

    std::ifstream file(filename, std::ios::binary);
    if (!file) {
        std::cerr << "Error opening navmesh file: " << filename << "\n";
        return false;
    }

    int numPolygons = 0;
    file.read(reinterpret_cast<char*>(&numPolygons), sizeof(int));
    if (numPolygons <= 0) {
        std::cerr << "Invalid polygon count in navmesh file.\n";
        return false;
    }

    polygons.resize(numPolygons);

    for (int i = 0; i < numPolygons; ++i) {
        int vertexCount = 0;
        file.read(reinterpret_cast<char*>(&vertexCount), sizeof(int));
        if (vertexCount < 3) {
            std::cerr << "Polygon " << i << " has fewer than 3 vertices.\n";
            return false;
        }

        NavPolygon poly;
        poly.vertices.resize(vertexCount);

        for (int j = 0; j < vertexCount; ++j) {
            int x = 0;
            int y = 0;
            file.read(reinterpret_cast<char*>(&x), sizeof(int));
            file.read(reinterpret_cast<char*>(&y), sizeof(int));
            poly.vertices[j] = {x, y};
        }

        int neighborCount = 0;
        file.read(reinterpret_cast<char*>(&neighborCount), sizeof(int));
        poly.neighborIndices.resize(neighborCount);

        for (int j = 0; j < neighborCount; ++j) {
            file.read(reinterpret_cast<char*>(&poly.neighborIndices[j]), sizeof(int));
        }

        polygons[i] = std::move(poly);
    }

    std::cout << "Loaded navmesh with " << numPolygons << " polygons.\n";
    return true;
}

void NavMesh::Clear() {
    polygons.clear();
}

void NavMesh::DebugRender(SDL_Renderer* renderer, float cameraX, float cameraY) const {
    SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);

    for (const auto& poly : polygons) {
        const size_t numVerts = poly.vertices.size();
        for (size_t i = 0; i < numVerts; ++i) {
            const Vec2& a = poly.vertices[i];
            const Vec2& b = poly.vertices[(i + 1) % numVerts];
            SDL_RenderLine(renderer,
                           static_cast<float>(a.x - cameraX),
                           static_cast<float>(a.y - cameraY),
                           static_cast<float>(b.x - cameraX),
                           static_cast<float>(b.y - cameraY));
        }
    }
}

std::vector<int> NavMesh::FindPath(int startIndex, int goalIndex) {
    std::vector<int> result;

    if (startIndex < 0 || goalIndex < 0 ||
        startIndex >= static_cast<int>(polygons.size()) ||
        goalIndex >= static_cast<int>(polygons.size())) {
        return result;
    }

    if (startIndex == goalIndex) {
        result.push_back(startIndex);
        return result;
    }

    struct Node {
        int polyIndex;
        double cost;
        double priority;
    };

    auto cmp = [](const Node& a, const Node& b) {
        return a.priority > b.priority;
    };

    std::priority_queue<Node, std::vector<Node>, decltype(cmp)> frontier(cmp);
    std::vector<double> costSoFar(polygons.size(), std::numeric_limits<double>::max());
    std::vector<int> cameFrom(polygons.size(), -1);
    std::vector<Vec2> centroids(polygons.size());

    for (size_t i = 0; i < polygons.size(); ++i) {
        centroids[i] = ComputeCentroid(polygons[i].vertices);
    }

    auto heuristic = [&](int idx) -> double {
        return Distance(centroids[idx], centroids[goalIndex]);
    };

    frontier.push({startIndex, 0.0, heuristic(startIndex)});
    costSoFar[startIndex] = 0.0;

    while (!frontier.empty()) {
        const Node current = frontier.top();
        frontier.pop();

        if (current.cost > costSoFar[current.polyIndex]) {
            continue;
        }

        if (current.polyIndex == goalIndex) {
            int cur = goalIndex;
            while (cur != -1) {
                result.push_back(cur);
                cur = cameFrom[cur];
            }
            std::reverse(result.begin(), result.end());
            return result;
        }

        for (int neighbor : polygons[current.polyIndex].neighborIndices) {
            if (neighbor < 0 || neighbor >= static_cast<int>(polygons.size())) {
                continue;
            }

            const double newCost =
                costSoFar[current.polyIndex] +
                Distance(centroids[current.polyIndex], centroids[neighbor]);

            if (newCost < costSoFar[neighbor]) {
                costSoFar[neighbor] = newCost;
                cameFrom[neighbor] = current.polyIndex;
                frontier.push({neighbor, newCost, newCost + heuristic(neighbor)});
            }
        }
    }

    return result;
}

std::vector<Vec2> NavMesh::FunnelPath(const std::vector<int>& polyPath,
                                      const Vec2& startPt,
                                      const Vec2& goalPt) {
    if (polyPath.empty() || polygons.empty()) {
        return {};
    }

    const Vec2 clampedStart = ClampToNavMesh(startPt);
    const Vec2 clampedGoal = ClampToNavMesh(goalPt);

    const std::vector<Portal> portals = BuildPortals(polyPath, polygons, clampedStart, clampedGoal);
    if (portals.empty()) {
        return {};
    }

    std::vector<Vec2> projectedPortalPath = BuildProjectedPortalPath(portals, clampedGoal);
    if (IsPathInsideCorridor(projectedPortalPath, polyPath, polygons)) {
        return projectedPortalPath;
    }

    std::cout << "NavMesh: projected portal path left corridor, falling back to midpoint portal path.\n";
    std::vector<Vec2> safePath = BuildSafePortalPath(portals);

    if (IsPathInsideCorridor(safePath, polyPath, polygons)) {
        return safePath;
    }

    std::cout << "NavMesh: midpoint portal path also failed corridor validation.\n";
    return {};
}

//------------------------------------------------------------
// Internal Helper Functions
//------------------------------------------------------------

int NavMesh::GetPolygonIndexAt(int x, int y) const {
    if (polygons.empty()) {
        return -1;
    }

    const Vec2 point = {x, y};

    for (size_t i = 0; i < polygons.size(); ++i) {
        if (PointInPolygon(point, polygons[i].vertices)) {
            return static_cast<int>(i);
        }
    }

    double bestDist = std::numeric_limits<double>::max();
    int bestIndex = -1;

    for (size_t i = 0; i < polygons.size(); ++i) {
        const Vec2 centroid = ComputeCentroid(polygons[i].vertices);
        const double d = DistanceSquared(point, centroid);
        if (d < bestDist) {
            bestDist = d;
            bestIndex = static_cast<int>(i);
        }
    }

    return bestIndex;
}

Vec2 NavMesh::ClampToNavMesh(const Vec2& pt) {
    if (polygons.empty()) {
        return pt;
    }

    const int pIdx = GetPolygonIndexAt(pt.x, pt.y);
    if (pIdx < 0 || pIdx >= static_cast<int>(polygons.size())) {
        return pt;
    }

    if (PointInPolygon(pt, polygons[pIdx].vertices)) {
        return pt;
    }

    return ClampPointToPolygon(pt, polygons[pIdx].vertices);
}