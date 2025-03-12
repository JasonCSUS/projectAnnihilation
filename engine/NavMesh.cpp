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

//---------------------------------------------------------------------
// Helper functions
//---------------------------------------------------------------------

void NavMesh::DebugRender(SDL_Renderer* renderer, float cameraX, float cameraY) const {
    SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
    for (const auto& poly : polygons) {
        size_t numVerts = poly.vertices.size();
        for (size_t i = 0; i < numVerts; ++i) {
            const Vec2& a = poly.vertices[i];
            const Vec2& b = poly.vertices[(i + 1) % numVerts];
            SDL_RenderLine(renderer, a.x - cameraX, a.y - cameraY, b.x - cameraX, b.y - cameraY);
        }
    }
}

// Ray-casting algorithm for point-in-polygon.
static bool PointInPolygon(const Vec2& p, const std::vector<Vec2>& vertices) {
    bool inside = false;
    int n = vertices.size();
    for (int i = 0, j = n - 1; i < n; j = i++) {
        if (((vertices[i].y > p.y) != (vertices[j].y > p.y)) &&
            (p.x < (vertices[j].x - vertices[i].x) * (p.y - vertices[i].y) /
                   static_cast<double>(vertices[j].y - vertices[i].y) + vertices[i].x))
            inside = !inside;
    }
    return inside;
}

// Compute polygon centroid.
static Vec2 ComputeCentroid(const std::vector<Vec2>& vertices) {
    long long sumX = 0, sumY = 0;
    for (const auto& v : vertices) {
        sumX += v.x;
        sumY += v.y;
    }
    int count = vertices.size();
    return { static_cast<int>(sumX / count), static_cast<int>(sumY / count) };
}

// Euclidean distance.
static double Distance(const Vec2& a, const Vec2& b) {
    int dx = a.x - b.x;
    int dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

//---------------------------------------------------------------------
// NavMesh singleton accessor
//---------------------------------------------------------------------
NavMesh& NavMesh::Instance() {
    static NavMesh instance;
    return instance;
}

//---------------------------------------------------------------------
// NavMesh class implementation
//---------------------------------------------------------------------
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
            int x, y;
            file.read(reinterpret_cast<char*>(&x), sizeof(int));
            file.read(reinterpret_cast<char*>(&y), sizeof(int));
            poly.vertices[j] = { x, y };
        }
        int neighborCount = 0;
        file.read(reinterpret_cast<char*>(&neighborCount), sizeof(int));
        poly.neighborIndices.resize(neighborCount);
        for (int j = 0; j < neighborCount; ++j) {
            file.read(reinterpret_cast<char*>(&poly.neighborIndices[j]), sizeof(int));
        }
        polygons[i] = poly;
    }
    file.close();
    std::cout << "Loaded navmesh with " << numPolygons << " polygons.\n";
    return true;
}

void NavMesh::Clear() {
    polygons.clear();
}

int NavMesh::GetPolygonIndexAt(int x, int y) const {
    Vec2 point = { x, y };
    for (size_t i = 0; i < polygons.size(); ++i) {
        if (PointInPolygon(point, polygons[i].vertices))
            return static_cast<int>(i);
    }
    double bestDist = std::numeric_limits<double>::max();
    int bestIndex = -1;
    for (size_t i = 0; i < polygons.size(); ++i) {
        Vec2 centroid = ComputeCentroid(polygons[i].vertices);
        double d = Distance(point, centroid);
        if (d < bestDist) {
            bestDist = d;
            bestIndex = static_cast<int>(i);
        }
    }
    return bestIndex;
}

//---------------------------------------------------------------------
// A* Pathfinding on polygon adjacency graph.
//---------------------------------------------------------------------
struct Node {
    int polyIndex;
    double cost;
    double priority;
};

std::vector<int> NavMesh::FindPath(int startIndex, int goalIndex) {
    std::vector<int> result;
    if (startIndex < 0 || goalIndex < 0 ||
        startIndex >= static_cast<int>(polygons.size()) ||
        goalIndex >= static_cast<int>(polygons.size()))
        return result;
    
    std::vector<Vec2> centroids(polygons.size());
    for (size_t i = 0; i < polygons.size(); ++i) {
        centroids[i] = ComputeCentroid(polygons[i].vertices);
    }
    auto heuristic = [&](int idx) -> double {
        return Distance(centroids[idx], centroids[goalIndex]);
    };
    auto cmp = [](const Node& a, const Node& b) { return a.priority > b.priority; };
    std::priority_queue<Node, std::vector<Node>, decltype(cmp)> frontier(cmp);
    std::vector<double> costSoFar(polygons.size(), std::numeric_limits<double>::max());
    std::vector<int> cameFrom(polygons.size(), -1);
    frontier.push({ startIndex, 0.0, heuristic(startIndex) });
    costSoFar[startIndex] = 0.0;
    while (!frontier.empty()) {
        Node current = frontier.top();
        frontier.pop();
        if (current.polyIndex == goalIndex) {
            int cur = current.polyIndex;
            while (cur != -1) {
                result.push_back(cur);
                cur = cameFrom[cur];
            }
            std::reverse(result.begin(), result.end());
            return result;
        }
        for (int neighbor : polygons[current.polyIndex].neighborIndices) {
            double newCost = costSoFar[current.polyIndex] + Distance(centroids[current.polyIndex], centroids[neighbor]);
            if (newCost < costSoFar[neighbor]) {
                costSoFar[neighbor] = newCost;
                cameFrom[neighbor] = current.polyIndex;
                double priority = newCost + heuristic(neighbor);
                frontier.push({ neighbor, newCost, priority });
            }
        }
    }
    return result;
}

//---------------------------------------------------------------------
// Funnel Algorithm Helpers
//---------------------------------------------------------------------
static int Cross(const Vec2& a, const Vec2& b, const Vec2& c) {
    int abx = b.x - a.x;
    int aby = b.y - a.y;
    int acx = c.x - a.x;
    int acy = c.y - a.y;
    return abx * acy - aby * acx;
}

static bool ApproximatelyEqual(const Vec2& a, const Vec2& b, int epsilon = 2) {
    return (std::abs(a.x - b.x) <= epsilon && std::abs(a.y - b.y) <= epsilon);
}

// Get shared edge between two adjacent polygons (requires two consecutive vertices shared).
static bool GetSharedEdge(const NavPolygon& A, const NavPolygon& B, Vec2& outLeft, Vec2& outRight) {
    size_t nA = A.vertices.size();
    for (size_t i = 0; i < nA; ++i) {
        Vec2 a1 = A.vertices[i];
        Vec2 a2 = A.vertices[(i + 1) % nA];
        bool foundA1 = false, foundA2 = false;
        for (const auto& v : B.vertices) {
            if (ApproximatelyEqual(a1, v))
                foundA1 = true;
            if (ApproximatelyEqual(a2, v))
                foundA2 = true;
        }
        if (foundA1 && foundA2) {
            outLeft = a1;
            outRight = a2;
            return true;
        }
    }
    return false;
}

static Vec2 ClampPointToPolygon(const Vec2& pt, const std::vector<Vec2>& poly) {
    double bestDist = std::numeric_limits<double>::max();
    Vec2 bestPt = pt;
    size_t n = poly.size();
    for (size_t i = 0; i < n; ++i) {
        Vec2 A = poly[i];
        Vec2 B = poly[(i + 1) % n];
        int abx = B.x - A.x;
        int aby = B.y - A.y;
        double abSq = (double)abx * abx + (double)aby * aby;
        if (abSq == 0) continue;
        double t = ((pt.x - A.x) * abx + (pt.y - A.y) * aby) / abSq;
        t = std::max(0.0, std::min(1.0, t));
        Vec2 proj = { A.x + static_cast<int>(abx * t), A.y + static_cast<int>(aby * t) };
        double d = std::hypot(pt.x - proj.x, pt.y - proj.y);
        if (d < bestDist) {
            bestDist = d;
            bestPt = proj;
        }
    }
    return bestPt;
}

static bool IsSegmentInside(const Vec2& A, const Vec2& B, int samples = 10) {
    for (int i = 0; i <= samples; i++) {
        double t = i / static_cast<double>(samples);
        Vec2 sample = { A.x + static_cast<int>((B.x - A.x) * t), A.y + static_cast<int>((B.y - A.y) * t) };
        int pIndex = NavMesh::Instance().GetPolygonIndexAt(sample.x, sample.y);
        if (!PointInPolygon(sample, NavMesh::Instance().polygons[pIndex].vertices))
            return false;
    }
    return true;
}

static std::vector<Vec2> SmoothPathRDP(const std::vector<Vec2>& pts, double epsilon) {
    if (pts.size() < 3) return pts;
    double maxDist = 0;
    int index = 0;
    Vec2 start = pts.front();
    Vec2 end = pts.back();
    for (size_t i = 1; i < pts.size() - 1; i++) {
        double dx = (double)end.x - start.x;
        double dy = (double)end.y - start.y;
        double norm = std::hypot(dx, dy);
        double d = 0;
        if (norm != 0.0)
            d = std::abs(dy * pts[i].x - dx * pts[i].y + end.x * start.y - end.y * start.x) / norm;
        if (d > maxDist) {
            maxDist = d;
            index = i;
        }
    }
    if (maxDist > epsilon) {
        std::vector<Vec2> rec1 = SmoothPathRDP(std::vector<Vec2>(pts.begin(), pts.begin() + index + 1), epsilon);
        std::vector<Vec2> rec2 = SmoothPathRDP(std::vector<Vec2>(pts.begin() + index, pts.end()), epsilon);
        rec1.pop_back();
        rec1.insert(rec1.end(), rec2.begin(), rec2.end());
        return rec1;
    } else {
        return { start, end };
    }
}

// -----------------------------------------------------------------
// Constrained Funnel Path:
// Every polygon in the A* path is forced, but we choose an optimal point 
// (the polygon's clamped centroid) for each intermediate node. Then we 
// attempt to smooth the path by skipping nodes if a direct line-of-sight exists.
// -----------------------------------------------------------------
std::vector<Vec2> NavMesh::ConstrainedFunnelPath(const std::vector<int>& polyPath, const Vec2& startPt, const Vec2& goalPt) {
    std::vector<Vec2> waypoints;
    if (polyPath.empty()) return waypoints;
    
    // Force start point.
    waypoints.push_back(startPt);
    
    // For each intermediate polygon, choose its centroid clamped to the polygon.
    for (size_t i = 1; i < polyPath.size() - 1; i++) {
        int idx = polyPath[i];
        Vec2 candidate = ComputeCentroid(NavMesh::Instance().polygons[idx].vertices);
        if (!PointInPolygon(candidate, NavMesh::Instance().polygons[idx].vertices))
            candidate = ClampPointToPolygon(candidate, NavMesh::Instance().polygons[idx].vertices);
        waypoints.push_back(candidate);
    }
    
    // Force goal.
    waypoints.push_back(goalPt);
    
    // Smooth the route: skip intermediate waypoints if a direct line-of-sight exists.
    std::vector<Vec2> optimized;
    optimized.push_back(waypoints[0]);
    size_t i = 0;
    while (i < waypoints.size() - 1) {
        size_t j = waypoints.size() - 1;
        for (; j > i; j--) {
            if (IsSegmentInside(optimized.back(), waypoints[j]))
                break;
        }
        optimized.push_back(waypoints[j]);
        i = j;
    }
    return optimized;
}


// -----------------------------------------------------------------
// Primary FunnelPath function using the constrained approach.
// -----------------------------------------------------------------
std::vector<Vec2> NavMesh::FunnelPath(const std::vector<int>& polyPath, const Vec2& startPt, const Vec2& goalPt) {
    std::vector<Vec2> path = ConstrainedFunnelPath(polyPath, startPt, goalPt);

    // Post-process: RDP smoothing.
    double rdpEpsilon = 5.0;
    std::vector<Vec2> smooth = SmoothPathRDP(path, rdpEpsilon);

    // Clamp each waypoint to be inside its containing polygon.
    for (Vec2 &pt : smooth) {
        int pIdx = this->GetPolygonIndexAt(pt.x, pt.y);
        if (!PointInPolygon(pt, this->polygons[pIdx].vertices))
            pt = ClampPointToPolygon(pt, this->polygons[pIdx].vertices);
    }
    return smooth;
}
