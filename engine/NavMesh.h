#ifndef NAVMESH_H
#define NAVMESH_H

#include <vector>
#include <string>
#include <SDL3/SDL.h>

struct Vec2 {
    int x;
    int y;
};

struct Portal {
    Vec2 left;
    Vec2 right;
};

struct NavPolygon {
    std::vector<Vec2> vertices;
    std::vector<int> neighborIndices;
};

class NavMesh {
public:
    static NavMesh& Instance();

    bool LoadFromFile(const std::string& filename);
    void Clear();

    std::vector<int> FindPath(int startIndex, int goalIndex);

    void DebugRender(SDL_Renderer* renderer, float cameraX, float cameraY) const;

    int GetPolygonIndexAt(int x, int y) const;
    Vec2 ClampToNavMesh(const Vec2& pt);

    std::vector<Vec2> FunnelPath(const std::vector<int>& polyPath,
                                 const Vec2& startPt,
                                 const Vec2& goalPt);

private:
    NavMesh() = default;
    NavMesh(const NavMesh&) = delete;
    NavMesh& operator=(const NavMesh&) = delete;

    std::vector<NavPolygon> polygons;
};

#endif // NAVMESH_H