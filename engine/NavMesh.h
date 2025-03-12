#ifndef NAVMESH_H
#define NAVMESH_H
#include <vector>
#include <string>
#include <SDL3/SDL.h>

// Simple 2D vector structure for vertex coordinates.
struct Vec2 {
    int x;
    int y;
};

// Represents one convex polygon in the navmesh.
struct NavPolygon {
    std::vector<Vec2> vertices;       // Vertices in world coordinates.
    std::vector<int> neighborIndices; // Indices of neighboring polygons.
};

class NavMesh {
public:
    // Accessor for the singleton instance.
    static NavMesh& Instance();

    // Loads the navmesh from a binary file.
    // The file format is defined by your Python generation script.
    bool LoadFromFile(const std::string& filename);

    // Clears the navmesh data.
    void Clear();

    // Returns the index of the polygon that contains the point (x, y).
    // If the point isn’t inside any polygon, returns the index of the polygon with the nearest centroid.
    int GetPolygonIndexAt(int x, int y) const;

    // Uses A* to find a sequence of polygon indices connecting startIndex to goalIndex.
    std::vector<int> FindPath(int startIndex, int goalIndex);

    std::vector<Vec2> NavMesh::ConstrainedFunnelPath(const std::vector<int>& polyPath, const Vec2& startPt, const Vec2& goalPt); 
    // Given a polygon path (a list of polygon indices), plus start and goal points,
    // computes a smoothed path (in world coordinates) using the funnel algorithm.
    std::vector<Vec2> FunnelPath(const std::vector<int>& polygonPath, const Vec2& startPoint, const Vec2& goalPoint);

    // Publicly accessible list of polygons.
    std::vector<NavPolygon> polygons;

    // Debug rendering: draws polygon outlines.
    void DebugRender(SDL_Renderer* renderer, float cameraX, float cameraY) const;

private:
    // Private constructor for singleton.
    NavMesh() = default;
    // Delete copy constructor and assignment operator.
    NavMesh(const NavMesh&) = delete;
    NavMesh& operator=(const NavMesh&) = delete;
};

#endif // NAVMESH_H
