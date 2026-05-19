
#ifndef NAVMESH_INTERNAL_H
#define NAVMESH_INTERNAL_H

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "NavMesh.h"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <limits>
#include <queue>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace navmesh_internal {

inline bool PointInPolygon(const Vec2& p, const std::vector<Vec2>& vertices) {
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

inline Vec2 ComputeCentroid(const std::vector<Vec2>& vertices) {
    long long sumX = 0;
    long long sumY = 0;
    for (const auto& v : vertices) {
        sumX += v.x;
        sumY += v.y;
    }
    const int count = static_cast<int>(vertices.size());
    if (count == 0) return {0, 0};
    return {static_cast<int>(sumX / count), static_cast<int>(sumY / count)};
}

inline double Distance(const Vec2& a, const Vec2& b) {
    const double dx = static_cast<double>(a.x - b.x);
    const double dy = static_cast<double>(a.y - b.y);
    return std::sqrt(dx * dx + dy * dy);
}

inline double DistanceSquared(const Vec2& a, const Vec2& b) {
    const double dx = static_cast<double>(a.x - b.x);
    const double dy = static_cast<double>(a.y - b.y);
    return dx * dx + dy * dy;
}

inline int TriArea2(const Vec2& a, const Vec2& b, const Vec2& c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

inline Vec2 ClampPointToPolygon(const Vec2& pt, const std::vector<Vec2>& poly) {
    double bestDist = std::numeric_limits<double>::max();
    Vec2 bestPt = pt;
    const size_t n = poly.size();
    for (size_t i = 0; i < n; ++i) {
        const Vec2 A = poly[i];
        const Vec2 B = poly[(i + 1) % n];
        const double abx = static_cast<double>(B.x - A.x);
        const double aby = static_cast<double>(B.y - A.y);
        const double abSq = abx * abx + aby * aby;
        if (abSq == 0.0) continue;
        double t = ((pt.x - A.x) * abx + (pt.y - A.y) * aby) / abSq;
        t = std::max(0.0, std::min(1.0, t));
        const Vec2 proj = {
            static_cast<int>(std::lround(A.x + abx * t)),
            static_cast<int>(std::lround(A.y + aby * t))
        };
        const double d = DistanceSquared(pt, proj);
        if (d < bestDist) {
            bestDist = d;
            bestPt = proj;
        }
    }
    return bestPt;
}

inline double AngleBetweenDirections(const Vec2& a, const Vec2& b, const Vec2& c) {
    const double v1x = static_cast<double>(b.x - a.x);
    const double v1y = static_cast<double>(b.y - a.y);
    const double v2x = static_cast<double>(c.x - b.x);
    const double v2y = static_cast<double>(c.y - b.y);
    const double len1 = std::sqrt(v1x * v1x + v1y * v1y);
    const double len2 = std::sqrt(v2x * v2x + v2y * v2y);
    if (len1 < 1e-9 || len2 < 1e-9) return 0.0;
    double dot = (v1x * v2x + v1y * v2y) / (len1 * len2);
    dot = std::max(-1.0, std::min(1.0, dot));
    return std::acos(dot);
}

inline bool PointInCorridor(const Vec2& p,
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

inline bool IsSegmentInsideCorridorSampled(const Vec2& a,
                                           const Vec2& b,
                                           const std::vector<int>& polyPath,
                                           const std::vector<NavPolygon>& polygons) {
    if (!PointInCorridor(a, polyPath, polygons) || !PointInCorridor(b, polyPath, polygons)) {
        return false;
    }

    const double dx = static_cast<double>(b.x - a.x);
    const double dy = static_cast<double>(b.y - a.y);
    const double length = std::sqrt(dx * dx + dy * dy);
    if (length <= 1.0) return true;

    const int steps = std::max(1, static_cast<int>(std::ceil(length / 8.0)));
    for (int i = 1; i < steps; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(steps);
        const Vec2 sample = {
            static_cast<int>(std::lround(a.x + dx * t)),
            static_cast<int>(std::lround(a.y + dy * t))
        };
        if (!PointInCorridor(sample, polyPath, polygons)) return false;
    }
    return true;
}

inline bool IsPathInsideCorridor(const std::vector<Vec2>& path,
                                 const std::vector<int>& polyPath,
                                 const std::vector<NavPolygon>& polygons) {
    if (path.empty()) return false;
    if (path.size() == 1) return PointInCorridor(path[0], polyPath, polygons);
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        if (!IsSegmentInsideCorridorSampled(path[i], path[i + 1], polyPath, polygons)) {
            return false;
        }
    }
    return true;
}

inline Vec2 PortalMidpoint(const Portal& p) {
    return {(p.left.x + p.right.x) / 2, (p.left.y + p.right.y) / 2};
}

inline Portal InsetPortal(const Portal& p, double insetDist) {
    const double dx = static_cast<double>(p.right.x - p.left.x);
    const double dy = static_cast<double>(p.right.y - p.left.y);
    const double len = std::sqrt(dx * dx + dy * dy);
    if (len <= 1e-6 || insetDist * 2.0 >= len) return p;

    const double ux = dx / len;
    const double uy = dy / len;

    Portal inset = p;
    inset.left = {
        static_cast<int>(std::lround(p.left.x + ux * insetDist)),
        static_cast<int>(std::lround(p.left.y + uy * insetDist))
    };
    inset.right = {
        static_cast<int>(std::lround(p.right.x - ux * insetDist)),
        static_cast<int>(std::lround(p.right.y - uy * insetDist))
    };
    return inset;
}

inline std::vector<Portal> BuildPortals(const std::vector<int>& polyPath,
                                        const std::vector<NavPolygon>& polygons,
                                        const Vec2& start,
                                        const Vec2& goal) {
    std::vector<Portal> portals;
    if (polyPath.empty()) return portals;

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

        if (!found) continue;

        const Vec2 currCenter = ComputeCentroid(A.vertices);
        const Vec2 nextCenter = ComputeCentroid(B.vertices);
        if (TriArea2(currCenter, nextCenter, sharedA) > 0) portals.push_back({sharedB, sharedA});
        else portals.push_back({sharedA, sharedB});
    }

    portals.push_back({goal, goal});
    return portals;
}

struct PortalSample {
    Vec2 point;
    double landingPenalty = 0.0;
};

inline std::vector<PortalSample> BuildPortalSamples(const Portal& portal, bool fixedPoint) {
    std::vector<PortalSample> samples;
    if (fixedPoint || (portal.left.x == portal.right.x && portal.left.y == portal.right.y)) {
        samples.push_back({portal.left, 0.0});
        return samples;
    }

    const Portal inset = InsetPortal(portal, 18.0);
    const double dx = static_cast<double>(inset.right.x - inset.left.x);
    const double dy = static_cast<double>(inset.right.y - inset.left.y);
    const double len = std::sqrt(dx * dx + dy * dy);

    if (len < 6.0) {
        samples.push_back({PortalMidpoint(inset), 0.0});
        return samples;
    }

    static const double TS[] = {0.0, 0.2, 0.4, 0.5, 0.6, 0.8, 1.0};
    for (double t : TS) {
        const Vec2 p = {
            static_cast<int>(std::lround(inset.left.x + dx * t)),
            static_cast<int>(std::lround(inset.left.y + dy * t))
        };

        bool duplicate = false;
        for (const auto& existing : samples) {
            if (existing.point.x == p.x && existing.point.y == p.y) {
                duplicate = true;
                break;
            }
        }
        if (duplicate) continue;

        const double midpointBias = std::abs(t - 0.5) * 10.0;
        samples.push_back({p, midpointBias});
    }

    if (samples.empty()) samples.push_back({PortalMidpoint(inset), 0.0});
    return samples;
}

struct CorridorNode {
    int layer = 0;
    int sample = 0;
    double cost = 0.0;
    bool operator>(const CorridorNode& other) const { return cost > other.cost; }
};

inline std::vector<Vec2> SimplifyCorridorPath(const std::vector<Vec2>& path,
                                              const std::vector<int>& polyPath,
                                              const std::vector<NavPolygon>& polygons) {
    if (path.size() <= 2) return path;

    std::vector<Vec2> simplified;
    simplified.push_back(path.front());

    size_t i = 0;
    while (i < path.size() - 1) {
        size_t best = i + 1;
        for (size_t j = path.size() - 1; j > i + 1; --j) {
            if (IsSegmentInsideCorridorSampled(simplified.back(), path[j], polyPath, polygons)) {
                best = j;
                break;
            }
        }
        simplified.push_back(path[best]);
        i = best;
    }

    return simplified;
}

inline std::vector<Vec2> BuildSampledCorridorPath(const std::vector<Portal>& portals,
                                                  const std::vector<int>& polyPath,
                                                  const std::vector<NavPolygon>& polygons) {
    std::vector<Vec2> empty;
    if (portals.empty()) return empty;

    std::vector<std::vector<PortalSample>> layers;
    layers.reserve(portals.size());
    for (size_t i = 0; i < portals.size(); ++i) {
        const bool fixedPoint = (i == 0 || i == portals.size() - 1);
        layers.push_back(BuildPortalSamples(portals[i], fixedPoint));
        if (layers.back().empty()) return empty;
    }

    const int layerCount = static_cast<int>(layers.size());
    std::vector<std::vector<double>> best(layerCount);
    std::vector<std::vector<int>> cameLayer(layerCount);
    std::vector<std::vector<int>> cameSample(layerCount);

    for (int i = 0; i < layerCount; ++i) {
        best[i].assign(layers[i].size(), std::numeric_limits<double>::max());
        cameLayer[i].assign(layers[i].size(), -1);
        cameSample[i].assign(layers[i].size(), -1);
    }

    std::priority_queue<CorridorNode, std::vector<CorridorNode>, std::greater<CorridorNode>> open;
    best[0][0] = 0.0;
    open.push({0, 0, 0.0});

    while (!open.empty()) {
        const CorridorNode cur = open.top();
        open.pop();

        if (cur.cost > best[cur.layer][cur.sample]) continue;

        if (cur.layer == layerCount - 1) {
            std::vector<Vec2> raw;
            int layer = cur.layer;
            int sample = cur.sample;

            while (layer >= 0 && sample >= 0) {
                raw.push_back(layers[layer][sample].point);
                const int prevLayer = cameLayer[layer][sample];
                const int prevSample = cameSample[layer][sample];
                layer = prevLayer;
                sample = prevSample;
            }

            std::reverse(raw.begin(), raw.end());
            return SimplifyCorridorPath(raw, polyPath, polygons);
        }

        const Vec2 fromPoint = layers[cur.layer][cur.sample].point;

        for (int nextLayer = cur.layer + 1; nextLayer < layerCount; ++nextLayer) {
            for (int nextSample = 0; nextSample < static_cast<int>(layers[nextLayer].size()); ++nextSample) {
                const Vec2 toPoint = layers[nextLayer][nextSample].point;
                if (!IsSegmentInsideCorridorSampled(fromPoint, toPoint, polyPath, polygons)) continue;

                double edgeCost = Distance(fromPoint, toPoint);
                edgeCost += layers[nextLayer][nextSample].landingPenalty;
                edgeCost -= static_cast<double>(nextLayer - cur.layer - 1) * 6.0;

                const double newCost = cur.cost + std::max(0.0, edgeCost);
                if (newCost < best[nextLayer][nextSample]) {
                    best[nextLayer][nextSample] = newCost;
                    cameLayer[nextLayer][nextSample] = cur.layer;
                    cameSample[nextLayer][nextSample] = cur.sample;
                    open.push({nextLayer, nextSample, newCost});
                }
            }
        }
    }

    return empty;
}

struct PolyState {
    int prevPoly = -1;
    int curPoly = -1;
    bool operator==(const PolyState& other) const {
        return prevPoly == other.prevPoly && curPoly == other.curPoly;
    }
};

struct PolyStateHash {
    std::size_t operator()(const PolyState& s) const {
        const std::size_t h1 = std::hash<int>{}(s.prevPoly);
        const std::size_t h2 = std::hash<int>{}(s.curPoly);
        return h1 ^ (h2 << 1);
    }
};

struct SearchNode {
    PolyState state;
    double g = 0.0;
    double f = 0.0;
    bool operator>(const SearchNode& other) const { return f > other.f; }
};

struct EdgeOwner {
    int polyIndex = -1;
    int edgeIndex = -1;
};

struct EdgeKey {
    int ax = 0;
    int ay = 0;
    int bx = 0;
    int by = 0;

    bool operator==(const EdgeKey& other) const {
        return ax == other.ax && ay == other.ay && bx == other.bx && by == other.by;
    }
};

struct EdgeKeyHash {
    std::size_t operator()(const EdgeKey& k) const {
        std::size_t h = 1469598103934665603ull;
        auto mix = [&](int v) {
            h ^= static_cast<std::size_t>(std::hash<int>{}(v));
            h *= 1099511628211ull;
        };
        mix(k.ax);
        mix(k.ay);
        mix(k.bx);
        mix(k.by);
        return h;
    }
};

inline EdgeKey MakeCanonicalEdgeKey(const Vec2& a, const Vec2& b) {
    if (a.x < b.x || (a.x == b.x && a.y <= b.y)) {
        return {a.x, a.y, b.x, b.y};
    }
    return {b.x, b.y, a.x, a.y};
}

inline bool ExtractSectionArray(const std::string& json,
                                const std::string& key,
                                std::string& outArrayBody) {
    const std::string token = "\"" + key + "\"";
    const size_t keyPos = json.find(token);
    if (keyPos == std::string::npos) return false;

    const size_t bracketStart = json.find('[', keyPos);
    if (bracketStart == std::string::npos) return false;

    int depth = 0;
    for (size_t i = bracketStart; i < json.size(); ++i) {
        if (json[i] == '[') depth++;
        else if (json[i] == ']') {
            depth--;
            if (depth == 0) {
                outArrayBody = json.substr(bracketStart + 1, i - bracketStart - 1);
                return true;
            }
        }
    }

    return false;
}

inline std::vector<std::string> SplitTopLevelObjects(const std::string& arrayBody) {
    std::vector<std::string> objects;
    int braceDepth = 0;
    size_t objStart = std::string::npos;

    for (size_t i = 0; i < arrayBody.size(); ++i) {
        if (arrayBody[i] == '{') {
            if (braceDepth == 0) objStart = i;
            braceDepth++;
        } else if (arrayBody[i] == '}') {
            braceDepth--;
            if (braceDepth == 0 && objStart != std::string::npos) {
                objects.push_back(arrayBody.substr(objStart, i - objStart + 1));
                objStart = std::string::npos;
            }
        }
    }

    return objects;
}

inline bool ExtractStringField(const std::string& objectText,
                               const std::string& key,
                               std::string& outValue) {
    const std::string token = "\"" + key + "\"";
    const size_t keyPos = objectText.find(token);
    if (keyPos == std::string::npos) return false;

    const size_t colon = objectText.find(':', keyPos);
    if (colon == std::string::npos) return false;

    const size_t firstQuote = objectText.find('"', colon + 1);
    if (firstQuote == std::string::npos) return false;

    const size_t secondQuote = objectText.find('"', firstQuote + 1);
    if (secondQuote == std::string::npos) return false;

    outValue = objectText.substr(firstQuote + 1, secondQuote - firstQuote - 1);
    return true;
}

inline bool ExtractBoolField(const std::string& objectText,
                             const std::string& key,
                             bool& outValue) {
    const std::string token = "\"" + key + "\"";
    const size_t keyPos = objectText.find(token);
    if (keyPos == std::string::npos) return false;

    const size_t colon = objectText.find(':', keyPos);
    if (colon == std::string::npos) return false;

    size_t start = colon + 1;
    while (start < objectText.size() &&
           std::isspace(static_cast<unsigned char>(objectText[start]))) {
        ++start;
    }

    if (objectText.compare(start, 4, "true") == 0) {
        outValue = true;
        return true;
    }
    if (objectText.compare(start, 5, "false") == 0) {
        outValue = false;
        return true;
    }

    return false;
}

inline bool ExtractIntField(const std::string& objectText,
                            const std::string& key,
                            int& outValue) {
    const std::string token = "\"" + key + "\"";
    const size_t keyPos = objectText.find(token);
    if (keyPos == std::string::npos) return false;

    const size_t colon = objectText.find(':', keyPos);
    if (colon == std::string::npos) return false;

    size_t start = colon + 1;
    while (start < objectText.size() &&
           std::isspace(static_cast<unsigned char>(objectText[start]))) {
        ++start;
    }

    size_t end = start;
    while (end < objectText.size() &&
           (objectText[end] == '-' || (objectText[end] >= '0' && objectText[end] <= '9'))) {
        ++end;
    }

    if (end == start) return false;

    outValue = std::stoi(objectText.substr(start, end - start));
    return true;
}

inline bool ExtractFloatField(const std::string& objectText,
                              const std::string& key,
                              float& outValue) {
    const std::string token = "\"" + key + "\"";
    const size_t keyPos = objectText.find(token);
    if (keyPos == std::string::npos) return false;

    const size_t colon = objectText.find(':', keyPos);
    if (colon == std::string::npos) return false;

    size_t start = colon + 1;
    while (start < objectText.size() &&
           std::isspace(static_cast<unsigned char>(objectText[start]))) {
        ++start;
    }

    size_t end = start;
    while (end < objectText.size() &&
           (objectText[end] == '-' || objectText[end] == '+' ||
            objectText[end] == '.' || objectText[end] == 'e' ||
            objectText[end] == 'E' || (objectText[end] >= '0' && objectText[end] <= '9'))) {
        ++end;
    }

    if (end == start) return false;

    outValue = std::stof(objectText.substr(start, end - start));
    return true;
}

inline bool ExtractIntArrayField(const std::string& objectText,
                                 const std::string& key,
                                 std::vector<int>& outValues) {
    const std::string token = "\"" + key + "\"";
    const size_t keyPos = objectText.find(token);
    if (keyPos == std::string::npos) return false;

    const size_t open = objectText.find('[', keyPos);
    if (open == std::string::npos) return false;

    int depth = 0;
    size_t close = std::string::npos;
    for (size_t i = open; i < objectText.size(); ++i) {
        if (objectText[i] == '[') depth++;
        else if (objectText[i] == ']') {
            depth--;
            if (depth == 0) {
                close = i;
                break;
            }
        }
    }

    if (close == std::string::npos) return false;

    std::stringstream ss(objectText.substr(open + 1, close - open - 1));
    std::string chunk;
    outValues.clear();

    while (std::getline(ss, chunk, ',')) {
        size_t start = 0;
        while (start < chunk.size() &&
               std::isspace(static_cast<unsigned char>(chunk[start]))) {
            ++start;
        }
        if (start >= chunk.size()) continue;
        outValues.push_back(std::stoi(chunk.substr(start)));
    }

    return true;
}

inline int QuantizeClearanceBucketValue(int radius) {
    if (radius <= 0) return 0;
    if (radius <= 20) return 20;
    if (radius <= 30) return 30;
    return 40;
}

inline float DistancePointToSegment(const Vec2& a, const Vec2& b, const Vec2& p) {
    const float abx = static_cast<float>(b.x - a.x);
    const float aby = static_cast<float>(b.y - a.y);
    const float abLenSq = abx * abx + aby * aby;
    if (abLenSq <= 0.0001f) {
        const float dx = static_cast<float>(p.x - a.x);
        const float dy = static_cast<float>(p.y - a.y);
        return std::sqrt(dx * dx + dy * dy);
    }

    float t = ((static_cast<float>(p.x - a.x) * abx) +
               (static_cast<float>(p.y - a.y) * aby)) / abLenSq;
    t = std::max(0.0f, std::min(1.0f, t));

    const float projX = static_cast<float>(a.x) + abx * t;
    const float projY = static_cast<float>(a.y) + aby * t;
    const float dx = static_cast<float>(p.x) - projX;
    const float dy = static_cast<float>(p.y) - projY;
    return std::sqrt(dx * dx + dy * dy);
}

inline void BuildRectWalls(float x,
                           float y,
                           float w,
                           float h,
                           int ownerPoly,
                           const std::string& toggleId,
                           std::vector<NavWallSegment>& outWalls) {
    const Vec2 p0{static_cast<int>(std::lround(x)), static_cast<int>(std::lround(y))};
    const Vec2 p1{static_cast<int>(std::lround(x + w)), static_cast<int>(std::lround(y))};
    const Vec2 p2{static_cast<int>(std::lround(x + w)), static_cast<int>(std::lround(y + h))};
    const Vec2 p3{static_cast<int>(std::lround(x)), static_cast<int>(std::lround(y + h))};

    outWalls.push_back({p0, p1, ownerPoly, true, toggleId});
    outWalls.push_back({p1, p2, ownerPoly, true, toggleId});
    outWalls.push_back({p2, p3, ownerPoly, true, toggleId});
    outWalls.push_back({p3, p0, ownerPoly, true, toggleId});
}

inline void BuildCircleWalls(float x,
                             float y,
                             float w,
                             float h,
                             int ownerPoly,
                             const std::string& toggleId,
                             std::vector<NavWallSegment>& outWalls) {
    const float r = std::min(w, h) * 0.5f;
    const float cx = x + w * 0.5f;
    const float cy = y + h * 0.5f;

    constexpr int SEGMENTS = 20;
    std::vector<Vec2> pts;
    pts.reserve(SEGMENTS);

    for (int i = 0; i < SEGMENTS; ++i) {
        const double t = (static_cast<double>(i) / static_cast<double>(SEGMENTS)) * 2.0 * M_PI;
        pts.push_back({
            static_cast<int>(std::lround(cx + std::cos(t) * r)),
            static_cast<int>(std::lround(cy + std::sin(t) * r))
        });
    }

    for (int i = 0; i < SEGMENTS; ++i) {
        outWalls.push_back({pts[i], pts[(i + 1) % SEGMENTS], ownerPoly, true, toggleId});
    }
}

inline Vec2 ComputeInwardNormal(const NavPolygon& poly, int edgeIndex) {
    if (poly.vertices.empty()) return {0, 0};
    const Vec2 a = poly.vertices[edgeIndex];
    const Vec2 b = poly.vertices[(edgeIndex + 1) % poly.vertices.size()];
    const int dx = b.x - a.x;
    const int dy = b.y - a.y;
    return {-dy, dx};
}

inline bool ClosestPointOnSegment(const Vec2& a,
                                  const Vec2& b,
                                  float px,
                                  float py,
                                  float& outX,
                                  float& outY,
                                  float& outT) {
    const float abx = static_cast<float>(b.x - a.x);
    const float aby = static_cast<float>(b.y - a.y);
    const float abLenSq = abx * abx + aby * aby;
    if (abLenSq <= 0.0001f) {
        outX = static_cast<float>(a.x);
        outY = static_cast<float>(a.y);
        outT = 0.0f;
        return false;
    }

    float t = ((px - static_cast<float>(a.x)) * abx +
               (py - static_cast<float>(a.y)) * aby) / abLenSq;
    t = std::max(0.0f, std::min(1.0f, t));

    outX = static_cast<float>(a.x) + abx * t;
    outY = static_cast<float>(a.y) + aby * t;
    outT = t;
    return true;
}

} // namespace navmesh_internal

#endif // NAVMESH_INTERNAL_H
