#include "Metadata.h"

#include "JsonUtils.h"
#include "RuntimeData.h"

#include <algorithm>
#include <cmath>
#include <iostream>

bool LoadRuntimeMetadata(const std::string& jsonPath) {
    g_points.clear();
    g_objects.clear();
    g_triggers.clear();

    const std::string json = ReadTextFile(jsonPath);
    if (json.empty()) {
        std::cerr << "Failed to read metadata json: " << jsonPath << std::endl;
        return false;
    }

    {
        std::string body;
        if (ExtractSectionArray(json, "points", body)) {
            for (const std::string& obj : SplitTopLevelObjects(body)) {
                RuntimePoint p;
                if (!GetStringField(obj, "label", p.label)) {
                    continue;
                }

                float x = 0.0f;
                float y = 0.0f;
                if (GetPairField(obj, "position", x, y)) {
                    p.x = x;
                    p.y = y;
                } else {
                    GetNumberField(obj, "x", p.x);
                    GetNumberField(obj, "y", p.y);
                }

                g_points.push_back(p);
            }
        }
    }

    {
        std::string body;
        if (ExtractSectionArray(json, "objects", body)) {
            for (const std::string& obj : SplitTopLevelObjects(body)) {
                RuntimeObject o;
                if (!GetStringField(obj, "label", o.label)) {
                    continue;
                }
                GetStringField(obj, "kind", o.kind);
                GetNumberField(obj, "x", o.x);
                GetNumberField(obj, "y", o.y);
                GetNumberField(obj, "w", o.w);
                GetNumberField(obj, "h", o.h);
                g_objects.push_back(o);
            }
        }
    }

    {
        std::string body;
        if (ExtractSectionArray(json, "triggers", body)) {
            for (const std::string& obj : SplitTopLevelObjects(body)) {
                RuntimeTrigger t;
                if (!GetStringField(obj, "label", t.label)) {
                    continue;
                }
                GetStringField(obj, "kind", t.kind);
                GetNumberField(obj, "x", t.x);
                GetNumberField(obj, "y", t.y);
                GetNumberField(obj, "w", t.w);
                GetNumberField(obj, "h", t.h);
                g_triggers.push_back(t);
            }
        }
    }

    return true;
}

bool GetPointPosition(const std::string& label, int& outX, int& outY) {
    for (const auto& p : g_points) {
        if (p.label == label) {
            outX = static_cast<int>(std::lround(p.x));
            outY = static_cast<int>(std::lround(p.y));
            return true;
        }
    }
    return false;
}

bool GetObjectCenter(const std::string& label, int& outX, int& outY) {
    for (const auto& o : g_objects) {
        if (o.label == label) {
            outX = static_cast<int>(std::lround(o.x + o.w * 0.5f));
            outY = static_cast<int>(std::lround(o.y + o.h * 0.5f));
            return true;
        }
    }
    return false;
}

bool GetTriggerCenter(const std::string& label, int& outX, int& outY) {
    for (const auto& t : g_triggers) {
        if (t.label == label) {
            outX = static_cast<int>(std::lround(t.x + t.w * 0.5f));
            outY = static_cast<int>(std::lround(t.y + t.h * 0.5f));
            return true;
        }
    }
    return false;
}

bool IsPointInsideTrigger(const std::string& label, float x, float y) {
    for (const auto& t : g_triggers) {
        if (t.label != label) {
            continue;
        }

        if (t.kind == "rect") {
            return (x >= t.x && x <= t.x + t.w &&
                    y >= t.y && y <= t.y + t.h);
        }

        if (t.kind == "circle") {
            const float cx = t.x + t.w * 0.5f;
            const float cy = t.y + t.h * 0.5f;
            const float r = std::min(t.w, t.h) * 0.5f;
            const float dx = x - cx;
            const float dy = y - cy;
            return (dx * dx + dy * dy) <= (r * r);
        }
    }

    return false;
}
