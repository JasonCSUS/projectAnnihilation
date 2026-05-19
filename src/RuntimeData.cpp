#include "RuntimeData.h"

std::unordered_map<std::string, RuntimeEntityDefinition> g_entityDefs;
std::vector<RuntimePoint> g_points;
std::vector<RuntimeObject> g_objects;
std::vector<RuntimeTrigger> g_triggers;
std::vector<RuntimeSpawner> g_activeSpawners;
float g_spawnerTimer = 0.0f;

void ClearRuntimeData() {
    g_entityDefs.clear();
    g_points.clear();
    g_objects.clear();
    g_triggers.clear();
    g_activeSpawners.clear();
    g_spawnerTimer = 0.0f;
}
