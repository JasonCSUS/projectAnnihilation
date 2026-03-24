#include "GameMain.h"
#include "../engine/EntityManager.h"
#include "../engine/GameLoop.h"
#include "../engine/MapLoader.h"
#include "../engine/Character.h"
#include "../engine/AnimationManager.h"
#include "../engine/InputHandler.h"
#include "../engine/NavMesh.h"
#include "../engine/NavigationSystem.h"
#include "EntityLogic.h"

#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <regex>
#include <cmath>
#include <algorithm>
#include <string>

EntityManager entityManager;
InputHandler inputHandler;
MapLoader mapLoader;
float elapsedTime = 0.0f;

// ---------------------------------------------------------------------
// Stress-test globals

static SDL_Window* g_window = nullptr;

static int g_aliveEnemyCount = 0;
static int g_frozenEnemyCount = 0;
static bool g_enemySpawningPaused = false;
static float g_currentFPS = 0.0f;

// Spawn tuning for stress test
static float g_enemySpawnTimer = 0.0f;
static constexpr float ENEMY_SPAWN_INTERVAL = 0.05f; // 20 enemies/sec while healthy
static constexpr float FRAME_TIME_LIMIT = 1.0f / 60.0f;

// FPS smoothing over a rolling time window
static std::vector<float> g_recentFrameTimes;
static float g_recentFrameTimeSum = 0.0f;
static constexpr float FPS_SMOOTHING_WINDOW_SECONDS = 2.0f;

// ---------------------------------------------------------------------
// Runtime metadata loaded from navmesh.json

struct RuntimePoint {
    std::string label;
    float x = 0.0f;
    float y = 0.0f;
};

struct RuntimeObject {
    std::string label;
    std::string kind; // rect/circle
    float x = 0.0f;
    float y = 0.0f;
    float w = 0.0f;
    float h = 0.0f;
};

struct RuntimeTrigger {
    std::string label;
    std::string kind; // rect/circle
    float x = 0.0f;
    float y = 0.0f;
    float w = 0.0f;
    float h = 0.0f;
};

static std::vector<RuntimePoint> g_points;
static std::vector<RuntimeObject> g_objects;
static std::vector<RuntimeTrigger> g_triggers;

// ---------------------------------------------------------------------
// Small JSON helpers tailored to nav_export.py output

static std::string ReadTextFile(const std::string& path) {
    std::ifstream in(path);
    if (!in) {
        return "";
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

static bool ExtractSectionArray(const std::string& json, const std::string& key, std::string& outArrayBody) {
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

static std::vector<std::string> SplitTopLevelObjects(const std::string& arrayBody) {
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

static bool GetStringField(const std::string& objectText, const std::string& key, std::string& outValue) {
    const std::regex re("\"" + key + "\"\\s*:\\s*\"([^\"]*)\"");
    std::smatch match;
    if (std::regex_search(objectText, match, re)) {
        outValue = match[1].str();
        return true;
    }
    return false;
}

static bool GetNumberField(const std::string& objectText, const std::string& key, float& outValue) {
    const std::regex re("\"" + key + "\"\\s*:\\s*(-?\\d+(?:\\.\\d+)?)");
    std::smatch match;
    if (std::regex_search(objectText, match, re)) {
        outValue = std::stof(match[1].str());
        return true;
    }
    return false;
}

static bool GetPairField(const std::string& objectText, const std::string& key, float& outA, float& outB) {
    const std::regex re("\"" + key + "\"\\s*:\\s*\\[\\s*(-?\\d+(?:\\.\\d+)?)\\s*,\\s*(-?\\d+(?:\\.\\d+)?)\\s*\\]");
    std::smatch match;
    if (std::regex_search(objectText, match, re)) {
        outA = std::stof(match[1].str());
        outB = std::stof(match[2].str());
        return true;
    }
    return false;
}

static bool LoadRuntimeMetadata(const std::string& jsonPath) {
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
                if (!GetStringField(obj, "label", p.label)) continue;

                float x = 0.0f, y = 0.0f;
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
                if (!GetStringField(obj, "label", o.label)) continue;
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
                if (!GetStringField(obj, "label", t.label)) continue;
                GetStringField(obj, "kind", t.kind);
                GetNumberField(obj, "x", t.x);
                GetNumberField(obj, "y", t.y);
                GetNumberField(obj, "w", t.w);
                GetNumberField(obj, "h", t.h);
                g_triggers.push_back(t);
            }
        }
    }

    std::cout << "Loaded metadata: "
              << g_points.size() << " points, "
              << g_objects.size() << " objects, "
              << g_triggers.size() << " triggers.\n";
    return true;
}

// ---------------------------------------------------------------------
// Public metadata query helpers

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
        if (t.label != label) continue;

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

// ---------------------------------------------------------------------
// Stress-test stat accessors

int GetAliveEnemyCount() {
    return g_aliveEnemyCount;
}

int GetFrozenEnemyCount() {
    return g_frozenEnemyCount;
}

float GetCurrentFPS() {
    return g_currentFPS;
}

bool IsEnemySpawningPaused() {
    return g_enemySpawningPaused;
}

static void UpdateWindowTitle() {
    if (!g_window) return;

    std::ostringstream ss;
    ss << "ProjectAnnihilation | FPS: " << static_cast<int>(std::lround(g_currentFPS));

    if (g_enemySpawningPaused) {
        ss << " | Enemies (frozen): " << g_frozenEnemyCount << " | Spawns paused";
    } else {
        ss << " | Enemies alive: " << g_aliveEnemyCount;
    }

    SDL_SetWindowTitle(g_window, ss.str().c_str());
}

// ---------------------------------------------------------------------

void Spawn(EntityManager& entityManager, int unitType, int x, int y) {
    int r = 32;
    float baseSpeed = 150.0f;
    int controller = PLAYER;
    float visionRange = 500.0f;
    float attackRange = 300.0f;
    int hp = 20;

    switch (unitType)
    {
        case UNIT1:
            r = 32; baseSpeed = 250.0f; controller = PLAYER; visionRange = 500.0f; attackRange = 500.0f; hp = 150;
            break;
        case UNIT2:
            r = 32; baseSpeed = 150.0f; controller = ENEMY; visionRange = 500.0f; attackRange = 300.0f; hp = 20;
            break;
        default:
            break;
    }

    SDL_FRect pos = { static_cast<float>(x), static_cast<float>(y), static_cast<float>(r * 2), static_cast<float>(r * 2) };

    std::vector<Sprite> frames = idleDown;
    Animation animation;
    animation.frames = frames;
    animation.frameTime = 0.25f;
    animation.spriteW = r * 2;
    animation.spriteH = r * 2;
    animation.elapsedTime = (static_cast<float>(rand()) / RAND_MAX) * animation.frameTime;
    animation.currentFrame = rand() % frames.size();

    entityManager.AddEntity(controller, r, pos, unitType, animation, baseSpeed, visionRange, attackRange, hp);
}

void UpdateGame(float deltaTime) {
    elapsedTime += deltaTime;

    // Rolling-average FPS over the last few seconds to avoid knee-jerk swings.
    if (deltaTime > 0.0f) {
        g_recentFrameTimes.push_back(deltaTime);
        g_recentFrameTimeSum += deltaTime;

        while (g_recentFrameTimeSum > FPS_SMOOTHING_WINDOW_SECONDS && !g_recentFrameTimes.empty()) {
            g_recentFrameTimeSum -= g_recentFrameTimes.front();
            g_recentFrameTimes.erase(g_recentFrameTimes.begin());
        }

        if (!g_recentFrameTimes.empty() && g_recentFrameTimeSum > 0.0f) {
            const float avgFrameTime = g_recentFrameTimeSum / static_cast<float>(g_recentFrameTimes.size());
            g_currentFPS = (avgFrameTime > 0.00001f) ? (1.0f / avgFrameTime) : 0.0f;
        }
    }

    UpdateEnemyAI(entityManager);

    // Count living enemies after AI/damage updates
    int livingEnemies = 0;
    for (const auto& entity : entityManager.entities) {
        if (entity.controller == ENEMY && !entity.isDead) {
            livingEnemies++;
        }
    }

    g_aliveEnemyCount = livingEnemies;

    // Freeze spawn test when frame time exceeds 1/60 second.
    const float averagedFrameTime = (g_currentFPS > 0.00001f) ? (1.0f / g_currentFPS) : 9999.0f;

    if (!g_enemySpawningPaused && averagedFrameTime > FRAME_TIME_LIMIT) {
        g_enemySpawningPaused = true;
        g_frozenEnemyCount = g_aliveEnemyCount;
        std::cout << "Stress test paused at " << g_frozenEnemyCount
                << " living enemies. Averaged frame time exceeded "
                << FRAME_TIME_LIMIT << " seconds.\n";
    }

    // Keep spawning enemies beside object "enemy1_1" until paused.
    if (!g_enemySpawningPaused) {
        g_enemySpawnTimer += deltaTime;

        int ox = 0, oy = 0;
        if (GetObjectCenter("enemy1_1", ox, oy)) {
            while (g_enemySpawnTimer >= ENEMY_SPAWN_INTERVAL) {
                g_enemySpawnTimer -= ENEMY_SPAWN_INTERVAL;

                const int laneOffset = (g_aliveEnemyCount % 6) * 28;
                const int rowOffset = (g_aliveEnemyCount / 6 % 6) * 28;

                Spawn(entityManager, UNIT2, ox + 48 + laneOffset, oy + rowOffset);
                g_aliveEnemyCount++;
            }
        }
    }

    UpdateWindowTitle();
}

// start method
void GameMain(SDL_Window *window, SDL_Renderer *renderer) {
    std::cout << "Starting GameMain..." << std::endl;
    g_window = window;

    SDL_Texture* unit1Sprite = mapLoader.LoadTexture("./assets/placeholderSpriteSheet.bmp", renderer);
    entityManager.LoadTexture(UNIT1, unit1Sprite);
    SDL_Texture* unit1Sprite2 = mapLoader.LoadTexture("./assets/placeholderSpriteSheet2.bmp", renderer);
    entityManager.LoadTexture(UNIT2, unit1Sprite2);

    if (!mapLoader.LoadMap("./assets/map_preview.bmp", renderer)) {
        std::cerr << "Failed to load map image.\n";
    }

    if (!NavMesh::Instance().LoadFromFile("./assets/navmesh_polygons.nav")) {
        std::cerr << "Failed to load navmesh.\n";
    }

    if (!LoadRuntimeMetadata("./assets/navmesh.json")) {
        std::cerr << "Failed to load runtime metadata.\n";
    }

    NavigationSystem::Instance().InitializePathCache("assets/navcache.bin");

    int playerX = 270;
    int playerY = 460;
    if (!GetPointPosition("player_spawn", playerX, playerY)) {
        std::cout << "player_spawn not found, using fallback spawn.\n";
    }

    Spawn(entityManager, UNIT1, playerX, playerY);

    // Start stress test with one enemy near the spawner object if present.
    int enemyX = 1470;
    int enemyY = 1270;
    if (GetObjectCenter("enemy1_1", enemyX, enemyY)) {
        enemyX += 48;
    }
    Spawn(entityManager, UNIT2, enemyX, enemyY);

    GameLoop(window, renderer, mapLoader, entityManager, inputHandler, UpdateGame);
}