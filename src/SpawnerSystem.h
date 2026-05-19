#ifndef SPAWNERSYSTEM_H
#define SPAWNERSYSTEM_H

#include <string>

void RefreshActiveSpawnersForRoom(const std::string& roomLabel);
void UpdateSpawners(float deltaTime);

#endif
