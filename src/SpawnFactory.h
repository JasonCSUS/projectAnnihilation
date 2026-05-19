#ifndef SPAWNFACTORY_H
#define SPAWNFACTORY_H

#include <string>

bool IsEntityAliveById(int entityId);
int SpawnNamedUnit(const std::string& entityName, int x, int y);
int SpawnNamedBuilding(const std::string& entityName, int x, int y);
int SpawnPlayerUnit(const std::string& entityName, int x, int y);

#endif
