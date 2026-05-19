#ifndef ENTITYDEFINITIONS_H
#define ENTITYDEFINITIONS_H

#include <SDL3/SDL.h>
#include <string>

bool LoadEntityDefinitionsFromProject(const std::string& jsonPath);
void RegisterEntitySheets(SDL_Renderer* renderer);

#endif
