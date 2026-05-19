#ifndef METADATA_H
#define METADATA_H

#include <string>

bool LoadRuntimeMetadata(const std::string& jsonPath);

bool GetPointPosition(const std::string& label, int& outX, int& outY);
bool GetObjectCenter(const std::string& label, int& outX, int& outY);
bool GetTriggerCenter(const std::string& label, int& outX, int& outY);
bool IsPointInsideTrigger(const std::string& label, float x, float y);

#endif
