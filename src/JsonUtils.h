#ifndef JSONUTILS_H
#define JSONUTILS_H

#include <string>
#include <unordered_map>
#include <vector>

std::string ReadTextFile(const std::string& path);
bool ExtractSectionArray(const std::string& json, const std::string& key, std::string& outArrayBody);
std::vector<std::string> SplitTopLevelObjects(const std::string& arrayBody);
bool GetStringField(const std::string& objectText, const std::string& key, std::string& outValue);
bool GetNumberField(const std::string& objectText, const std::string& key, float& outValue);
bool GetPairField(const std::string& objectText, const std::string& key, float& outA, float& outB);
std::unordered_map<std::string, std::string> ParseCustomFields(const std::string& objectText);
bool ParseBoolString(const std::unordered_map<std::string, std::string>& fields,
                     const std::string& key,
                     bool defaultValue = false);
int ParseIntString(const std::unordered_map<std::string, std::string>& fields,
                   const std::string& key,
                   int defaultValue = 0);
float ParseFloatString(const std::unordered_map<std::string, std::string>& fields,
                       const std::string& key,
                       float defaultValue = 0.0f);
std::string ParseStringValue(const std::unordered_map<std::string, std::string>& fields,
                             const std::string& key,
                             const std::string& defaultValue = "");
std::string NormalizeAssetPath(const std::string& fileName);

#endif
