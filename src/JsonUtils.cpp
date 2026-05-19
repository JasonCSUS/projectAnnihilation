#include "JsonUtils.h"

#include <fstream>
#include <regex>
#include <sstream>

std::string ReadTextFile(const std::string& path) {
    std::ifstream in(path);
    if (!in) {
        return "";
    }

    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

bool ExtractSectionArray(const std::string& json, const std::string& key, std::string& outArrayBody) {
    const std::string token = "\"" + key + "\"";
    const size_t keyPos = json.find(token);
    if (keyPos == std::string::npos) {
        return false;
    }

    const size_t bracketStart = json.find('[', keyPos);
    if (bracketStart == std::string::npos) {
        return false;
    }

    int depth = 0;
    for (size_t i = bracketStart; i < json.size(); ++i) {
        if (json[i] == '[') {
            depth++;
        } else if (json[i] == ']') {
            depth--;
            if (depth == 0) {
                outArrayBody = json.substr(bracketStart + 1, i - bracketStart - 1);
                return true;
            }
        }
    }

    return false;
}

std::vector<std::string> SplitTopLevelObjects(const std::string& arrayBody) {
    std::vector<std::string> objects;
    int braceDepth = 0;
    size_t objStart = std::string::npos;

    for (size_t i = 0; i < arrayBody.size(); ++i) {
        if (arrayBody[i] == '{') {
            if (braceDepth == 0) {
                objStart = i;
            }
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

bool GetStringField(const std::string& objectText, const std::string& key, std::string& outValue) {
    const std::regex re("\\\"" + key + "\\\"\\s*:\\s*\\\"([^\\\"]*)\\\"");
    std::smatch match;
    if (std::regex_search(objectText, match, re)) {
        outValue = match[1].str();
        return true;
    }
    return false;
}

bool GetNumberField(const std::string& objectText, const std::string& key, float& outValue) {
    const std::regex re("\\\"" + key + "\\\"\\s*:\\s*(-?\\d+(?:\\.\\d+)?)");
    std::smatch match;
    if (std::regex_search(objectText, match, re)) {
        outValue = std::stof(match[1].str());
        return true;
    }
    return false;
}

bool GetPairField(const std::string& objectText, const std::string& key, float& outA, float& outB) {
    const std::regex re("\\\"" + key + "\\\"\\s*:\\s*\\[\\s*(-?\\d+(?:\\.\\d+)?)\\s*,\\s*(-?\\d+(?:\\.\\d+)?)\\s*\\]");
    std::smatch match;
    if (std::regex_search(objectText, match, re)) {
        outA = std::stof(match[1].str());
        outB = std::stof(match[2].str());
        return true;
    }
    return false;
}

std::unordered_map<std::string, std::string> ParseCustomFields(const std::string& objectText) {
    std::unordered_map<std::string, std::string> result;
    const std::regex blockRe("\\\"custom_fields\\\"\\s*:\\s*\\{([^}]*)\\}");
    std::smatch blockMatch;
    if (!std::regex_search(objectText, blockMatch, blockRe)) {
        return result;
    }

    const std::string body = blockMatch[1].str();
    const std::regex pairRe("\\\"([^\\\"]+)\\\"\\s*:\\s*\\\"([^\\\"]*)\\\"");
    auto begin = std::sregex_iterator(body.begin(), body.end(), pairRe);
    auto end = std::sregex_iterator();
    for (auto it = begin; it != end; ++it) {
        result[(*it)[1].str()] = (*it)[2].str();
    }

    return result;
}

bool ParseBoolString(const std::unordered_map<std::string, std::string>& fields,
                     const std::string& key,
                     bool defaultValue) {
    auto it = fields.find(key);
    if (it == fields.end()) {
        return defaultValue;
    }
    return it->second == "true" || it->second == "1";
}

int ParseIntString(const std::unordered_map<std::string, std::string>& fields,
                   const std::string& key,
                   int defaultValue) {
    auto it = fields.find(key);
    if (it == fields.end() || it->second.empty()) {
        return defaultValue;
    }

    try {
        return std::stoi(it->second);
    } catch (...) {
        return defaultValue;
    }
}

float ParseFloatString(const std::unordered_map<std::string, std::string>& fields,
                       const std::string& key,
                       float defaultValue) {
    auto it = fields.find(key);
    if (it == fields.end() || it->second.empty()) {
        return defaultValue;
    }

    try {
        return std::stof(it->second);
    } catch (...) {
        return defaultValue;
    }
}

std::string ParseStringValue(const std::unordered_map<std::string, std::string>& fields,
                             const std::string& key,
                             const std::string& defaultValue) {
    auto it = fields.find(key);
    if (it == fields.end()) {
        return defaultValue;
    }
    return it->second;
}

std::string NormalizeAssetPath(const std::string& fileName) {
    if (fileName.empty()) {
        return "";
    }
    if (fileName.find('/') != std::string::npos || fileName.find('\\') != std::string::npos) {
        return fileName;
    }
    return std::string("./assets/") + fileName;
}
