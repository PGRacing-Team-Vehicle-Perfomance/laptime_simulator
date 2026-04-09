#pragma once

#include "configHelper.cpp"
#include "coordTypes.h"
#include "types.h"
#include "vehicle/vehicleHelper.h"

#include <string>
#include <unordered_map>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <iostream>

class Config {
private:
    std::unordered_map<std::string, float> data;
    std::unordered_map<std::string, std::string> stringData;

public:
    Config() = default;

    Config(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            file.open("../" + filename);
        }
        if (!file.is_open()) {
            throw std::runtime_error("Could not open config file: " + filename);
        }

        std::string line;
        bool header = true;
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;

            if (header) {
                header = false;
                continue;
            }

            std::stringstream ss(line);
            std::string module, param, valueStr;

            if (std::getline(ss, module, ',') &&
                std::getline(ss, param, ',') &&
                std::getline(ss, valueStr, ',')) {
                
                if (!valueStr.empty() && valueStr.back() == '\r') {
                    valueStr.pop_back();
                }
                
                stringData[module + "." + param] = valueStr;
                
                try {
                    float value = std::stof(valueStr);
                    data[module + "." + param] = value;
                } catch (const std::exception& e) {
                    // Fail silently for floats, it might be a valid intentional text field
                }
            }
        }

        if(data.find("Environment.airDensity") == data.end()) {
            data["Environment.airDensity"] = airDensity(data["Environment.airTemperature"], data["Environment.airPressure"], data["Environment.airHumidity"]);
        }
    }

    float get(const std::string& module, const std::string& param) const {
        std::string key = module + "." + param;
        auto it = data.find(key);
        if (it != data.end()) {
            return it->second;
        }
        std::cerr << "Config parameter not found: " << key << "\n";
        return 0.0f;
    }
    
    float get(const std::string& module, const std::string& param, float defaultVal) const {
        std::string key = module + "." + param;
        auto it = data.find(key);
        if (it != data.end()) {
            return it->second;
        }
        return defaultVal;
    }

    std::string getString(const std::string& module, const std::string& param, const std::string& defaultVal = "") const {
        std::string key = module + "." + param;
        auto it = stringData.find(key);
        if (it != stringData.end()) {
            return it->second;
        }
        return defaultVal;
    }

    template <typename T>
    WheelData<T> getWheelData(const std::string& module, const std::string& prefix) const {
        WheelData<T> wd;
        wd.FL = static_cast<T>(get(module, prefix + ".FL"));
        wd.FR = static_cast<T>(get(module, prefix + ".FR"));
        wd.RL = static_cast<T>(get(module, prefix + ".RL"));
        wd.RR = static_cast<T>(get(module, prefix + ".RR"));
        return wd;
    }
    
    template <typename Frame>
    WheelData<Alpha<Frame>> getAlphaWheelData(const std::string& module, const std::string& prefix) const {
        WheelData<Alpha<Frame>> wd;
        wd.FL = Alpha<Frame>(get(module, prefix + ".FL"));
        wd.FR = Alpha<Frame>(get(module, prefix + ".FR"));
        wd.RL = Alpha<Frame>(get(module, prefix + ".RL"));
        wd.RR = Alpha<Frame>(get(module, prefix + ".RR"));
        return wd;
    }

    template <typename Frame>
    Vec<Frame> getVec(const std::string& module, const std::string& prefix) const {
        return Vec<Frame>(get(module, prefix + ".x"), get(module, prefix + ".y"), get(module, prefix + ".z"));
    }

    std::unordered_map<std::string, float> getModule(const std::string& module) const {
        std::unordered_map<std::string, float> result;
        std::string prefix = module + ".";
        for (const auto& pair : data) {
            if (pair.first.rfind(prefix, 0) == 0) {
                result[pair.first.substr(prefix.length())] = pair.second;
            }
        }
        return result;
    }
};
