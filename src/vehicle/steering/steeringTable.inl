#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include "vehicle/steering/steeringTable.h"

template <typename Internal, typename External>
inline typename SteeringTable<Internal, External>::Entry
SteeringTable<Internal, External>::loadEntry(const Config& config, const std::string& entryPrefix,
                                             float scale) {
    if (!config.has("Vehicle", entryPrefix + ".inner") ||
        !config.has("Vehicle", entryPrefix + ".outer")) {
        throw std::runtime_error(entryPrefix +
                                 " missing .inner or .outer (both required alongside .input)");
    }
    Entry entry;
    entry.input = config.get("Vehicle", entryPrefix + ".input") * scale;
    entry.inner = config.get("Vehicle", entryPrefix + ".inner") * scale;
    entry.outer = config.get("Vehicle", entryPrefix + ".outer") * scale;
    return entry;
}

template <typename Internal, typename External>
inline void SteeringTable<Internal, External>::checkNoEntryGap(const Config& config,
                                                               const std::string& prefix,
                                                               int validCount) {
    constexpr int forwardScan = 16;
    for (int j = validCount + 1; j <= validCount + forwardScan; j++) {
        std::string entryPrefix = prefix + "." + std::to_string(j);
        if (config.has("Vehicle", entryPrefix + ".input")) {
            throw std::runtime_error(prefix + " has gap at index " + std::to_string(validCount) +
                                     " (index " + std::to_string(j) + " is present)");
        }
    }
}

template <typename Internal, typename External>
inline void SteeringTable<Internal, External>::checkNoOrphanAsymKeys(const Config& config) {
    if (config.has("Vehicle", "steeringTable.left.0.input") ||
        config.has("Vehicle", "steeringTable.right.0.input")) {
        throw std::runtime_error(
            "steeringTable.left.*/right.* keys present but symmetry=1 (symmetric mode); "
            "set steeringTable.symmetry=0 for asymmetric, or remove .left/.right keys");
    }
}

template <typename Internal, typename External>
inline std::vector<typename SteeringTable<Internal, External>::Entry>
SteeringTable<Internal, External>::loadEntries(const Config& config, const std::string& prefix,
                                               float scale) {
    std::vector<float> rawInputs;
    std::vector<Entry> result;
    for (int i = 0;; i++) {
        std::string entryPrefix = prefix + "." + std::to_string(i);
        if (!config.has("Vehicle", entryPrefix + ".input")) break;
        rawInputs.push_back(config.get("Vehicle", entryPrefix + ".input"));
        result.push_back(loadEntry(config, entryPrefix, scale));
    }
    checkNoEntryGap(config, prefix, static_cast<int>(result.size()));
    std::vector<float> sortedRawInputs = rawInputs;
    std::sort(sortedRawInputs.begin(), sortedRawInputs.end());
    for (size_t i = 1; i < sortedRawInputs.size(); i++) {
        if (sortedRawInputs[i] == sortedRawInputs[i - 1]) {
            throw std::runtime_error(prefix + " has duplicate input value " +
                                     std::to_string(sortedRawInputs[i]));
        }
    }
    std::sort(result.begin(), result.end(),
              [](const Entry& a, const Entry& b) { return a.input < b.input; });
    return result;
}

template <typename Internal, typename External>
inline SteeringTable<Internal, External>::SteeringTable(const Config& config) {
    float scale = config.angleUnitScale("Vehicle", "steeringTable");
    float symmetryRaw = config.get("Vehicle", "steeringTable.symmetry", 1);
    if (symmetryRaw != 0 && symmetryRaw != 1) {
        throw std::runtime_error(
            "steeringTable.symmetry must be 0 (asymmetric) or 1 (symmetric), got: " +
            std::to_string(symmetryRaw));
    }
    int symmetry = static_cast<int>(symmetryRaw);
    if (symmetry == 0) {
        mode = Mode::Asymmetric;
        asymLeftEntries = loadEntries(config, "steeringTable.left", scale);
        asymRightEntries = loadEntries(config, "steeringTable.right", scale);
        if (asymLeftEntries.empty() || asymRightEntries.empty()) {
            throw std::runtime_error(
                "steeringTable.symmetry=0 (asymmetric) requires both steeringTable.left.* and "
                "steeringTable.right.* entries");
        }
    } else {
        mode = Mode::Symmetric;
        checkNoOrphanAsymKeys(config);
        symEntries = loadEntries(config, "steeringTable", scale);
    }

    std::string behaviour =
        config.getString("Vehicle", "steeringTable.outOfRangeBehaviour", "throw");
    if (behaviour == "throw") {
        outOfRangeBehaviour = OutOfRangeBehaviour::Throw;
    } else if (behaviour == "extrapolate") {
        outOfRangeBehaviour = OutOfRangeBehaviour::Extrapolate;
    } else {
        throw std::runtime_error(
            "steeringTable.outOfRangeBehaviour must be 'throw' or 'extrapolate', got: " +
            behaviour);
    }
}

template <typename Internal, typename External>
inline SteeringWheelAngles<External> SteeringTable<Internal, External>::lookup(
    Alpha<External> steeringAngle) const {
    if (mode == Mode::Symmetric && symEntries.empty()) {
        return {steeringAngle, steeringAngle};
    }
    Alpha<Internal> internalSteer = toInternal(steeringAngle);
    bool leftTurn = internalSteer.v >= 0;
    float sign = leftTurn ? 1.0f : -1.0f;
    float absSteer = std::fabs(internalSteer.v);

    const std::vector<Entry>& table =
        mode == Mode::Asymmetric ? (leftTurn ? asymLeftEntries : asymRightEntries) : symEntries;
    InnerOuter io = lookupAbs(table, absSteer);
    if (leftTurn) {
        return {toExternal(Alpha<Internal>{sign * io.inner}),
                toExternal(Alpha<Internal>{sign * io.outer})};
    }
    return {toExternal(Alpha<Internal>{sign * io.outer}),
            toExternal(Alpha<Internal>{sign * io.inner})};
}

template <typename Internal, typename External>
inline typename SteeringTable<Internal, External>::InnerOuter
SteeringTable<Internal, External>::lookupAbs(const std::vector<Entry>& entries,
                                             float absInput) const {
    if (absInput <= entries.front().input) {
        if (entries.front().input <= 0) {
            return {entries.front().inner, entries.front().outer};
        }
        float t = absInput / entries.front().input;
        return {t * entries.front().inner, t * entries.front().outer};
    }
    if (absInput > entries.back().input) {
        if (outOfRangeBehaviour == OutOfRangeBehaviour::Throw) {
            throw std::runtime_error(
                "steering input " + std::to_string(absInput) +
                " rad exceeds steeringTable upper bound " + std::to_string(entries.back().input) +
                " rad (set steeringTable.outOfRangeBehaviour=extrapolate to allow)");
        }
        float prevInput = entries.size() >= 2 ? entries[entries.size() - 2].input : 0.0f;
        float prevInner = entries.size() >= 2 ? entries[entries.size() - 2].inner : 0.0f;
        float prevOuter = entries.size() >= 2 ? entries[entries.size() - 2].outer : 0.0f;
        float span = entries.back().input - prevInput;
        float t = (absInput - prevInput) / span;
        return {prevInner + t * (entries.back().inner - prevInner),
                prevOuter + t * (entries.back().outer - prevOuter)};
    }
    for (size_t i = 0; i + 1 < entries.size(); i++) {
        if (absInput >= entries[i].input && absInput <= entries[i + 1].input) {
            float t = (absInput - entries[i].input) / (entries[i + 1].input - entries[i].input);
            return {entries[i].inner + t * (entries[i + 1].inner - entries[i].inner),
                    entries[i].outer + t * (entries[i + 1].outer - entries[i].outer)};
        }
    }
    throw std::runtime_error("steering input " + std::to_string(absInput) +
                             " is not finite or steeringTable entries are non-monotonic");
}
