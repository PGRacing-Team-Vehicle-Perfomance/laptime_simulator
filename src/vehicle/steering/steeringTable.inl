#pragma once

#include "vehicle/steering/steeringTable.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

template <typename Frame>
inline typename SteeringTable<Frame>::Entry SteeringTable<Frame>::loadEntry(
    const Config& config, const std::string& entryPrefix, float scale) {
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

template <typename Frame>
inline void SteeringTable<Frame>::checkNoEntryGap(const Config& config, const std::string& prefix,
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

template <typename Frame>
inline void SteeringTable<Frame>::checkNoOrphanAsymKeys(const Config& config) {
    if (config.has("Vehicle", "steeringTable.left.0.input") ||
        config.has("Vehicle", "steeringTable.right.0.input")) {
        throw std::runtime_error(
            "steeringTable.left.*/right.* keys present but symmetry=1 (symmetric mode); "
            "set steeringTable.symmetry=0 for asymmetric, or remove .left/.right keys");
    }
}

template <typename Frame>
inline std::vector<typename SteeringTable<Frame>::Entry>
SteeringTable<Frame>::loadEntries(const Config& config, const std::string& prefix, float scale) {
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

template <typename Frame>
inline SteeringTable<Frame>::SteeringTable(const Config& config) {
    float scale = config.angleUnitScale("Vehicle", "steeringTable");
    float symmetryRaw = config.get("Vehicle", "steeringTable.symmetry", 1);
    if (symmetryRaw != 0 && symmetryRaw != 1) {
        throw std::runtime_error(
            "steeringTable.symmetry must be 0 (asymmetric) or 1 (symmetric), got: " +
            std::to_string(symmetryRaw));
    }
    int symmetry = static_cast<int>(symmetryRaw);
    bool hasAnyEntry = false;
    if (symmetry == 0) {
        mode = Mode::Asymmetric;
        asymLeftEntries = loadEntries(config, "steeringTable.left", scale);
        asymRightEntries = loadEntries(config, "steeringTable.right", scale);
        if (asymLeftEntries.empty() || asymRightEntries.empty()) {
            throw std::runtime_error(
                "steeringTable.symmetry=0 (asymmetric) requires both steeringTable.left.* and "
                "steeringTable.right.* entries");
        }
        hasAnyEntry = true;
    } else {
        mode = Mode::Symmetric;
        checkNoOrphanAsymKeys(config);
        symEntries = loadEntries(config, "steeringTable", scale);
        hasAnyEntry = !symEntries.empty();
    }

    bool behaviourSet = config.has("Vehicle", "steeringTable.outOfRangeBehaviour");
    if (behaviourSet && !hasAnyEntry) {
        throw std::runtime_error(
            "steeringTable.outOfRangeBehaviour set but no steeringTable entries defined");
    }
    std::string behaviour =
        config.getString("Vehicle", "steeringTable.outOfRangeBehaviour", "throw");
    if (behaviour == "throw") {
        outOfRangeBehaviour = OutOfRangeBehaviour::Throw;
    } else if (behaviour == "extrapolate") {
        outOfRangeBehaviour = OutOfRangeBehaviour::Extrapolate;
    } else {
        throw std::runtime_error(
            "steeringTable.outOfRangeBehaviour must be 'throw' or 'extrapolate', got: " + behaviour);
    }
}

template <typename Frame>
inline typename SteeringTable<Frame>::WheelAngles SteeringTable<Frame>::lookup(
    Alpha<Frame> steeringAngle) const {
    if (mode == Mode::Symmetric && symEntries.empty()) {
        return {steeringAngle, steeringAngle};
    }
    float frameSign = (steeringAngle.v >= 0) ? 1.0f : -1.0f;
    bool physicalLeftTurn = toIso(steeringAngle).v >= 0;
    float absSteer = std::fabs(steeringAngle.v);

    const std::vector<Entry>& table =
        mode == Mode::Asymmetric ? (physicalLeftTurn ? asymLeftEntries : asymRightEntries)
                                 : symEntries;
    InnerOuter io = lookupAbs(table, absSteer);
    if (physicalLeftTurn) {
        return {Alpha<Frame>(frameSign * io.inner), Alpha<Frame>(frameSign * io.outer)};
    }
    return {Alpha<Frame>(frameSign * io.outer), Alpha<Frame>(frameSign * io.inner)};
}

template <typename Frame>
inline typename SteeringTable<Frame>::InnerOuter SteeringTable<Frame>::lookupAbs(
    const std::vector<Entry>& entries, float absInput) const {
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
                " rad exceeds steeringTable upper bound " +
                std::to_string(entries.back().input) +
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
