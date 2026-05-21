#pragma once

#include "vehicle/steering/ackermannTable.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

template <typename Internal, typename External>
inline AckermannTable<Internal, External>::AckermannTable(const Config& config) {
    constexpr float degToRad = M_PI / 180.0f;
    for (int i = 0;; i++) {
        std::string prefix = "ackermannTable." + std::to_string(i);
        if (!config.has("Vehicle", prefix + ".input")) break;
        Entry entry;
        entry.input = config.get("Vehicle", prefix + ".input") * degToRad;
        entry.inner = config.get("Vehicle", prefix + ".inner") * degToRad;
        entry.outer = config.get("Vehicle", prefix + ".outer") * degToRad;
        entries.push_back(entry);
    }
    std::sort(entries.begin(), entries.end(),
              [](const Entry& a, const Entry& b) { return a.input < b.input; });

    bool behaviourSet = config.has("Vehicle", "ackermannTable.outOfRangeBehaviour");
    if (behaviourSet && entries.empty()) {
        throw std::runtime_error(
            "ackermannTable.outOfRangeBehaviour set but no ackermannTable entries defined");
    }
    if (config.getString("Vehicle", "ackermannTable.outOfRangeBehaviour", "extrapolate") == "clamp") {
        outOfRangeBehaviour = OutOfRangeBehaviour::Clamp;
    }
}

template <typename Internal, typename External>
inline typename AckermannTable<Internal, External>::WheelAngles
AckermannTable<Internal, External>::lookup(Alpha<External> steeringAngle) const {
    Alpha<Internal> internalSteer = toInternal(steeringAngle);
    if (entries.empty()) {
        return {toExternal(internalSteer), toExternal(internalSteer)};
    }
    InnerOuter io = lookupAbs(std::fabs(internalSteer.v));
    if (internalSteer.v >= 0) {
        return {toExternal(Alpha<Internal>(io.inner)), toExternal(Alpha<Internal>(io.outer))};
    }
    return {toExternal(Alpha<Internal>(-io.outer)), toExternal(Alpha<Internal>(-io.inner))};
}

template <typename Internal, typename External>
inline typename AckermannTable<Internal, External>::InnerOuter
AckermannTable<Internal, External>::lookupAbs(float absInput) const {
    if (absInput <= entries.front().input) {
        if (entries.front().input <= 0) {
            return {entries.front().inner, entries.front().outer};
        }
        float t = absInput / entries.front().input;
        return {t * entries.front().inner, t * entries.front().outer};
    }
    if (absInput >= entries.back().input) {
        if (outOfRangeBehaviour == OutOfRangeBehaviour::Clamp) {
            return {entries.back().inner, entries.back().outer};
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
            float t = (absInput - entries[i].input) /
                      (entries[i + 1].input - entries[i].input);
            return {entries[i].inner + t * (entries[i + 1].inner - entries[i].inner),
                    entries[i].outer + t * (entries[i + 1].outer - entries[i].outer)};
        }
    }
    return {absInput, absInput};
}
