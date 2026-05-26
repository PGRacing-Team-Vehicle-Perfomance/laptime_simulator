#pragma once

#include <vector>

#include "config/config.h"
#include "coordTypes.h"

template <typename Frame>
class SteeringTable {
   public:
    enum class OutOfRangeBehaviour { Throw, Extrapolate };
    enum class Mode { Symmetric, Asymmetric };

    struct WheelAngles {
        Alpha<Frame> left;
        Alpha<Frame> right;
    };

    SteeringTable() = default;
    explicit SteeringTable(const Config& config);

    WheelAngles lookup(Alpha<Frame> steeringAngle) const;

   private:
    struct Entry {
        float input;
        float inner;
        float outer;
    };
    struct InnerOuter {
        float inner;
        float outer;
    };

    Mode mode = Mode::Symmetric;
    std::vector<Entry> symEntries;
    std::vector<Entry> asymLeftEntries;
    std::vector<Entry> asymRightEntries;
    OutOfRangeBehaviour outOfRangeBehaviour = OutOfRangeBehaviour::Throw;
    Transform<Frame, ISO8855> toIso;

    static std::vector<Entry> loadEntries(const Config& config, const std::string& prefix,
                                          float scale);
    InnerOuter lookupAbs(const std::vector<Entry>& entries, float absInput) const;
};

#include "steeringTable.inl"
