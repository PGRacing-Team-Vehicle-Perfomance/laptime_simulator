#pragma once

#include <vector>

#include "config/config.h"
#include "coordTypes.h"

template <typename Frame>
struct SteeringWheelAngles {
    Alpha<Frame> left;
    Alpha<Frame> right;
};

template <typename External>
class SteeringTableBase {
   public:
    virtual ~SteeringTableBase() = default;
    virtual SteeringWheelAngles<External> lookup(Alpha<External> steeringAngle) const = 0;
};

template <typename Internal, typename External>
class SteeringTable : public SteeringTableBase<External> {
   public:
    enum class OutOfRangeBehaviour { Throw, Extrapolate };
    enum class Mode { Symmetric, Asymmetric };

    SteeringTable() = default;
    explicit SteeringTable(const Config& config);

    SteeringWheelAngles<External> lookup(Alpha<External> steeringAngle) const override;

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
    Transform<External, Internal> toInternal;
    Transform<Internal, External> toExternal;

    static std::vector<Entry> loadEntries(const Config& config, const std::string& prefix,
                                          float scale);
    static Entry loadEntry(const Config& config, const std::string& entryPrefix, float scale);
    static void checkNoEntryGap(const Config& config, const std::string& prefix, int validCount);
    static void checkNoOrphanAsymKeys(const Config& config);
    InnerOuter lookupAbs(const std::vector<Entry>& entries, float absInput) const;
};

#include "steeringTable.inl"
