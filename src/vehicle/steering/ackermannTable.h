#pragma once

#include <vector>

#include "config/config.h"
#include "coordTypes.h"

template <typename Internal, typename External>
class AckermannTable {
   public:
    enum class OutOfRangeBehaviour { Clamp, Extrapolate };

    struct WheelAngles {
        Alpha<External> left;
        Alpha<External> right;
    };

    AckermannTable() = default;
    explicit AckermannTable(const Config& config);

    WheelAngles lookup(Alpha<External> steeringAngle) const;

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

    std::vector<Entry> entries;
    OutOfRangeBehaviour outOfRangeBehaviour = OutOfRangeBehaviour::Extrapolate;
    Transform<External, Internal> toInternal;
    Transform<Internal, External> toExternal;

    InnerOuter lookupAbs(float absInputInternal) const;
};

#include "ackermannTable.inl"
