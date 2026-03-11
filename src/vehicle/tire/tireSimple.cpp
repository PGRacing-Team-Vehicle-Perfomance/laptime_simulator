#include "vehicle/tire/tireSimple.h"

#include <cmath>

#include "config/config.h"
#include "coordTypes.h"
#include "types.h"

SimpleModel::SimpleModel(const TireConfig& config)
    : scalingFactor(config.scalingFac), quadFac(config.quadFac), linFac(config.linFac) {}

float SimpleModel::computeFy(float verticalLoad) {
    return scalingFactor * (quadFac * std::pow(verticalLoad, 2) + linFac * verticalLoad);
}
float SimpleModel::computeFx(float verticalLoad, bool isDriven) {
    if (isDriven) {
        return scalingFactor * (quadFac * std::pow(verticalLoad, 2) + linFac * verticalLoad);
    }
    return 0;
}
