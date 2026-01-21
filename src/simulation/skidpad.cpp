#include "simulation/skidpad.h"

SkidPad::SkidPad(const VehicleConfig& vehicleConfig, const SkidPadConfig& skidPadConfig,
                 const EnvironmentConfig& environmentConfig)
    : errDelta(skidPadConfig.errDelta),
      maxIterConv(skidPadConfig.maxIterConv),
      diameter(skidPadConfig.diameter),
      airDensity(environmentConfig.airDensity),
      earthAcc(environmentConfig.earthAcc) {
}
