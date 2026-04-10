#include "config/config.h"
#include "vehicle/vehicle.h"

#define _USE_MATH_DEFINES
#include <math.h>

class Simulation {
    Config cfg;
    
    template <typename VehicleFrame>
    WheelData<Positioned<std::unique_ptr<TireBase<VehicleFrame>>, VehicleFrame>> buildTires(Config& cfg);
    template <typename Frame>
    std::vector<std::array<float, 4>> getYawMomentDiagramPoints(
        Vehicle<Frame>& v, float speed, const Config& cfg,
        float maxSteeringAngle, float steeringAngleStep, float maxSlipAngle,
        float slipAngleStep, float tolerance, int maxIterations);

public:
    Simulation(Config config) : cfg(config) {}
    std::vector<std::array<float, 4>> run();
};
