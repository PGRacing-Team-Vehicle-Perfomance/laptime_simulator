#pragma once

#include <cstdio>

#include "config/config.h"
#include "vehicle/vehicle.h"

#define _USE_MATH_DEFINES
#include <math.h>

struct DiagramSample {
    float steering;
    float slip;
    bool inSteeringFamily;
    bool inSlipFamily;
    PointSolution solution;
};

void writeDiagramHeader(FILE* f);
void writeDiagramRow(FILE* f, const DiagramSample& sample);

class Simulation {
    Config cfg;

    template <typename VehicleFrame>
    WheelData<Positioned<std::unique_ptr<TireBase<VehicleFrame>>, VehicleFrame>> buildTires(
        Config& cfg);
    template <typename VehicleFrame>
    Positioned<std::unique_ptr<AeroBase<VehicleFrame>>, VehicleFrame> buildAero(Config& cfg);
    template <typename VehicleFrame>
    std::unique_ptr<SteeringTableBase<VehicleFrame>> buildSteeringTable(Config& cfg);
    template <typename VehicleFrame>
    std::unique_ptr<DifferentialBase<VehicleFrame>> buildDifferential(Config& cfg);
    template <typename Frame>
    std::vector<DiagramSample> getYawMomentDiagramPoints(Vehicle<Frame>& v, float speed,
                                                         const Config& cfg, float maxSteeringAngle,
                                                         float steeringAngleStep,
                                                         float maxSlipAngle, float slipAngleStep,
                                                         float tolerance, int maxIterations);

   public:
    Simulation(Config config) : cfg(config) {}
    std::vector<DiagramSample> run();
};
