#include <cstdio>
#include <iostream>

#include "config/config.h"
#include "vehicle/vehicle.h"

template <typename Frame>
std::vector<std::array<float, 4>> getYawMomentDiagramPoints(
    Vehicle<Frame>& v, float speed, const EnvironmentConfig<Frame>& environmentConfig,
    float maxSteeringAngle = 10, float steeringAngleStep = 1, float maxSlipAngle = 10,
    float slipAngleStep = 1, float tolerance = 0.01, int maxIterations = 100) {
    std::vector<std::array<float, 4>> out;

    v.setSpeed(11);

    for (float steeringAngle = -maxSteeringAngle; steeringAngle <= maxSteeringAngle;
         steeringAngle += steeringAngleStep) {
        v.setSteeringAngle(Alpha<Frame>(steeringAngle * M_PI / 180.f));

        for (float chassisSlipAngle = -maxSlipAngle; chassisSlipAngle <= maxSlipAngle;
             chassisSlipAngle += slipAngleStep) {
            v.setChassisSlipAngle(Alpha<Frame>(chassisSlipAngle * M_PI / 180.f));

            std::array<float, 2> diagramPoint =
                v.calculateLatAccAndYawMoment(tolerance, maxIterations, environmentConfig);
            out.push_back({steeringAngle, chassisSlipAngle, diagramPoint[0], diagramPoint[1]});
        }
    }
    return out;
}

int main() {
    using VehicleFrame = ISO8855;
    using TireFrame = SAE;

    TireConfig tireConfig;
    VehicleConfig<VehicleFrame> vehicleConfig;
    EnvironmentConfig<VehicleFrame> environmentConfig;

    // later will be part of vehicle builder
    WheelData<Positioned<std::unique_ptr<TireBase<VehicleFrame>>, VehicleFrame>> tires;
    tires.FL.value =
        std::make_unique<TirePacejka<TireFrame, VehicleFrame>>(tireConfig, false, Left);
    tires.FR.value =
        std::make_unique<TirePacejka<TireFrame, VehicleFrame>>(tireConfig, false, Right);
    tires.RL.value =
        std::make_unique<TirePacejka<TireFrame, VehicleFrame>>(tireConfig, false, Left);
    tires.RR.value =
        std::make_unique<TirePacejka<TireFrame, VehicleFrame>>(tireConfig, false, Right);

    Vehicle<VehicleFrame> v(vehicleConfig, std::move(tires));

    auto points = getYawMomentDiagramPoints(v, 11, environmentConfig, 20, 2, 20, 1);
    FILE* f = fopen("build/yaw_diagram.csv", "w");
    if (f) {
        fprintf(f, "steering,slip,latAcc,yawMoment\n");
        for (const auto& p : points) {
            fprintf(f, "%f,%f,%f,%f\n", p[0], p[1], p[2], p[3]);
        }
        fclose(f);
        std::cout << "Wrote build/yaw_diagram.csv\n";
    } else {
        std::cerr << "Failed to open build/yaw_diagram.csv for writing\n";
    }
}
