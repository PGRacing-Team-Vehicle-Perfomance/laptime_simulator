#include <cstdio>
#include <iostream>

#include "config/config.h"
#include "vehicle/vehicle.h"
#include "vehicle/tire/tirePacejkaV2.h"
#include "vehicle/tire/tireSimple.h"

template <typename Frame>
std::vector<std::array<float, 4>> getYawMomentDiagramPoints(
    Vehicle<Frame>& v, float speed, const Config& config,
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
                v.calculateLatAccAndYawMoment(tolerance, maxIterations, config);
            out.push_back({steeringAngle, chassisSlipAngle, diagramPoint[0], diagramPoint[1]});
        }
    }
    return out;
}

int main() {
    using VehicleFrame = ISO8855;
    using TireFrame = SAE;

    Config cfg("config_pacejka_v1.csv");

    // later will be part of vehicle builder
    WheelData<Positioned<std::unique_ptr<TireBase<VehicleFrame>>, VehicleFrame>> tires;
    
    std::string impl = cfg.getString("Tire", "implementation");
    auto createTire = [&cfg, &impl](Side side) -> std::unique_ptr<TireBase<VehicleFrame>> {
        if (impl == "Simple") {
            return std::make_unique<TireSimple<TireFrame, VehicleFrame>>(cfg, false);
        } else if (impl == "PacejkaV2") {
            return std::make_unique<TirePacejkaV2<TireFrame, VehicleFrame>>(cfg, false, side);
        } else if (impl == "PacejkaV1"  ) {
            return std::make_unique<TirePacejkaV1<TireFrame, VehicleFrame>>(cfg, false, side);
        } else {
            throw std::runtime_error("Unknown tire implementation: " + impl);
        }
    };

    tires.FL.value = createTire(Left);
    tires.FR.value = createTire(Right);
    tires.RL.value = createTire(Left);
    tires.RR.value = createTire(Right);

    Vehicle<VehicleFrame> v(cfg, std::move(tires));

    auto points = getYawMomentDiagramPoints(v, 11, cfg, 20, 2, 20, 1);
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
