#include "simulation/simulation.h"
#include "vehicle/tire/tirePacejkaV2.h"
#include "vehicle/tire/tirePacejkaV1.h"
#include "vehicle/tire/tireSimple.h"

#define _USE_MATH_DEFINES
#include <math.h>

    
template <typename VehicleFrame>
WheelData<Positioned<std::unique_ptr<TireBase<VehicleFrame>>, VehicleFrame>> Simulation::buildTires(Config& cfg) {
    WheelData<Positioned<std::unique_ptr<TireBase<VehicleFrame>>, VehicleFrame>> tires;
    
    std::string tFrameStr = cfg.getString("Tire", "frame");
    std::string impl = cfg.getString("Tire", "implementation");

    auto createTire = [&cfg, &impl, &tFrameStr](Side side) -> std::unique_ptr<TireBase<VehicleFrame>> {
        if (tFrameStr == "ISO8855") {
            using TireFrame = ISO8855;
            if (impl == "Simple") return std::make_unique<TireSimple<TireFrame, VehicleFrame>>(cfg, false);
            if (impl == "PacejkaV2") return std::make_unique<TirePacejkaV2<TireFrame, VehicleFrame>>(cfg, false, side);
            if (impl == "PacejkaV1") return std::make_unique<TirePacejkaV1<TireFrame, VehicleFrame>>(cfg, false, side);
        } else if (tFrameStr == "SAE") {
            using TireFrame = SAE;
            if (impl == "Simple") return std::make_unique<TireSimple<TireFrame, VehicleFrame>>(cfg, false);
            if (impl == "PacejkaV2") return std::make_unique<TirePacejkaV2<TireFrame, VehicleFrame>>(cfg, false, side);
            if (impl == "PacejkaV1") return std::make_unique<TirePacejkaV1<TireFrame, VehicleFrame>>(cfg, false, side);
        }
        throw std::runtime_error("Unknown tire implementation or frame");
    };

    tires.FL.value = createTire(Left);
    tires.FR.value = createTire(Right);
    tires.RL.value = createTire(Left);
    tires.RR.value = createTire(Right);
    
    return tires;
}

template <typename Frame>
std::vector<std::array<float, 4>> Simulation::getYawMomentDiagramPoints(
    Vehicle<Frame>& v, float speed, const Config& cfg,
    float maxSteeringAngle, float steeringAngleStep, float maxSlipAngle,
    float slipAngleStep, float tolerance, int maxIterations) {
    std::vector<std::array<float, 4>> out;

    v.setSpeed(speed);

    for (float steeringAngle = -maxSteeringAngle; steeringAngle <= maxSteeringAngle;
        steeringAngle += steeringAngleStep) {
        v.setSteeringAngle(Alpha<Frame>(steeringAngle * M_PI / 180.f));
        // Sweep from slip=0 outward in both directions to maintain branch continuity
        std::vector<std::array<float, 4>> steeringPoints;

        for (float s = 0; s <= maxSlipAngle; s += slipAngleStep) {
            v.setChassisSlipAngle(Alpha<Frame>(s * M_PI / 180.f));
            auto dp = v.calculateLatAccAndYawMoment(tolerance, maxIterations, cfg);
            steeringPoints.push_back({steeringAngle, s, dp[0], dp[1]});
        }

        for (float s = -slipAngleStep; s >= -maxSlipAngle; s -= slipAngleStep) {
            v.setChassisSlipAngle(Alpha<Frame>(s * M_PI / 180.f));
            auto dp = v.calculateLatAccAndYawMoment(tolerance, maxIterations, cfg);
            steeringPoints.push_back({steeringAngle, s, dp[0], dp[1]});
        }

        std::sort(steeringPoints.begin(), steeringPoints.end(),
                  [](const auto& a, const auto& b) { return a[1] < b[1]; });
        for (const auto& p : steeringPoints) out.push_back(p);
    }
    return out;
}

std::vector<std::array<float, 4>> Simulation::run(){
    std::string vehicleFrameStr = cfg.getString("Vehicle", "frame");
        
    if(vehicleFrameStr == "ISO8855") {
        using VehicleFrame = ISO8855;
        auto tires = buildTires<VehicleFrame>(cfg);
        Vehicle<VehicleFrame> v(cfg, std::move(tires));
        return getYawMomentDiagramPoints(v, cfg.get("Simlation", "speed"), cfg, cfg.get("Simlation", "maxSteeringAngle"), cfg.get("Simlation", "steeringAngleStep"), cfg.get("Simlation", "maxSlipAngle"), cfg.get("Simlation", "slipAngleStep"), cfg.get("Simlation", "tolerance"), cfg.get("Simlation", "maxIterations"));
    } else if(vehicleFrameStr == "SAE") {
        using VehicleFrame = SAE;
        auto tires = buildTires<VehicleFrame>(cfg);
        Vehicle<VehicleFrame> v(cfg, std::move(tires));
        return getYawMomentDiagramPoints(v, cfg.get("Simlation", "speed"), cfg, cfg.get("Simlation", "maxSteeringAngle"), cfg.get("Simlation", "steeringAngleStep"), cfg.get("Simlation", "maxSlipAngle"), cfg.get("Simlation", "slipAngleStep"), cfg.get("Simlation", "tolerance"), cfg.get("Simlation", "maxIterations"));
    } else {
        throw std::runtime_error("Unknown vehicle frame: " + vehicleFrameStr);
    }
}
