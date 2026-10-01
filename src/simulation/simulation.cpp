#include "simulation/simulation.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>

#include "vehicle/aero/aeroSimple.h"
#include "vehicle/differential/differential.h"
#include "vehicle/tire/tirePacejkaV1.h"
#include "vehicle/tire/tirePacejkaV2.h"
#include "vehicle/tire/tireSimple.h"

#define _USE_MATH_DEFINES
#include <math.h>

template <typename VehicleFrame>
WheelData<Positioned<std::unique_ptr<TireBase<VehicleFrame>>, VehicleFrame>> Simulation::buildTires(
    Config& cfg) {
    WheelData<Positioned<std::unique_ptr<TireBase<VehicleFrame>>, VehicleFrame>> tires;

    std::string tFrameStr = cfg.getString("Tire", "frame");
    std::string impl = cfg.getString("Tire", "implementation");

    auto createTire = [&cfg, &impl,
                       &tFrameStr](Side side) -> std::unique_ptr<TireBase<VehicleFrame>> {
        if (tFrameStr == "ISO8855") {
            using TireFrame = ISO8855;
            if (impl == "Simple")
                return std::make_unique<TireSimple<TireFrame, VehicleFrame>>(cfg, false);
            if (impl == "PacejkaV2")
                return std::make_unique<TirePacejkaV2<TireFrame, VehicleFrame>>(cfg, false, side);
            if (impl == "PacejkaV1")
                return std::make_unique<TirePacejkaV1<TireFrame, VehicleFrame>>(cfg, false, side);
        } else if (tFrameStr == "SAE") {
            using TireFrame = SAE;
            if (impl == "Simple")
                return std::make_unique<TireSimple<TireFrame, VehicleFrame>>(cfg, false);
            if (impl == "PacejkaV2")
                return std::make_unique<TirePacejkaV2<TireFrame, VehicleFrame>>(cfg, false, side);
            if (impl == "PacejkaV1")
                return std::make_unique<TirePacejkaV1<TireFrame, VehicleFrame>>(cfg, false, side);
        }
        throw std::runtime_error("Unknown tire implementation or frame");
    };

    tires.FL.value = createTire(Left);
    tires.FR.value = createTire(Right);
    tires.RL.value = createTire(Left);
    tires.RR.value = createTire(Right);

    return tires;
}

template <typename VehicleFrame>
Positioned<std::unique_ptr<AeroBase<VehicleFrame>>, VehicleFrame> Simulation::buildAero(
    Config& cfg) {
    std::string aFrameStr = cfg.getString("Aero", "frame");
    std::string impl = cfg.getString("Aero", "implementation");
    Positioned<std::unique_ptr<AeroBase<VehicleFrame>>, VehicleFrame> aero;
    if (aFrameStr == "ISO8855") {
        using AeroFrame = ISO8855;
        if (impl == "Simple")
            aero.value = std::make_unique<AeroSimple<AeroFrame, VehicleFrame>>(cfg);
    } else if (aFrameStr == "SAE") {
        using AeroFrame = SAE;
        if (impl == "Simple")
            aero.value = std::make_unique<AeroSimple<AeroFrame, VehicleFrame>>(cfg);
    }
    if (!aero.value) throw std::runtime_error("Unknown aero implementation or frame");
    aero.position = cfg.getVec<VehicleFrame>("Aero", "claPosition");
    return aero;
}

template <typename VehicleFrame>
std::unique_ptr<SteeringTableBase<VehicleFrame>> Simulation::buildSteeringTable(Config& cfg) {
    std::string frameStr = cfg.getString("Vehicle", "steeringTable.frame", "ISO8855");
    if (frameStr == "ISO8855") {
        return std::make_unique<SteeringTable<ISO8855, VehicleFrame>>(cfg);
    } else if (frameStr == "SAE") {
        return std::make_unique<SteeringTable<SAE, VehicleFrame>>(cfg);
    }
    throw std::runtime_error("Unknown steeringTable frame: " + frameStr);
}

template <typename VehicleFrame>
std::unique_ptr<DifferentialBase<VehicleFrame>> Simulation::buildDifferential(Config& cfg) {
    std::string impl = cfg.getString("Differential", "implementation", "Open");
    if (impl == "Open") return std::make_unique<OpenDifferential<VehicleFrame>>();
    throw std::runtime_error("Unknown differential implementation: " + impl);
}

template <typename Frame>
std::vector<std::array<float, 6>> Simulation::getYawMomentDiagramPoints(
    Vehicle<Frame>& v, float speed, const Config& cfg, float maxSteeringAngle,
    float steeringAngleStep, float maxSlipAngle, float slipAngleStep, float tolerance,
    int maxIterations) {
    v.setSpeed(speed);

    struct Sample {
        float steering;
        float slip;
        float latAcc;
        float yawMoment;
        bool inSteeringFamily;
        bool inSlipFamily;
    };

    auto solveAt = [&](float steering, float slip, bool inSteer, bool inSlip) {
        v.setSteeringAngle(Alpha<Frame>(steering * M_PI / 180.f));
        v.setChassisSlipAngle(Alpha<Frame>(slip * M_PI / 180.f));
        std::array<float, 2> point = v.calculateLatAccAndYawMoment(tolerance, maxIterations, cfg);
        return Sample{steering, slip, point[0], point[1], inSteer, inSlip};
    };

    std::vector<std::pair<float, std::vector<Sample>>> isolines;
    auto sweepSteering = [&](float steeringAngle) {
        std::vector<Sample> samples;
        for (float slip = -maxSlipAngle; slip <= maxSlipAngle; slip += slipAngleStep) {
            samples.push_back(solveAt(steeringAngle, slip, true, true));
        }
        isolines.push_back({steeringAngle, std::move(samples)});
    };

    for (float steeringAngle = -maxSteeringAngle; steeringAngle <= maxSteeringAngle;
         steeringAngle += steeringAngleStep) {
        sweepSteering(steeringAngle);
    }

    std::vector<Sample> steeringRefined;
    bool refine = cfg.get("Simlation", "refine", 1.0f) > 0.5f;
    if (refine) {
        float latMin = 1e30f, latMax = -1e30f, yawMin = 1e30f, yawMax = -1e30f;
        for (auto& [steering, samples] : isolines) {
            for (const Sample& s : samples) {
                latMin = std::min(latMin, s.latAcc);
                latMax = std::max(latMax, s.latAcc);
                yawMin = std::min(yawMin, s.yawMoment);
                yawMax = std::max(yawMax, s.yawMoment);
            }
        }
        float latRange = std::max(1e-6f, latMax - latMin);
        float yawRange = std::max(1e-6f, yawMax - yawMin);
        auto distance = [&](const Sample& a, const Sample& b) {
            float dl = (a.latAcc - b.latAcc) / latRange;
            float dy = (a.yawMoment - b.yawMoment) / yawRange;
            return std::sqrt(dl * dl + dy * dy);
        };

        std::map<long, std::vector<Sample>> baseBySlip;
        for (auto& [steering, samples] : isolines) {
            for (const Sample& s : samples) {
                baseBySlip[std::lround(s.slip / slipAngleStep)].push_back(s);
            }
        }
        for (auto& [key, group] : baseBySlip) {
            std::sort(group.begin(), group.end(),
                      [](const Sample& a, const Sample& b) { return a.steering < b.steering; });
        }

        std::vector<float> gaps;
        for (auto& [steering, samples] : isolines) {
            for (size_t i = 1; i < samples.size(); i++) {
                gaps.push_back(distance(samples[i - 1], samples[i]));
            }
        }
        for (auto& [key, group] : baseBySlip) {
            for (size_t i = 1; i < group.size(); i++) {
                gaps.push_back(distance(group[i - 1], group[i]));
            }
        }
        std::sort(gaps.begin(), gaps.end());
        float median = gaps.empty() ? 0.0f : gaps[gaps.size() / 2];
        float target = cfg.get("Simlation", "refineFactor", 1.5f) * median;
        int maxDepth = (int)cfg.get("Simlation", "refineMaxDepth", 4.0f);

        std::function<void(const Sample&, const Sample&, int,
                           const std::function<Sample(const Sample&, const Sample&)>&,
                           std::vector<Sample>&)>
            bisect = [&](const Sample& left, const Sample& right, int depth,
                         const std::function<Sample(const Sample&, const Sample&)>& solveMid,
                         std::vector<Sample>& into) -> void {
            if (depth <= 0 || distance(left, right) <= target) return;
            Sample mid = solveMid(left, right);
            bisect(left, mid, depth - 1, solveMid, into);
            into.push_back(mid);
            bisect(mid, right, depth - 1, solveMid, into);
        };

        std::function<Sample(const Sample&, const Sample&)> slipMid = [&](const Sample& l,
                                                                          const Sample& r) {
            return solveAt(l.steering, 0.5f * (l.slip + r.slip), true, false);
        };
        for (auto& [steering, samples] : isolines) {
            std::vector<Sample> refined{samples[0]};
            for (size_t i = 1; i < samples.size(); i++) {
                bisect(samples[i - 1], samples[i], maxDepth, slipMid, refined);
                refined.push_back(samples[i]);
            }
            samples = std::move(refined);
        }

        std::function<Sample(const Sample&, const Sample&)> steeringMid = [&](const Sample& l,
                                                                              const Sample& r) {
            return solveAt(0.5f * (l.steering + r.steering), l.slip, false, true);
        };
        for (auto& [key, group] : baseBySlip) {
            for (size_t i = 1; i < group.size(); i++) {
                bisect(group[i - 1], group[i], maxDepth, steeringMid, steeringRefined);
            }
        }
    }

    std::vector<std::array<float, 6>> out;
    auto emit = [&](const Sample& s) {
        out.push_back({s.steering, s.slip, s.latAcc, s.yawMoment, s.inSteeringFamily ? 1.0f : 0.0f,
                       s.inSlipFamily ? 1.0f : 0.0f});
    };
    for (auto& [steering, samples] : isolines) {
        for (const Sample& s : samples) emit(s);
    }
    for (const Sample& s : steeringRefined) emit(s);

    std::sort(out.begin(), out.end(), [](const auto& a, const auto& b) {
        return a[0] != b[0] ? a[0] < b[0] : a[1] < b[1];
    });
    return out;
}

std::vector<std::array<float, 6>> Simulation::run() {
    std::string vehicleFrameStr = cfg.getString("Vehicle", "frame");

    if (vehicleFrameStr == "ISO8855") {
        using VehicleFrame = ISO8855;
        auto tires = buildTires<VehicleFrame>(cfg);
        auto aero = buildAero<VehicleFrame>(cfg);
        auto steeringTable = buildSteeringTable<VehicleFrame>(cfg);
        auto differential = buildDifferential<VehicleFrame>(cfg);
        Vehicle<VehicleFrame> v(cfg, std::move(tires), std::move(aero), std::move(steeringTable),
                                std::move(differential));
        return getYawMomentDiagramPoints(
            v, cfg.get("Simlation", "speed"), cfg, cfg.get("Simlation", "maxSteeringAngle"),
            cfg.get("Simlation", "steeringAngleStep"), cfg.get("Simlation", "maxSlipAngle"),
            cfg.get("Simlation", "slipAngleStep"), cfg.get("Simlation", "tolerance"),
            cfg.get("Simlation", "maxIterations"));
    } else if (vehicleFrameStr == "SAE") {
        using VehicleFrame = SAE;
        auto tires = buildTires<VehicleFrame>(cfg);
        auto aero = buildAero<VehicleFrame>(cfg);
        auto steeringTable = buildSteeringTable<VehicleFrame>(cfg);
        auto differential = buildDifferential<VehicleFrame>(cfg);
        Vehicle<VehicleFrame> v(cfg, std::move(tires), std::move(aero), std::move(steeringTable),
                                std::move(differential));
        return getYawMomentDiagramPoints(
            v, cfg.get("Simlation", "speed"), cfg, cfg.get("Simlation", "maxSteeringAngle"),
            cfg.get("Simlation", "steeringAngleStep"), cfg.get("Simlation", "maxSlipAngle"),
            cfg.get("Simlation", "slipAngleStep"), cfg.get("Simlation", "tolerance"),
            cfg.get("Simlation", "maxIterations"));
    } else {
        throw std::runtime_error("Unknown vehicle frame: " + vehicleFrameStr);
    }
}
