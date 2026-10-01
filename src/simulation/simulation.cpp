#include "simulation/simulation.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <vector>

#include "vehicle/aero/aeroSimple.h"
#include "vehicle/differential/differential.h"
#include "vehicle/tire/tirePacejkaV1.h"
#include "vehicle/tire/tirePacejkaV2.h"
#include "vehicle/tire/tireSimple.h"

#define _USE_MATH_DEFINES
#include <math.h>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace {
int workerCount() {
#ifdef _OPENMP
    return omp_get_max_threads();
#else
    return 1;
#endif
}

int workerIndex() {
#ifdef _OPENMP
    return omp_get_thread_num();
#else
    return 0;
#endif
}
}  // namespace

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
std::vector<DiagramSample> Simulation::getYawMomentDiagramPoints(
    const std::function<std::unique_ptr<Vehicle<Frame>>()>& makeVehicle, float speed,
    const Config& cfg, float maxSteeringAngle, float steeringAngleStep, float maxSlipAngle,
    float slipAngleStep, float tolerance, int maxIterations) {
    std::vector<std::unique_ptr<Vehicle<Frame>>> pool;
    for (int i = 0; i < workerCount(); i++) {
        pool.push_back(makeVehicle());
        pool.back()->setSpeed(speed);
    }

    auto solveAt = [&](Vehicle<Frame>& v, float steering, float slip, bool inSteer, bool inSlip) {
        v.setSteeringAngle(Alpha<Frame>(steering * M_PI / 180.f));
        v.setChassisSlipAngle(Alpha<Frame>(slip * M_PI / 180.f));
        PointSolution solution = v.solveDiagramPoint(tolerance, maxIterations, cfg);
        return DiagramSample{steering, slip, inSteer, inSlip, solution};
    };

    std::vector<float> steeringAngles;
    for (float steeringAngle = -maxSteeringAngle; steeringAngle <= maxSteeringAngle;
         steeringAngle += steeringAngleStep) {
        steeringAngles.push_back(steeringAngle);
    }

    std::vector<std::pair<float, std::vector<DiagramSample>>> isolines(steeringAngles.size());
#pragma omp parallel for schedule(dynamic)
    for (size_t line = 0; line < steeringAngles.size(); line++) {
        Vehicle<Frame>& v = *pool[workerIndex()];
        float steeringAngle = steeringAngles[line];
        std::vector<DiagramSample> samples;
        for (float slip = -maxSlipAngle; slip <= maxSlipAngle; slip += slipAngleStep) {
            samples.push_back(solveAt(v, steeringAngle, slip, true, true));
        }
        isolines[line] = {steeringAngle, std::move(samples)};
    }

    std::vector<DiagramSample> steeringRefined;
    bool refine = cfg.get("Simlation", "refine", 1.0f) > 0.5f;
    if (refine) {
        float latMin = 1e30f, latMax = -1e30f, yawMin = 1e30f, yawMax = -1e30f;
        for (auto& [steering, samples] : isolines) {
            for (const DiagramSample& s : samples) {
                latMin = std::min(latMin, s.solution.latAcc);
                latMax = std::max(latMax, s.solution.latAcc);
                yawMin = std::min(yawMin, s.solution.yawMoment);
                yawMax = std::max(yawMax, s.solution.yawMoment);
            }
        }
        float latRange = std::max(1e-6f, latMax - latMin);
        float yawRange = std::max(1e-6f, yawMax - yawMin);
        auto distance = [&](const DiagramSample& a, const DiagramSample& b) {
            float dl = (a.solution.latAcc - b.solution.latAcc) / latRange;
            float dy = (a.solution.yawMoment - b.solution.yawMoment) / yawRange;
            return std::sqrt(dl * dl + dy * dy);
        };

        std::map<long, std::vector<DiagramSample>> baseBySlip;
        for (auto& [steering, samples] : isolines) {
            for (const DiagramSample& s : samples) {
                baseBySlip[std::lround(s.slip / slipAngleStep)].push_back(s);
            }
        }
        for (auto& [key, group] : baseBySlip) {
            std::sort(group.begin(), group.end(),
                      [](const DiagramSample& a, const DiagramSample& b) {
                          return a.steering < b.steering;
                      });
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

        using MidSolver = std::function<DiagramSample(Vehicle<Frame>&, const DiagramSample&,
                                                      const DiagramSample&)>;
        std::function<void(Vehicle<Frame>&, const DiagramSample&, const DiagramSample&, int,
                           const MidSolver&, std::vector<DiagramSample>&)>
            bisect = [&](Vehicle<Frame>& v, const DiagramSample& left, const DiagramSample& right,
                         int depth, const MidSolver& solveMid,
                         std::vector<DiagramSample>& into) -> void {
            if (depth <= 0 || distance(left, right) <= target) return;
            DiagramSample mid = solveMid(v, left, right);
            bisect(v, left, mid, depth - 1, solveMid, into);
            into.push_back(mid);
            bisect(v, mid, right, depth - 1, solveMid, into);
        };

        MidSolver slipMid = [&](Vehicle<Frame>& v, const DiagramSample& l, const DiagramSample& r) {
            return solveAt(v, l.steering, 0.5f * (l.slip + r.slip), true, false);
        };
#pragma omp parallel for schedule(dynamic)
        for (size_t line = 0; line < isolines.size(); line++) {
            Vehicle<Frame>& v = *pool[workerIndex()];
            std::vector<DiagramSample>& samples = isolines[line].second;
            std::vector<DiagramSample> refined{samples[0]};
            for (size_t i = 1; i < samples.size(); i++) {
                bisect(v, samples[i - 1], samples[i], maxDepth, slipMid, refined);
                refined.push_back(samples[i]);
            }
            samples = std::move(refined);
        }

        MidSolver steeringMid = [&](Vehicle<Frame>& v, const DiagramSample& l,
                                    const DiagramSample& r) {
            return solveAt(v, 0.5f * (l.steering + r.steering), l.slip, false, true);
        };
        std::vector<std::vector<DiagramSample>*> groups;
        for (auto& [key, group] : baseBySlip) {
            groups.push_back(&group);
        }
        std::vector<std::vector<DiagramSample>> refinedGroups(groups.size());
#pragma omp parallel for schedule(dynamic)
        for (size_t g = 0; g < groups.size(); g++) {
            Vehicle<Frame>& v = *pool[workerIndex()];
            std::vector<DiagramSample>& group = *groups[g];
            for (size_t i = 1; i < group.size(); i++) {
                bisect(v, group[i - 1], group[i], maxDepth, steeringMid, refinedGroups[g]);
            }
        }
        for (const std::vector<DiagramSample>& refined : refinedGroups) {
            for (const DiagramSample& s : refined) steeringRefined.push_back(s);
        }
    }

    std::vector<DiagramSample> out;
    for (auto& [steering, samples] : isolines) {
        for (const DiagramSample& s : samples) out.push_back(s);
    }
    for (const DiagramSample& s : steeringRefined) out.push_back(s);

    std::sort(out.begin(), out.end(), [](const DiagramSample& a, const DiagramSample& b) {
        return a.steering != b.steering ? a.steering < b.steering : a.slip < b.slip;
    });
    return out;
}

namespace {
struct TireColumn {
    const char* name;
    WheelData<float> PointSolution::* field;
};

constexpr TireColumn TIRE_COLUMNS[] = {
    {"load", &PointSolution::load},           {"slipAngle", &PointSolution::slipAngle},
    {"slipRatio", &PointSolution::slipRatio}, {"Fx", &PointSolution::forceX},
    {"Fy", &PointSolution::forceY},           {"FxCar", &PointSolution::forceXCar},
    {"FyCar", &PointSolution::forceYCar},     {"Mz", &PointSolution::momentZ},
    {"camber", &PointSolution::camber},       {"aeroLoad", &PointSolution::aeroLoad},
};

constexpr const char* WHEEL_SUFFIX[CarConstants::WHEEL_COUNT] = {"FL", "FR", "RL", "RR"};
}  // namespace

void writeDiagramHeader(FILE* f) {
    fprintf(f,
            "steering,slip,latAcc,yawMoment,baseSteering,baseSlip,longAcc,aeroDownforce,aeroDrag,"
            "totalLoad");
    for (const TireColumn& column : TIRE_COLUMNS) {
        for (const char* suffix : WHEEL_SUFFIX) {
            fprintf(f, ",%s_%s", column.name, suffix);
        }
    }
    fprintf(f, "\n");
}

void writeDiagramRow(FILE* f, const DiagramSample& sample) {
    const PointSolution& p = sample.solution;
    fprintf(f, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", sample.steering, sample.slip, p.latAcc, p.yawMoment,
            sample.inSteeringFamily ? 1.0f : 0.0f, sample.inSlipFamily ? 1.0f : 0.0f, p.longAcc,
            p.aeroDownforce, p.aeroDrag, p.totalLoad);
    for (const TireColumn& column : TIRE_COLUMNS) {
        const WheelData<float>& wheels = p.*(column.field);
        for (size_t i = 0; i < CarConstants::WHEEL_COUNT; i++) {
            fprintf(f, ",%f", wheels[i]);
        }
    }
    fprintf(f, "\n");
}

std::vector<DiagramSample> Simulation::run() {
    std::string vehicleFrameStr = cfg.getString("Vehicle", "frame");

    if (vehicleFrameStr == "ISO8855") {
        using VehicleFrame = ISO8855;
        auto makeVehicle = [this]() {
            return std::make_unique<Vehicle<VehicleFrame>>(
                cfg, buildTires<VehicleFrame>(cfg), buildAero<VehicleFrame>(cfg),
                buildSteeringTable<VehicleFrame>(cfg), buildDifferential<VehicleFrame>(cfg));
        };
        return getYawMomentDiagramPoints<VehicleFrame>(
            makeVehicle, cfg.get("Simlation", "speed"), cfg,
            cfg.get("Simlation", "maxSteeringAngle"), cfg.get("Simlation", "steeringAngleStep"),
            cfg.get("Simlation", "maxSlipAngle"), cfg.get("Simlation", "slipAngleStep"),
            cfg.get("Simlation", "tolerance"), cfg.get("Simlation", "maxIterations"));
    } else if (vehicleFrameStr == "SAE") {
        using VehicleFrame = SAE;
        auto makeVehicle = [this]() {
            return std::make_unique<Vehicle<VehicleFrame>>(
                cfg, buildTires<VehicleFrame>(cfg), buildAero<VehicleFrame>(cfg),
                buildSteeringTable<VehicleFrame>(cfg), buildDifferential<VehicleFrame>(cfg));
        };
        return getYawMomentDiagramPoints<VehicleFrame>(
            makeVehicle, cfg.get("Simlation", "speed"), cfg,
            cfg.get("Simlation", "maxSteeringAngle"), cfg.get("Simlation", "steeringAngleStep"),
            cfg.get("Simlation", "maxSlipAngle"), cfg.get("Simlation", "slipAngleStep"),
            cfg.get("Simlation", "tolerance"), cfg.get("Simlation", "maxIterations"));
    } else {
        throw std::runtime_error("Unknown vehicle frame: " + vehicleFrameStr);
    }
}
