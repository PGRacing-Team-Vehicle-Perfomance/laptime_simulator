#include "simulation/simulation.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
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

using Clock = std::chrono::steady_clock;

double secondsSince(Clock::time_point start) {
    return std::chrono::duration<double>(Clock::now() - start).count();
}

void printProgressBar(const char* label, size_t done, size_t total) {
    constexpr int width = 30;
    float frac = total > 0 ? static_cast<float>(done) / static_cast<float>(total) : 1.0f;
    int filled = static_cast<int>(frac * width);
    fprintf(stderr, "\r[%-12s] ", label);
    for (int i = 0; i < width; i++) fputc(i < filled ? '#' : '-', stderr);
    fprintf(stderr, " %3.0f%% (%zu/%zu)", frac * 100.0f, done, total);
    if (done >= total) fputc('\n', stderr);
    fflush(stderr);
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
    auto tStart = Clock::now();
    std::vector<std::unique_ptr<Vehicle<Frame>>> pool;
    for (int i = 0; i < workerCount(); i++) {
        pool.push_back(makeVehicle());
        pool.back()->setSpeed(speed);
    }
    double poolSeconds = secondsSince(tStart);

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
    std::vector<float> slipAngles;
    for (float slip = -maxSlipAngle; slip <= maxSlipAngle; slip += slipAngleStep) {
        slipAngles.push_back(slip);
    }
    // Each slip isoline solves steeringAngles[centerSteering] first, so an empty
    // steering grid (pathological config) would dereference past the vector.
    if (steeringAngles.empty() || slipAngles.empty()) return {};
    size_t centerSteering = 0;
    for (size_t i = 1; i < steeringAngles.size(); i++) {
        if (std::abs(steeringAngles[i]) < std::abs(steeringAngles[centerSteering]))
            centerSteering = i;
    }

    auto tBaseStart = Clock::now();
    std::vector<std::pair<float, std::vector<DiagramSample>>> isolines(slipAngles.size());
    std::vector<double> lineSeconds(slipAngles.size(), 0.0);
    size_t baseDone = 0;
    printProgressBar("base sweep", 0, slipAngles.size());  // show the phase before line 1 finishes
#pragma omp parallel for schedule(dynamic)
    for (size_t line = 0; line < slipAngles.size(); line++) {
        auto tLine = Clock::now();
        Vehicle<Frame>& v = *pool[workerIndex()];
        float slip = slipAngles[line];
        DiagramSample center = solveAt(v, steeringAngles[centerSteering], slip, true, true);
        std::vector<DiagramSample> forward, backward;
        float forwardLat = center.solution.latAcc;
        for (size_t i = centerSteering + 1; i < steeringAngles.size(); i++) {
            DiagramSample sample = solveAt(v, steeringAngles[i], slip, true, true);
            if (!std::isfinite(sample.solution.latAcc) || sample.solution.latAcc < forwardLat)
                break;
            forward.push_back(sample);
            forwardLat = sample.solution.latAcc;
        }
        float backwardLat = center.solution.latAcc;
        for (size_t i = centerSteering; i-- > 0;) {
            DiagramSample sample = solveAt(v, steeringAngles[i], slip, true, true);
            if (!std::isfinite(sample.solution.latAcc) || sample.solution.latAcc > backwardLat)
                break;
            backward.push_back(sample);
            backwardLat = sample.solution.latAcc;
        }
        std::vector<DiagramSample> samples;
        samples.reserve(backward.size() + 1 + forward.size());
        for (auto it = backward.rbegin(); it != backward.rend(); ++it) samples.push_back(*it);
        samples.push_back(center);
        for (DiagramSample& sample : forward) samples.push_back(sample);
        isolines[line] = {slip, std::move(samples)};
        lineSeconds[line] = secondsSince(tLine);
#pragma omp critical
        printProgressBar("base sweep", ++baseDone, slipAngles.size());
    }
    double baseSeconds = secondsSince(tBaseStart);
    size_t basePoints = 0;
    for (const auto& [slip, samples] : isolines) basePoints += samples.size();

    double prepSeconds = 0.0, slipRefineSeconds = 0.0, steerRefineSeconds = 0.0;
    size_t slipAdded = 0, steerAdded = 0;
    std::vector<DiagramSample> slipRefined;
    bool refine = cfg.getString("Refine", "enabled", "true") == "true";
    if (refine) {
        auto tPrep = Clock::now();
        float latMin = 1e30f, latMax = -1e30f, yawMin = 1e30f, yawMax = -1e30f;
        for (auto& [slip, samples] : isolines) {
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

        std::map<long, std::vector<DiagramSample>> baseBySteering;
        for (auto& [slip, samples] : isolines) {
            for (const DiagramSample& s : samples) {
                baseBySteering[std::lround(s.steering / steeringAngleStep)].push_back(s);
            }
        }
        for (auto& [key, group] : baseBySteering) {
            std::sort(
                group.begin(), group.end(),
                [](const DiagramSample& a, const DiagramSample& b) { return a.slip < b.slip; });
        }

        std::vector<float> gaps;
        for (auto& [slip, samples] : isolines) {
            for (size_t i = 1; i < samples.size(); i++) {
                gaps.push_back(distance(samples[i - 1], samples[i]));
            }
        }
        for (auto& [key, group] : baseBySteering) {
            for (size_t i = 1; i < group.size(); i++) {
                gaps.push_back(distance(group[i - 1], group[i]));
            }
        }
        std::sort(gaps.begin(), gaps.end());
        float median = gaps.empty() ? 0.0f : gaps[gaps.size() / 2];
        float target = cfg.get("Refine", "factor", 1.5f) * median;
        int maxDepth = (int)cfg.get("Refine", "maxDepth", 4.0f);

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

        MidSolver steeringMid = [&](Vehicle<Frame>& v, const DiagramSample& l,
                                    const DiagramSample& r) {
            return solveAt(v, 0.5f * (l.steering + r.steering), l.slip, false, true);
        };
        prepSeconds = secondsSince(tPrep);

        auto tSteer = Clock::now();
        size_t steerDone = 0;
        printProgressBar("steer refine", 0, isolines.size());
#pragma omp parallel for schedule(dynamic)
        for (size_t line = 0; line < isolines.size(); line++) {
            Vehicle<Frame>& v = *pool[workerIndex()];
            std::vector<DiagramSample>& samples = isolines[line].second;
            std::vector<DiagramSample> refined{samples[0]};
            for (size_t i = 1; i < samples.size(); i++) {
                bisect(v, samples[i - 1], samples[i], maxDepth, steeringMid, refined);
                refined.push_back(samples[i]);
            }
            samples = std::move(refined);
#pragma omp critical
            printProgressBar("steer refine", ++steerDone, isolines.size());
        }
        steerRefineSeconds = secondsSince(tSteer);
        size_t afterSteer = 0;
        for (const auto& [slip, samples] : isolines) afterSteer += samples.size();
        steerAdded = afterSteer - basePoints;

        auto tSlip = Clock::now();
        MidSolver slipMid = [&](Vehicle<Frame>& v, const DiagramSample& l, const DiagramSample& r) {
            return solveAt(v, l.steering, 0.5f * (l.slip + r.slip), true, false);
        };
        std::vector<std::vector<DiagramSample>*> groups;
        for (auto& [key, group] : baseBySteering) {
            groups.push_back(&group);
        }
        std::vector<std::vector<DiagramSample>> refinedGroups(groups.size());
        size_t slipDone = 0;
        printProgressBar("slip refine", 0, groups.size());
#pragma omp parallel for schedule(dynamic)
        for (size_t g = 0; g < groups.size(); g++) {
            Vehicle<Frame>& v = *pool[workerIndex()];
            std::vector<DiagramSample>& group = *groups[g];
            for (size_t i = 1; i < group.size(); i++) {
                bisect(v, group[i - 1], group[i], maxDepth, slipMid, refinedGroups[g]);
            }
#pragma omp critical
            printProgressBar("slip refine", ++slipDone, groups.size());
        }
        for (const std::vector<DiagramSample>& refined : refinedGroups) {
            for (const DiagramSample& s : refined) slipRefined.push_back(s);
        }
        slipRefineSeconds = secondsSince(tSlip);
        slipAdded = slipRefined.size();
    }

    auto tAssemble = Clock::now();
    std::vector<DiagramSample> out;
    for (auto& [slip, samples] : isolines) {
        for (const DiagramSample& s : samples) out.push_back(s);
    }
    for (const DiagramSample& s : slipRefined) out.push_back(s);

    std::sort(out.begin(), out.end(), [](const DiagramSample& a, const DiagramSample& b) {
        return a.steering != b.steering ? a.steering < b.steering : a.slip < b.slip;
    });

    double assembleSeconds = secondsSince(tAssemble);
    double totalSeconds = secondsSince(tStart);
    double points = out.empty() ? 1.0 : static_cast<double>(out.size());

    double lineMin = lineSeconds.empty() ? 0.0 : lineSeconds.front();
    double lineMax = 0.0, lineSum = 0.0;
    for (double s : lineSeconds) {
        lineMin = std::min(lineMin, s);
        lineMax = std::max(lineMax, s);
        lineSum += s;
    }
    double lineAvg = lineSeconds.empty() ? 0.0 : lineSum / lineSeconds.size();

    size_t evaluations = 0;
    double loadSeconds = 0, slipAngleSeconds = 0, tireForceSeconds = 0, axleSolveSeconds = 0;
    for (const auto& v : pool) {
        evaluations += v->solverEvaluations;
        loadSeconds += v->loadSeconds;
        slipAngleSeconds += v->slipAngleSeconds;
        tireForceSeconds += v->tireForceSeconds;
        axleSolveSeconds += v->axleSolveSeconds;
    }
    double solverCpu = loadSeconds + slipAngleSeconds + tireForceSeconds;
    auto cpuShare = [&](double seconds) {
        return solverCpu > 0 ? 100.0 * seconds / solverCpu : 0.0;
    };

    size_t nonConverged = 0;
    for (const DiagramSample& s : out) {
        if (!std::isfinite(s.solution.latAcc)) nonConverged++;
    }

    auto share = [&](double seconds) {
        return totalSeconds > 0 ? 100.0 * seconds / totalSeconds : 0.0;
    };
    fprintf(stderr,
            "[sim] threads=%d  pts=%zu  total=%.3fs (%.0f pts/s)  evals=%zu (%.1f/pt)  "
            "non-converged=%zu (%.1f%%)\n"
            "      pool=%.3fs (%.0f%%)  base=%zu pts %.3fs (%.0f%%)  prep=%.3fs (%.0f%%)  "
            "slip-refine=+%zu pts %.3fs (%.0f%%)  steer-refine=+%zu pts %.3fs (%.0f%%)  "
            "assemble=%.3fs (%.0f%%)\n"
            "      base line time: min=%.1fms avg=%.1fms max=%.1fms over %zu lines\n",
            workerCount(), out.size(), totalSeconds, out.size() / totalSeconds, evaluations,
            evaluations / points, nonConverged, 100.0 * nonConverged / points, poolSeconds,
            share(poolSeconds), basePoints, baseSeconds, share(baseSeconds), prepSeconds,
            share(prepSeconds), slipAdded, slipRefineSeconds, share(slipRefineSeconds), steerAdded,
            steerRefineSeconds, share(steerRefineSeconds), assembleSeconds, share(assembleSeconds),
            lineMin * 1000, lineAvg * 1000, lineMax * 1000, lineSeconds.size());
    double lateralTireSeconds = tireForceSeconds - axleSolveSeconds;
    fprintf(stderr,
            "      solver cpu (sum over %d threads): longitudinal-axle=%.2fs (%.0f%%)  "
            "lateral-tire=%.2fs (%.0f%%)  loads+aero=%.2fs (%.0f%%)  slip-angles=%.2fs (%.0f%%)\n",
            workerCount(), axleSolveSeconds, cpuShare(axleSolveSeconds), lateralTireSeconds,
            cpuShare(lateralTireSeconds), loadSeconds, cpuShare(loadSeconds), slipAngleSeconds,
            cpuShare(slipAngleSeconds));
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
            makeVehicle, cfg.get("Sweep", "speed"), cfg, cfg.get("Sweep", "maxSteeringAngle"),
            cfg.get("Sweep", "steeringAngleStep"), cfg.get("Sweep", "maxSlipAngle"),
            cfg.get("Sweep", "slipAngleStep"), cfg.get("Solver", "tolerance"),
            cfg.get("Solver", "maxIterations"));
    } else if (vehicleFrameStr == "SAE") {
        using VehicleFrame = SAE;
        auto makeVehicle = [this]() {
            return std::make_unique<Vehicle<VehicleFrame>>(
                cfg, buildTires<VehicleFrame>(cfg), buildAero<VehicleFrame>(cfg),
                buildSteeringTable<VehicleFrame>(cfg), buildDifferential<VehicleFrame>(cfg));
        };
        return getYawMomentDiagramPoints<VehicleFrame>(
            makeVehicle, cfg.get("Sweep", "speed"), cfg, cfg.get("Sweep", "maxSteeringAngle"),
            cfg.get("Sweep", "steeringAngleStep"), cfg.get("Sweep", "maxSlipAngle"),
            cfg.get("Sweep", "slipAngleStep"), cfg.get("Solver", "tolerance"),
            cfg.get("Solver", "maxIterations"));
    } else {
        throw std::runtime_error("Unknown vehicle frame: " + vehicleFrameStr);
    }
}

template <typename Frame>
void Simulation::dumpTireFrame(FILE* f) {
    auto tires = buildTires<Frame>(cfg);
    TireBase<Frame>& tire = *tires.FL.value;
    constexpr float degToRad = static_cast<float>(M_PI) / 180.0f;
    float loads[] = {250.0f, 500.0f, 750.0f, 1000.0f, 1500.0f};
    for (float load : loads) {
        for (float kappa = -0.3f; kappa <= 0.3001f; kappa += 0.005f) {
            tire.calculate(load, Alpha<Frame>(0.0f), kappa, Gamma<Frame>(0.0f));
            fprintf(f, "slipRatio,%.0f,%.5f,%f,%f,%f\n", load, kappa, tire.getForce().value.x.v,
                    tire.getForce().value.y.v, tire.getTorque().z.v);
        }
    }
    for (float load : loads) {
        for (float deg = -20.0f; deg <= 20.001f; deg += 0.5f) {
            tire.calculate(load, Alpha<Frame>(deg * degToRad), 0.0f, Gamma<Frame>(0.0f));
            fprintf(f, "slipAngle,%.0f,%.5f,%f,%f,%f\n", load, deg, tire.getForce().value.x.v,
                    tire.getForce().value.y.v, tire.getTorque().z.v);
        }
    }
}

void Simulation::dumpTireModel(const std::string& path) {
    FILE* f = fopen(path.c_str(), "w");
    if (!f) {
        fprintf(stderr, "Failed to open %s for writing\n", path.c_str());
        return;
    }
    fprintf(f, "sweep,load,input,Fx,Fy,Mz\n");
    std::string frameStr = cfg.getString("Vehicle", "frame");
    if (frameStr == "ISO8855") {
        dumpTireFrame<ISO8855>(f);
    } else if (frameStr == "SAE") {
        dumpTireFrame<SAE>(f);
    } else {
        fclose(f);
        throw std::runtime_error("Unknown vehicle frame: " + frameStr);
    }
    fclose(f);
    printf("Wrote %s\n", path.c_str());
}
