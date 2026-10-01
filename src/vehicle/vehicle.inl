#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <numbers>

#define _USE_MATH_DEFINES
#include <math.h>

#include "config/config.h"
#include "coordTypes.h"
#include "vehicle/aero/aero.h"
#include "vehicle/slipSolve.h"
#include "vehicle/tire/tire.h"
#include "vehicle/vehicleHelper.h"

constexpr int longitudinalRelaxIterations = 8;
constexpr int newtonIterations = 12;
constexpr float newtonStepScale = 10.0f;
constexpr float newtonSingularJacobianEpsilon = 1e-6f;

template <typename Residual>
inline float bracketedRoot(Residual residual, float warmCenter, float fullLo, float fullHi,
                           float initialStep, float tolerance, int maxIterations) {
    float lo = std::max(fullLo, warmCenter - initialStep);
    float hi = std::min(fullHi, warmCenter + initialStep);
    float fLo = residual(lo);
    float fHi = residual(hi);
    float step = initialStep;
    while (fLo * fHi > 0 && (lo > fullLo || hi < fullHi)) {
        step *= 2.0f;
        lo = std::max(fullLo, warmCenter - step);
        hi = std::min(fullHi, warmCenter + step);
        fLo = residual(lo);
        fHi = residual(hi);
    }
    if (fLo * fHi > 0) return std::numeric_limits<float>::quiet_NaN();

    float root = 0.5f * (lo + hi);
    int retainedSide = 0;
    for (int iter = 0; iter < maxIterations && hi - lo > tolerance; iter++) {
        root = (lo * fHi - hi * fLo) / (fHi - fLo);
        float fRoot = residual(root);
        if (fRoot * fHi > 0) {
            hi = root;
            fHi = fRoot;
            if (retainedSide == -1) fLo *= 0.5f;
            retainedSide = -1;
        } else if (fRoot * fLo > 0) {
            lo = root;
            fLo = fRoot;
            if (retainedSide == 1) fHi *= 0.5f;
            retainedSide = 1;
        } else {
            break;
        }
    }
    return root;
}

template <typename Frame>
Vehicle<Frame>::Vehicle(const Config& config,
                        WheelData<Positioned<std::unique_ptr<TireBase<Frame>>, Frame>>&& tires,
                        Positioned<std::unique_ptr<AeroBase<Frame>>, Frame>&& aero,
                        std::unique_ptr<SteeringTableBase<Frame>>&& steeringTable,
                        std::unique_ptr<DifferentialBase<Frame>>&& differential)
    : rollCenterHeightFront(config.get("Vehicle", "rollCenterHeightFront")),
      rollCenterHeightBack(config.get("Vehicle", "rollCenterHeightBack")),
      frontTrackWidth(config.get("Vehicle", "frontTrackWidth")),
      rearTrackWidth(config.get("Vehicle", "rearTrackWidth")),
      trackDistance(config.get("Vehicle", "trackDistance")),
      toeAngle(config.getAlphaWheelData<Frame>("Vehicle", "toeAngle")),
      camber(config.getGammaWheelData<Frame>("Vehicle", "camber")),
      suspendedMassAtWheels(config.getWheelData<float>("Vehicle", "suspendedMassAtWheels")),
      nonSuspendedMassAtWheels(config.getWheelData<float>("Vehicle", "nonSuspendedMassAtWheels")),
      aero(std::move(aero)),
      steeringTable(std::move(steeringTable)),
      tires(std::move(tires)),
      differential(std::move(differential)) {
    combinedNonSuspendedMass = {0, {0, 0, 0}};
    combinedSuspendedMass = {0, {0, 0, 0}};

    for (size_t i = 0; i < CarConstants::WHEEL_COUNT; i++) {
        toeAngle[i].v = mirrorBySide(toeAngle[i].v, WHEEL_SIDE[i]);
        camber[i].v = mirrorBySide(camber[i].v, WHEEL_SIDE[i]);
        combinedNonSuspendedMass.value += nonSuspendedMassAtWheels[i];
        combinedSuspendedMass.value += suspendedMassAtWheels[i];
    }
    tires.FL.position = {0, frontTrackWidth / 2, 0};
    tires.FR.position = {0, -frontTrackWidth / 2, 0};
    tires.RL.position = {trackDistance, rearTrackWidth / 2, 0};
    tires.RR.position = {trackDistance, -rearTrackWidth / 2, 0};

    for (size_t i = 0; i < CarConstants::WHEEL_COUNT; i++) {
        combinedNonSuspendedMass.position.x.v +=
            nonSuspendedMassAtWheels[i] * tires[i].position.x.v;
        combinedNonSuspendedMass.position.y.v +=
            nonSuspendedMassAtWheels[i] * tires[i].position.y.v;

        combinedSuspendedMass.position.x.v += suspendedMassAtWheels[i] * tires[i].position.x.v;
        combinedSuspendedMass.position.y.v += suspendedMassAtWheels[i] * tires[i].position.y.v;
    }
    combinedNonSuspendedMass.position.x.v /= combinedNonSuspendedMass.value;
    combinedNonSuspendedMass.position.y.v /= combinedNonSuspendedMass.value;
    combinedNonSuspendedMass.position.z.v = config.get("Vehicle", "nonSuspendedMassHeight");

    combinedSuspendedMass.position.x.v /= combinedSuspendedMass.value;
    combinedSuspendedMass.position.y.v /= combinedSuspendedMass.value;
    combinedSuspendedMass.position.z.v = config.get("Vehicle", "suspendedMassHeight");

    combinedTotalMass.value = combinedSuspendedMass.value + combinedNonSuspendedMass.value;
    combinedTotalMass.position = {
        (combinedSuspendedMass.position.x.v * combinedSuspendedMass.value +
         combinedNonSuspendedMass.position.x.v * combinedNonSuspendedMass.value) /
            combinedTotalMass.value,
        (combinedSuspendedMass.position.y.v * combinedSuspendedMass.value +
         combinedNonSuspendedMass.position.y.v * combinedNonSuspendedMass.value) /
            combinedTotalMass.value,
        (combinedSuspendedMass.position.z.v * combinedSuspendedMass.value +
         combinedNonSuspendedMass.position.z.v * combinedNonSuspendedMass.value) /
            combinedTotalMass.value};

    float frontSpringWheelRate = config.get("Vehicle", "frontKspring") /
                                 std::pow(config.get("Vehicle", "frontSpringMotionRatio"), 2);
    float frontTorqueSpring =
        std::pow(frontTrackWidth, 2) * std::tan(M_PI / 180) * frontSpringWheelRate / 2;
    float frontArbTorque = antiRollBarTorque(
        config.get("Vehicle", "frontKarb"), config.getString("Vehicle", "frontKarb.unit", "N/mm"),
        config.get("Vehicle", "frontArbMotionRatio"),
        config.getString("Vehicle", "frontArbMotionRatio.unit", "mm/mm"), frontTrackWidth);
    antiRollStiffnessFront = frontArbTorque + frontTorqueSpring;

    float rearSpringWheelRate = config.get("Vehicle", "rearKspring") /
                                std::pow(config.get("Vehicle", "rearSpringMotionRatio"), 2);
    float rearTorqueSpring =
        std::pow(rearTrackWidth, 2) * std::tan(M_PI / 180) * rearSpringWheelRate / 2;
    float rearArbTorque = antiRollBarTorque(
        config.get("Vehicle", "rearKarb"), config.getString("Vehicle", "rearKarb.unit", "N/mm"),
        config.get("Vehicle", "rearArbMotionRatio"),
        config.getString("Vehicle", "rearArbMotionRatio.unit", "mm/mm"), rearTrackWidth);
    antiRollStiffnessRear = rearArbTorque + rearTorqueSpring;

    driveBiasFront = config.get("Vehicle", "driveBiasFront", 0.0f);
    brakeBiasFront = config.get("Vehicle", "brakeBiasFront", 0.6f);
    dragCoefficientArea = config.get("Aero", "cda", 0.0f);
    airDensityValue = config.get("Environment", "airDensity");
    longEquilibriumEnabled = config.getString("Simlation", "longEquilibrium", "false") == "true";
    targetLongAcc = config.get("Simlation", "targetLongAcc", 0.0f);
    tireCalibrationSlip = config.get("Tire", "calibrationSlipAngle", 90.0f) *
                          config.angleUnitScale("Tire", "calibrationSlipAngle");
    lateralAccBracketG = config.get("Simlation", "latAccBracketG", 4.0f);
}

template <typename Frame>
typename Vehicle<Frame>::SolverStep Vehicle<Frame>::evaluateAt(Y<Frame> testLatAcc,
                                                               const Config& config) {
    state.angularVelocity.z = Z<Frame>{testLatAcc.v / state.velocity.getLength()};
    auto slipAngles = calculateSlipAngles();
    auto loads = totalTireLoads(testLatAcc, config);

    SolverStep step;
    computeTireForces(loads, slipAngles, step);
    step.latAcc = calculateLatAcc(step.tireForcesX, step.tireForcesY);
    longitudinalAccEstimate = calculateBodyLongAcc(step.tireForcesX, step.tireForcesY).v;
    return step;
}

template <typename Frame>
void Vehicle<Frame>::computeTireForces(const WheelData<float>& loads,
                                       const WheelData<Alpha<Frame>>& slipAngles,
                                       SolverStep& step) {
    WheelData<float> slipRatio{};
    if (longEquilibriumEnabled && std::abs(longForceDemand) > minSolveForce) {
        bool driving = longForceDemand > 0;
        float frontBias = driving ? driveBiasFront : brakeBiasFront;
        float frontAxle = longForceDemand * frontBias;
        float rearAxle = longForceDemand * (1.0f - frontBias);
        solveAxle(0, 1, loads, slipAngles, frontAxle, slipRatio.FL, slipRatio.FR);
        solveAxle(2, 3, loads, slipAngles, rearAxle, slipRatio.RL, slipRatio.RR);
    }

    for (size_t i = 0; i < CarConstants::WHEEL_COUNT; i++) {
        tires[i].value->calculate(loads[i], slipAngles[i], slipRatio[i], camber[i]);
        step.tireForcesX[i] = tires[i].value->getForce().value.x;
        step.tireForcesY[i] = tires[i].value->getForce().value.y;
        step.tireMomentsZ[i] = tires[i].value->getTorque().z;
    }

    step.loads = loads;
    step.slipRatios = slipRatio;
}

template <typename Frame>
typename Vehicle<Frame>::SolverStep Vehicle<Frame>::bisectLatAcc(const Config& config,
                                                                 float maxLatAcc, float tolerance,
                                                                 int maxIterations) {
    float frozenLongAcc = longitudinalAccEstimate;
    float frozenDemand = longForceDemand;
    auto residualAt = [&](float testLatAcc) {
        longitudinalAccEstimate = frozenLongAcc;
        longForceDemand = frozenDemand;
        return evaluateAt(Y<Frame>{testLatAcc}, config).latAcc.v - testLatAcc;
    };

    float root = bracketedRoot(residualAt, 0.0f, -maxLatAcc, maxLatAcc, maxLatAcc, tolerance,
                               maxIterations);
    SolverStep step;
    if (std::isnan(root)) {
        step = evaluateAt(Y<Frame>{0}, config);
        step.latAcc = Y<Frame>{root};
        return step;
    }
    longitudinalAccEstimate = frozenLongAcc;
    longForceDemand = frozenDemand;
    step = evaluateAt(Y<Frame>{root}, config);
    step.latAcc = Y<Frame>{root};
    return step;
}

template <typename Frame>
typename Vehicle<Frame>::SolverStep Vehicle<Frame>::bisectDemand(const Config& config,
                                                                 float maxForce, float maxLatAcc,
                                                                 float tolerance, int maxIterations) {
    float frozenLongAcc = longitudinalAccEstimate;
    SolverStep step;
    auto longResidualAt = [&](float testDemand) {
        longitudinalAccEstimate = frozenLongAcc;
        longForceDemand = testDemand;
        step = bisectLatAcc(config, maxLatAcc, tolerance, maxIterations);
        return targetLongAcc - calculatePathLongAcc(step.tireForcesX, step.tireForcesY).v;
    };

    float demandTolerance = combinedTotalMass.value * tolerance;
    float root = bracketedRoot(longResidualAt, 0.0f, -maxForce, maxForce, maxForce,
                               demandTolerance, maxIterations);
    if (std::isnan(root)) {
        float rLo = longResidualAt(-maxForce);
        float rHi = longResidualAt(maxForce);
        if (std::abs(rLo) < std::abs(rHi)) {
            longResidualAt(-maxForce);
        } else {
            longResidualAt(maxForce);
        }
        return step;
    }
    longResidualAt(root);
    return step;
}

template <typename Frame>
typename Vehicle<Frame>::SolverStep Vehicle<Frame>::solveCoupled(const Config& config,
                                                                 float tolerance,
                                                                 int maxIterations) {
    float earthAcc = config.get("Environment", "earthAcc");
    float maxForce = combinedTotalMass.value * earthAcc * 2.0f;
    float maxLatAcc = lateralAccBracketG * earthAcc;

    longitudinalAccEstimate = 0;

    SolverStep step;
    for (int outer = 0; outer < longitudinalRelaxIterations; outer++) {
        float frozenLongAcc = longitudinalAccEstimate;

        auto residuals = [&](float latAcc, float demand, float& latResidual, float& longResidual) {
            longitudinalAccEstimate = frozenLongAcc;
            longForceDemand = demand;
            step = evaluateAt(Y<Frame>{latAcc}, config);
            latResidual = step.latAcc.v - latAcc;
            longResidual =
                calculatePathLongAcc(step.tireForcesX, step.tireForcesY).v - targetLongAcc;
        };

        float latAcc = 0;
        float demand = 0;
        float epsLat = newtonStepScale * tolerance;
        float epsDemand = newtonStepScale * combinedTotalMass.value * tolerance;
        bool converged = false;
        for (int iter = 0; iter < newtonIterations; iter++) {
            float r1, r2;
            residuals(latAcc, demand, r1, r2);
            if (std::abs(r1) < tolerance && std::abs(r2) < tolerance) {
                converged = true;
                break;
            }
            float r1Lat, r2Lat, r1Demand, r2Demand;
            residuals(latAcc + epsLat, demand, r1Lat, r2Lat);
            residuals(latAcc, demand + epsDemand, r1Demand, r2Demand);
            float j11 = (r1Lat - r1) / epsLat, j12 = (r1Demand - r1) / epsDemand;
            float j21 = (r2Lat - r2) / epsLat, j22 = (r2Demand - r2) / epsDemand;
            float det = j11 * j22 - j12 * j21;
            if (std::abs(det) < newtonSingularJacobianEpsilon) break;
            latAcc -= (j22 * r1 - j12 * r2) / det;
            demand -= (j11 * r2 - j21 * r1) / det;
            if (std::abs(latAcc) > maxLatAcc || std::abs(demand) > maxForce) break;
        }

        if (converged) {
            float r1, r2;
            residuals(latAcc, demand, r1, r2);
        } else {
            longitudinalAccEstimate = frozenLongAcc;
            step = bisectDemand(config, maxForce, maxLatAcc, tolerance, maxIterations);
        }

        float bodyLongAcc = calculateBodyLongAcc(step.tireForcesX, step.tireForcesY).v;
        longitudinalAccEstimate = frozenLongAcc + 0.5f * (bodyLongAcc - frozenLongAcc);
        if (std::abs(longitudinalAccEstimate - frozenLongAcc) < tolerance) break;
    }

    return step;
}

template <typename Frame>
AxleWheel<Frame> Vehicle<Frame>::axleWheel(size_t wheel, const WheelData<float>& loads,
                                           const WheelData<Alpha<Frame>>& slipAngles) {
    return AxleWheel<Frame>{[this, wheel, &loads, &slipAngles](float slipRatio) {
        tires[wheel].value->calculate(loads[wheel], slipAngles[wheel], slipRatio, camber[wheel]);
        return tires[wheel].value->getForce().value.x;
    }};
}

template <typename Frame>
void Vehicle<Frame>::solveAxle(size_t leftWheel, size_t rightWheel, const WheelData<float>& loads,
                               const WheelData<Alpha<Frame>>& slipAngles, float axleDemand,
                               float& leftSlipRatio, float& rightSlipRatio) {
    AxleSlipRatios ratios =
        differential->solve(X<Frame>{axleDemand}, axleWheel(leftWheel, loads, slipAngles),
                            axleWheel(rightWheel, loads, slipAngles));
    leftSlipRatio = ratios.left;
    rightSlipRatio = ratios.right;
}

template <typename Frame>
X<Frame> Vehicle<Frame>::calculatePathLongAcc(const WheelData<X<Frame>>& tireForcesX,
                                              const WheelData<Y<Frame>>& tireForcesY) {
    auto vehicleFx = getVehicleFxFromTireForces(tireForcesX, tireForcesY);
    auto vehicleFy = getVehicleFyFromTireForces(tireForcesX, tireForcesY);
    float chassisSlipAngle = std::atan2(state.velocity.y.v, state.velocity.x.v);

    float velocity = state.velocity.getLength();
    float drag = 0.5f * dragCoefficientArea * airDensityValue * velocity * velocity;

    float longForce = -drag;
    for (size_t i = 0; i < CarConstants::WHEEL_COUNT; i++) {
        longForce += vehicleFx[i].v * std::cos(chassisSlipAngle) +
                     vehicleFy[i].v * std::sin(chassisSlipAngle);
    }
    return X<Frame>{longForce / combinedTotalMass.value};
}

template <typename Frame>
X<Frame> Vehicle<Frame>::calculateBodyLongAcc(const WheelData<X<Frame>>& tireForcesX,
                                              const WheelData<Y<Frame>>& tireForcesY) {
    auto vehicleFx = getVehicleFxFromTireForces(tireForcesX, tireForcesY);
    float chassisSlipAngle = std::atan2(state.velocity.y.v, state.velocity.x.v);
    float velocity = state.velocity.getLength();
    float drag = 0.5f * dragCoefficientArea * airDensityValue * velocity * velocity;

    float longForce = -drag * std::cos(chassisSlipAngle);
    for (size_t i = 0; i < CarConstants::WHEEL_COUNT; i++) {
        longForce += vehicleFx[i].v;
    }
    return X<Frame>{longForce / combinedTotalMass.value};
}

template <typename Frame>
typename Vehicle<Frame>::SolverStep Vehicle<Frame>::solveLatAcc(const Config& config,
                                                                float tolerance,
                                                                int maxIterations) {
    longitudinalAccEstimate = 0;
    longForceDemand = 0;
    float maxLatAcc = lateralAccBracketG * config.get("Environment", "earthAcc");

    SolverStep step;
    for (int outer = 0; outer < maxIterations; outer++) {
        float frozenLongAcc = longitudinalAccEstimate;
        step = bisectLatAcc(config, maxLatAcc, tolerance, maxIterations);
        if (std::abs(longitudinalAccEstimate - frozenLongAcc) < tolerance) break;
    }

    return step;
}

template <typename Frame>
PointSolution Vehicle<Frame>::solveDiagramPoint(float tolerance, int maxIterations,
                                                const Config& config) {
    SolverStep step;
    if (longEquilibriumEnabled) {
        step = solveCoupled(config, tolerance, maxIterations);
    } else {
        longForceDemand = 0;
        step = solveLatAcc(config, tolerance, maxIterations);
    }
    Y<Frame> latAcc = step.latAcc;

    state.angularVelocity.z = Z<Frame>{latAcc.v / state.velocity.getLength()};
    auto solutionSlipAngles = calculateSlipAngles();
    float maxSlip = 0;
    for (size_t i = 0; i < CarConstants::WHEEL_COUNT; i++) {
        maxSlip = std::max(maxSlip, std::abs(solutionSlipAngles[i].v));
    }
    if (maxSlip > tireCalibrationSlip) {
        return invalidSolution();
    }

    float yawMomentFromTires = 0;
    for (size_t i = 0; i < CarConstants::WHEEL_COUNT; i++) {
        yawMomentFromTires += step.tireMomentsZ[i].v;
    }

    auto vehicleFx = getVehicleFxFromTireForces(step.tireForcesX, step.tireForcesY);
    auto vehicleFy = getVehicleFyFromTireForces(step.tireForcesX, step.tireForcesY);

    float yawMomentFromFy =
        ((vehicleFy.FL.v + vehicleFy.FR.v) * combinedTotalMass.position.x.v) -
        ((vehicleFy.RL.v + vehicleFy.RR.v) * (trackDistance - combinedTotalMass.position.x.v));

    float yawMomentFromFx =
        (frontTrackWidth / 2) *
            (vehicleFx.FL.v -
             vehicleFx.FR.v) +  // if center of mass is not in the middle of track, this is wrong
        (rearTrackWidth / 2) * (vehicleFx.RL.v - vehicleFx.RR.v);

    // TODO: aero yaw moment
    float yawMoment = yawMomentFromFy + yawMomentFromFx + yawMomentFromTires;
    return assembleSolution(latAcc.v, yawMoment, step, solutionSlipAngles, config);
}

template <typename Frame>
PointSolution Vehicle<Frame>::invalidSolution() {
    float nan = std::numeric_limits<float>::quiet_NaN();
    WheelData<float> nanWheels{nan, nan, nan, nan};
    return {nan,       nan,       nan,       nan,       nan,       nan,       nanWheels, nanWheels,
            nanWheels, nanWheels, nanWheels, nanWheels, nanWheels, nanWheels, nanWheels, nanWheels};
}

template <typename Frame>
PointSolution Vehicle<Frame>::assembleSolution(float latAcc, float yawMoment,
                                               const typename Vehicle<Frame>::SolverStep& step,
                                               const WheelData<Alpha<Frame>>& slipAngles,
                                               const Config& config) {
    constexpr float radToDeg = 180.0f / static_cast<float>(M_PI);
    auto aeroLoads = aeroLoad(config);
    float aeroDownforce = 0;
    for (size_t i = 0; i < CarConstants::WHEEL_COUNT; i++) {
        aeroDownforce += aeroLoads[i];
    }
    Transform<Frame, ISO8855> toIso;
    float aeroDrag = -toIso(aero.value->getForce().value.x).v;
    float totalLoad = 0;
    for (size_t i = 0; i < CarConstants::WHEEL_COUNT; i++) {
        totalLoad += step.loads[i];
    }
    auto vehicleFx = getVehicleFxFromTireForces(step.tireForcesX, step.tireForcesY);
    auto vehicleFy = getVehicleFyFromTireForces(step.tireForcesX, step.tireForcesY);

    PointSolution solution;
    solution.latAcc = latAcc;
    solution.yawMoment = yawMoment;
    solution.longAcc = longitudinalAccEstimate;
    solution.aeroDownforce = aeroDownforce;
    solution.aeroDrag = aeroDrag;
    solution.totalLoad = totalLoad;
    for (size_t i = 0; i < CarConstants::WHEEL_COUNT; i++) {
        solution.load[i] = step.loads[i];
        solution.slipAngle[i] = slipAngles[i].v * radToDeg;
        solution.slipRatio[i] = step.slipRatios[i];
        solution.forceX[i] = step.tireForcesX[i].v;
        solution.forceY[i] = step.tireForcesY[i].v;
        solution.forceXCar[i] = vehicleFx[i].v;
        solution.forceYCar[i] = vehicleFy[i].v;
        solution.momentZ[i] = step.tireMomentsZ[i].v;
        solution.camber[i] = camber[i].v * radToDeg;
        solution.aeroLoad[i] = aeroLoads[i];
    }
    return solution;
}

template <typename Frame>
void Vehicle<Frame>::setSteeringAngle(Alpha<Frame> steeringAngle) {
    state.steeringAngle = steeringAngle;
    auto wheelAngles = steeringTable->lookup(steeringAngle);
    state.wheelAngles.FL = wheelAngles.left;
    state.wheelAngles.FR = wheelAngles.right;
    state.wheelAngles.RL = Alpha<Frame>(0);
    state.wheelAngles.RR = Alpha<Frame>(0);
}

template <typename Frame>
void Vehicle<Frame>::setChassisSlipAngle(Alpha<Frame> chassisSlipAngle) {
    // Use the stored speed rather than re-deriving it from the current velocity:
    // rotating the vector in float drifts its length by ~1 ulp each call, and that
    // drift accumulates across the points a reused vehicle solves (nondeterministic
    // under the parallel schedule).
    state.velocity.x = X<Frame>{speed * std::cos(chassisSlipAngle.v)};
    state.velocity.y = Y<Frame>{speed * std::sin(chassisSlipAngle.v)};
    state.velocity.z = Z<Frame>(0);
}

template <typename Frame>
void Vehicle<Frame>::setSpeed(float speed) {
    this->speed = speed;
    state.velocity.setLength(speed);
}

template <typename Frame>
WheelData<Alpha<Frame>> Vehicle<Frame>::calculateSlipAngles() {
    float massToFront = combinedTotalMass.position.x.v;
    float massToRear = trackDistance - massToFront;

    WheelData<Alpha<Frame>> slipAngle;

    slipAngle.FL = Alpha<Frame>{static_cast<float>(
        std::atan((state.velocity.y.v + state.angularVelocity.z.v * massToFront) /
                  (state.velocity.x.v - state.angularVelocity.z.v * frontTrackWidth / 2.0)) -
        state.wheelAngles.FL.v - toeAngle.FL.v)};

    slipAngle.FR = Alpha<Frame>{static_cast<float>(
        std::atan((state.velocity.y.v + state.angularVelocity.z.v * massToFront) /
                  (state.velocity.x.v + state.angularVelocity.z.v * frontTrackWidth / 2.0)) -
        state.wheelAngles.FR.v - toeAngle.FR.v)};

    slipAngle.RL = Alpha<Frame>{static_cast<float>(
        std::atan((state.velocity.y.v - state.angularVelocity.z.v * massToRear) /
                  (state.velocity.x.v - state.angularVelocity.z.v * rearTrackWidth / 2.0)) -
        state.wheelAngles.RL.v - toeAngle.RL.v)};

    slipAngle.RR = Alpha<Frame>{static_cast<float>(
        std::atan((state.velocity.y.v - state.angularVelocity.z.v * massToRear) /
                  (state.velocity.x.v + state.angularVelocity.z.v * rearTrackWidth / 2.0)) -
        state.wheelAngles.RR.v - toeAngle.RR.v)};

    return slipAngle;
}

template <typename Frame>
Y<Frame> Vehicle<Frame>::calculateLatAcc(const WheelData<X<Frame>>& tireForcesX,
                                         const WheelData<Y<Frame>>& tireForcesY) {
    auto velocityFy = getVelocityFyFromTireForces(tireForcesX, tireForcesY);
    float latForce = 0;
    for (size_t i = 0; i < CarConstants::WHEEL_COUNT; i++) {
        latForce += velocityFy[i].v;
    }
    return Y<Frame>{latForce / combinedTotalMass.value};
}

template <typename Frame>
WheelData<Y<Frame>> Vehicle<Frame>::getVehicleFyFromTireForces(const WheelData<X<Frame>>& tireFx,
                                                               const WheelData<Y<Frame>>& tireFy) {
    WheelData<Y<Frame>> vehicleFy;

    float deltaFL = state.wheelAngles.FL.v;
    float deltaFR = state.wheelAngles.FR.v;

    vehicleFy.FL = Y<Frame>{tireFx.FL.v * std::sin(deltaFL) + tireFy.FL.v * std::cos(deltaFL)};
    vehicleFy.FR = Y<Frame>{tireFx.FR.v * std::sin(deltaFR) + tireFy.FR.v * std::cos(deltaFR)};
    vehicleFy.RL = tireFy.RL;
    vehicleFy.RR = tireFy.RR;

    return vehicleFy;
}

template <typename Frame>
WheelData<X<Frame>> Vehicle<Frame>::getVehicleFxFromTireForces(const WheelData<X<Frame>>& tireFx,
                                                               const WheelData<Y<Frame>>& tireFy) {
    WheelData<X<Frame>> vehicleFx;

    float deltaFL = state.wheelAngles.FL.v;
    float deltaFR = state.wheelAngles.FR.v;

    vehicleFx.FL = X<Frame>{tireFx.FL.v * std::cos(deltaFL) - tireFy.FL.v * std::sin(deltaFL)};
    vehicleFx.FR = X<Frame>{tireFx.FR.v * std::cos(deltaFR) - tireFy.FR.v * std::sin(deltaFR)};
    vehicleFx.RL = tireFx.RL;
    vehicleFx.RR = tireFx.RR;

    return vehicleFx;
}

template <typename Frame>
WheelData<Y<Frame>> Vehicle<Frame>::getVelocityFyFromTireForces(const WheelData<X<Frame>>& tireFx,
                                                                const WheelData<Y<Frame>>& tireFy) {
    WheelData<X<Frame>> vehicleFx = getVehicleFxFromTireForces(tireFx, tireFy);
    WheelData<Y<Frame>> vehicleFy = getVehicleFyFromTireForces(tireFx, tireFy);

    float chassisSlipAngle = std::atan2(state.velocity.y.v, state.velocity.x.v);

    WheelData<Y<Frame>> velocityAlignedFy;
    for (size_t i = 0; i < CarConstants::WHEEL_COUNT; i++) {
        velocityAlignedFy[i] = Y<Frame>{vehicleFy[i].v * std::cos(chassisSlipAngle) -
                                        vehicleFx[i].v * std::sin(chassisSlipAngle)};
    }

    return velocityAlignedFy;
}

template <typename Frame>
WheelData<float> Vehicle<Frame>::totalTireLoads(Y<Frame> latAcc, const Config& config) {
    float earthAcc = config.get("Environment", "earthAcc");
    auto staticLoads = staticLoad(earthAcc);
    auto aeroLoads = aeroLoad(config);
    auto transfer = loadTransfer(latAcc);
    auto longTransfer = loadTransferLongitudinal(longitudinalAccEstimate);
    WheelData<float> tireLoads;
    for (size_t i = 0; i < CarConstants::WHEEL_COUNT; i++) {
        tireLoads[i] = staticLoads[i] + aeroLoads[i] + transfer[i] + longTransfer[i];
    }

    resolveAxleLiftOff(tireLoads.FL, tireLoads.FR);
    resolveAxleLiftOff(tireLoads.RL, tireLoads.RR);

    return tireLoads;
}

template <typename Frame>
WheelData<float> Vehicle<Frame>::loadTransferLongitudinal(float longAcc) {
    float transfer =
        combinedTotalMass.value * longAcc * combinedTotalMass.position.z.v / trackDistance;
    WheelData<float> loads;
    loads.FL = -transfer * 0.5f;
    loads.FR = -transfer * 0.5f;
    loads.RL = transfer * 0.5f;
    loads.RR = transfer * 0.5f;
    return loads;
}

template <typename Frame>
void Vehicle<Frame>::resolveAxleLiftOff(float& leftLoad, float& rightLoad) {
    float axleLoad = std::max(0.f, leftLoad + rightLoad);
    if (leftLoad < 0.f) {
        leftLoad = 0.f;
        rightLoad = axleLoad;
    } else if (rightLoad < 0.f) {
        rightLoad = 0.f;
        leftLoad = axleLoad;
    }
}

template <typename Frame>
WheelData<float> Vehicle<Frame>::staticLoad(float earthAcc) {
    WheelData<float> loads;
    for (int i = 0; i < CarConstants::WHEEL_COUNT; i++) {
        loads[i] = (nonSuspendedMassAtWheels[i] + suspendedMassAtWheels[i]) * earthAcc;
    }
    return loads;
}

template <typename Frame>
WheelData<float> Vehicle<Frame>::distributeForces(float totalForce, float frontDist,
                                                  float leftDist) {
    WheelData<float> forces;
    forces.FL = totalForce * (trackDistance - frontDist) / trackDistance *
                (frontTrackWidth / 2 + leftDist) / frontTrackWidth;
    forces.FR = totalForce * (trackDistance - frontDist) / trackDistance *
                (frontTrackWidth / 2 - leftDist) / frontTrackWidth;
    forces.RL =
        totalForce * frontDist / trackDistance * (rearTrackWidth / 2 + leftDist) / rearTrackWidth;
    forces.RR =
        totalForce * frontDist / trackDistance * (rearTrackWidth / 2 - leftDist) / rearTrackWidth;
    return forces;
}

template <typename Frame>
WheelData<float> Vehicle<Frame>::aeroLoad(const Config& config) {
    float airDensityVal = config.get("Environment", "airDensity");

    aero.value->calculate(state, airDensityVal);
    Transform<Frame, ISO8855> toIso;
    float load = -toIso(aero.value->getForce().value.z).v;
    return distributeForces(load, aero.position.x.v, aero.position.y.v);
}

template <typename Frame>
WheelData<float> Vehicle<Frame>::loadTransfer(Y<Frame> latAcc) {
    float ay = latAcc.v;

    float nonSuspendedMassFront = combinedNonSuspendedMass.value *
                                  (trackDistance - combinedNonSuspendedMass.position.x.v) /
                                  trackDistance;
    float nonSuspendedMassRear = combinedNonSuspendedMass.value - nonSuspendedMassFront;

    float nonSuspendedWTFront =
        nonSuspendedMassFront * ay * combinedNonSuspendedMass.position.z.v / frontTrackWidth;
    float nonSuspendedWTRear =
        nonSuspendedMassRear * ay * combinedNonSuspendedMass.position.z.v / rearTrackWidth;

    float suspendedMassFront = combinedSuspendedMass.value *
                               (trackDistance - combinedSuspendedMass.position.x.v) / trackDistance;
    float suspendedMassRear = combinedSuspendedMass.value - suspendedMassFront;

    float geometricWTFront = suspendedMassFront * ay * rollCenterHeightFront / frontTrackWidth;
    float geometricWTRear = suspendedMassRear * ay * rollCenterHeightBack / rearTrackWidth;

    float antiRollStiffnessTotal = antiRollStiffnessFront + antiRollStiffnessRear;

    float elasticWTFront = suspendedMassFront * ay *
                           (combinedSuspendedMass.position.z.v - rollCenterHeightFront) *
                           (antiRollStiffnessFront / antiRollStiffnessTotal) / frontTrackWidth;
    float elasticWTRear = suspendedMassRear * ay *
                          (combinedSuspendedMass.position.z.v - rollCenterHeightBack) *
                          (antiRollStiffnessRear / antiRollStiffnessTotal) / rearTrackWidth;

    float frontTransfer = nonSuspendedWTFront + geometricWTFront + elasticWTFront;
    float rearTransfer = nonSuspendedWTRear + geometricWTRear + elasticWTRear;

    WheelData<float> loads;

    loads.FL = -frontTransfer;
    loads.FR = frontTransfer;
    loads.RL = -rearTransfer;
    loads.RR = rearTransfer;

    return loads;
}
