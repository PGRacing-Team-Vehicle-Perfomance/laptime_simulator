#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <numbers>

#define _USE_MATH_DEFINES
#include <math.h>

#include "config/config.h"
#include "coordTypes.h"
#include "vehicle/aero/aero.h"
#include "vehicle/tire/tire.h"
#include "vehicle/vehicleHelper.h"

template <typename ForceFn>
inline float ascendingBranchSlipRatio(ForceFn forceAt, float target) {
    if (std::abs(target) < 0.5f) return 0;
    float direction = target > 0 ? 1.0f : -1.0f;

    float peakSlipRatio = 0;
    float peakForce = forceAt(0.0f);
    for (int scan = 1; scan <= 20; scan++) {
        float slipRatio = direction * 0.015f * scan;
        float force = forceAt(slipRatio);
        if (direction * force > direction * peakForce) {
            peakForce = force;
            peakSlipRatio = slipRatio;
        }
    }
    float low = peakSlipRatio - direction * 0.015f;
    float high = peakSlipRatio + direction * 0.015f;
    const float invphi = 0.618033988f;
    float c = high - (high - low) * invphi;
    float d = low + (high - low) * invphi;
    float fc = forceAt(c);
    float fd = forceAt(d);
    for (int iter = 0; iter < 20; iter++) {
        if (direction * fc > direction * fd) {
            high = d;
            d = c;
            fd = fc;
            c = high - (high - low) * invphi;
            fc = forceAt(c);
        } else {
            low = c;
            c = d;
            fc = fd;
            d = low + (high - low) * invphi;
            fd = forceAt(d);
        }
    }
    peakSlipRatio = (low + high) * 0.5f;
    peakForce = forceAt(peakSlipRatio);
    if (direction * target >= direction * peakForce) return peakSlipRatio;

    float lo = 0.0f;
    float hi = peakSlipRatio;
    for (int iter = 0; iter < 30; iter++) {
        float mid = (lo + hi) * 0.5f;
        if (direction * forceAt(mid) < direction * target) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    return (lo + hi) * 0.5f;
}

template <typename Frame>
Vehicle<Frame>::Vehicle(const Config& config,
                        WheelData<Positioned<std::unique_ptr<TireBase<Frame>>, Frame>>&& tires,
                        Positioned<std::unique_ptr<AeroBase<Frame>>, Frame>&& aero,
                        std::unique_ptr<SteeringTableBase<Frame>>&& steeringTable)
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
      tires(std::move(tires)) {
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
    frontDiffLocking = config.get("Vehicle", "frontDiffLocking", 1.0f);
    rearDiffLocking = config.get("Vehicle", "rearDiffLocking", 1.0f);
    dragCoefficientArea = config.get("Aero", "cda", 0.0f);
    airDensityValue = config.get("Environment", "airDensity");
    longEquilibriumEnabled = config.get("Simlation", "longEquilibrium", 0.0f) > 0.5f;
    targetLongAcc = config.get("Simlation", "targetLongAcc", 0.0f);
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
    if (longEquilibriumEnabled && std::abs(longForceDemand) > 0.5f) {
        bool driving = longForceDemand > 0;
        float frontBias = driving ? driveBiasFront : brakeBiasFront;
        float frontAxle = longForceDemand * frontBias;
        float rearAxle = longForceDemand * (1.0f - frontBias);
        bool frontDriven = driveBiasFront > 1e-6f;
        bool rearDriven = (1.0f - driveBiasFront) > 1e-6f;
        solveAxle(0, 1, loads, slipAngles, frontAxle, frontDriven, frontDiffLocking, slipRatio.FL,
                  slipRatio.FR);
        solveAxle(2, 3, loads, slipAngles, rearAxle, rearDriven, rearDiffLocking, slipRatio.RL,
                  slipRatio.RR);
    }

    for (size_t i = 0; i < CarConstants::WHEEL_COUNT; i++) {
        lastKappa[i] = slipRatio[i];
        tires[i].value->calculate(loads[i], slipAngles[i], slipRatio[i], camber[i]);
        step.tireForcesX[i] = tires[i].value->getForce().value.x;
        step.tireForcesY[i] = tires[i].value->getForce().value.y;
        step.tireMomentsZ[i] = tires[i].value->getTorque().z;
    }
}

template <typename Frame>
typename Vehicle<Frame>::SolverStep Vehicle<Frame>::solveCoupled(const Config& config,
                                                                 float tolerance,
                                                                 int maxIterations) {
    float earthAcc = config.get("Environment", "earthAcc");
    float maxForce = combinedTotalMass.value * earthAcc * 2.0f;
    float velocity = state.velocity.getLength();

    float demandLo = -maxForce;
    float demandHi = maxForce;

    float latAcc = lastLatAcc;
    float demand = std::max(demandLo, std::min(demandHi, lastDemand));
    longitudinalAccEstimate = lastLongAcc;
    float relaxation = 1.0f;
    float previousLatResidual = 0;

    SolverStep step;
    for (int iter = 0; iter < maxIterations; iter++) {
        state.angularVelocity.z = Z<Frame>{latAcc / velocity};
        auto slipAngles = calculateSlipAngles();
        auto loads = totalTireLoads(Y<Frame>{latAcc}, config);
        longForceDemand = demand;
        computeTireForces(loads, slipAngles, step);

        float newLatAcc = calculateLatAcc(step.tireForcesX, step.tireForcesY).v;
        float actualLongAcc = calculateLongAcc(step.tireForcesX, step.tireForcesY).v;
        float bodyLongAcc = calculateBodyLongAcc(step.tireForcesX, step.tireForcesY).v;
        float latResidual = newLatAcc - latAcc;
        float longResidual = targetLongAcc - actualLongAcc;
        longitudinalAccEstimate += 0.5f * (bodyLongAcc - longitudinalAccEstimate);

        if (std::abs(latResidual) < tolerance && std::abs(longResidual) < tolerance) {
            latAcc = newLatAcc;
            break;
        }

        if (iter > 0 && latResidual * previousLatResidual < 0) {
            relaxation = std::max(0.2f, relaxation * 0.5f);
        }
        previousLatResidual = latResidual;

        latAcc += relaxation * latResidual;
        demand =
            std::max(demandLo, std::min(demandHi, demand + combinedTotalMass.value * longResidual));
    }

    lastLatAcc = latAcc;
    lastDemand = demand;
    lastLongAcc = longitudinalAccEstimate;
    longForceDemand = demand;
    step.latAcc = Y<Frame>{latAcc};
    return step;
}

template <typename Frame>
float Vehicle<Frame>::slipRatioForForce(size_t wheel, float load, Alpha<Frame> slipAngle,
                                        Gamma<Frame> camber, float targetFx) {
    if (load < 1.0f || std::abs(targetFx) < 0.5f) return 0;

    auto forceAt = [&](float slipRatio) {
        tires[wheel].value->calculate(load, slipAngle, slipRatio, camber);
        return tires[wheel].value->getForce().value.x.v;
    };
    return ascendingBranchSlipRatio(forceAt, targetFx);
}

template <typename Frame>
float Vehicle<Frame>::wheelLongSpeed(size_t wheel) {
    float x = tires[wheel].position.x.v;
    float y = tires[wheel].position.y.v;
    float yawRate = state.angularVelocity.z.v;
    float contactX = state.velocity.x.v - yawRate * y;
    float contactY = state.velocity.y.v + yawRate * x;
    float heading = state.wheelAngles[wheel].v;
    return contactX * std::cos(heading) + contactY * std::sin(heading);
}

template <typename Frame>
void Vehicle<Frame>::solveAxle(size_t leftWheel, size_t rightWheel, const WheelData<float>& loads,
                               const WheelData<Alpha<Frame>>& slipAngles, float axleDemand,
                               bool hasDiff, float locking, float& leftSlipRatio,
                               float& rightSlipRatio) {
    float openForce = axleDemand * 0.5f;
    float leftOpen = slipRatioForForce(leftWheel, loads[leftWheel], slipAngles[leftWheel],
                                       camber[leftWheel], openForce);
    float rightOpen = slipRatioForForce(rightWheel, loads[rightWheel], slipAngles[rightWheel],
                                        camber[rightWheel], openForce);

    float leftSpeed = wheelLongSpeed(leftWheel);
    float rightSpeed = wheelLongSpeed(rightWheel);
    if (!hasDiff || leftSpeed < 1.0f || rightSpeed < 1.0f) {
        leftSlipRatio = leftOpen;
        rightSlipRatio = rightOpen;
        return;
    }

    float refSpeed = 0.5f * (leftSpeed + rightSpeed);
    auto clampRatio = [](float ratio) { return std::max(-0.9f, std::min(0.9f, ratio)); };
    auto axleForceAt = [&](float meanSlip) {
        float surfaceSpeed = refSpeed * (1.0f + meanSlip);
        float leftRatio = clampRatio(surfaceSpeed / leftSpeed - 1.0f);
        float rightRatio = clampRatio(surfaceSpeed / rightSpeed - 1.0f);
        tires[leftWheel].value->calculate(loads[leftWheel], slipAngles[leftWheel], leftRatio,
                                          camber[leftWheel]);
        float leftForce = tires[leftWheel].value->getForce().value.x.v;
        tires[rightWheel].value->calculate(loads[rightWheel], slipAngles[rightWheel], rightRatio,
                                           camber[rightWheel]);
        float rightForce = tires[rightWheel].value->getForce().value.x.v;
        return leftForce + rightForce;
    };
    float meanSlip = ascendingBranchSlipRatio(axleForceAt, axleDemand);
    float surfaceSpeed = refSpeed * (1.0f + meanSlip);
    float lockedLeft = clampRatio(surfaceSpeed / leftSpeed - 1.0f);
    float lockedRight = clampRatio(surfaceSpeed / rightSpeed - 1.0f);

    leftSlipRatio = (1.0f - locking) * leftOpen + locking * lockedLeft;
    rightSlipRatio = (1.0f - locking) * rightOpen + locking * lockedRight;
}

template <typename Frame>
X<Frame> Vehicle<Frame>::calculateLongAcc(const WheelData<X<Frame>>& tireForcesX,
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
    Y<Frame> latAcc{0};
    Y<Frame> prevLatAcc{0};
    float residual = 0;
    float prevResidual = 0;
    int iterations = 0;
    bool oscillating = false;

    longitudinalAccEstimate = lastLongAcc;
    state.angularVelocity.setLength(0);

    SolverStep step;
    do {
        iterations++;
        step = evaluateAt(latAcc, config);
        prevResidual = residual;
        residual = step.latAcc.v - latAcc.v;

        if (iterations >= 2 && (residual > 0) != (prevResidual > 0)) {
            oscillating = true;
            break;
        }

        prevLatAcc = latAcc;
        latAcc = step.latAcc;
    } while (std::abs(residual) > tolerance && iterations < maxIterations);

    if (oscillating) {
        Y<Frame> latAccLo{std::min(prevLatAcc.v, latAcc.v)};
        Y<Frame> latAccHi{std::max(prevLatAcc.v, latAcc.v)};
        float residualLo = (prevLatAcc.v < latAcc.v) ? prevResidual : residual;
        Y<Frame> latAccMid;

        for (int i = 0; i < maxIterations && latAccHi.v - latAccLo.v > tolerance; i++) {
            latAccMid.v = (latAccLo.v + latAccHi.v) * 0.5f;
            step = evaluateAt(latAccMid, config);
            float residualMid = step.latAcc.v - latAccMid.v;
            if ((residualMid > 0) == (residualLo > 0)) {
                latAccLo = latAccMid;
                residualLo = residualMid;
            } else {
                latAccHi = latAccMid;
            }
        }

        latAcc = latAccMid;
    }

    lastLongAcc = longitudinalAccEstimate;
    step.latAcc = latAcc;
    return step;
}

template <typename Frame>
std::array<float, 2> Vehicle<Frame>::calculateLatAccAndYawMoment(float tolerance, int maxIterations,
                                                                 const Config& config) {
    SolverStep step;
    if (longEquilibriumEnabled) {
        step = solveCoupled(config, tolerance, maxIterations);
    } else {
        longForceDemand = 0;
        step = solveLatAcc(config, tolerance, maxIterations);
    }
    Y<Frame> latAcc = step.latAcc;

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
    return {latAcc.v, yawMoment};
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
    float speed = state.velocity.getLength();
    state.velocity.x = X<Frame>{speed * std::cos(chassisSlipAngle.v)};
    state.velocity.y = Y<Frame>{speed * std::sin(chassisSlipAngle.v)};
    state.velocity.z = Z<Frame>(0);
}

template <typename Frame>
void Vehicle<Frame>::setSpeed(float speed) {
    state.velocity.setLength(speed);
}

template <typename Frame>
void Vehicle<Frame>::resetContinuity() {
    lastLatAcc = 0;
    lastDemand = combinedTotalMass.value * targetLongAcc;
    lastLongAcc = 0;
    lastKappa = WheelData<float>{};
}

template <typename Frame>
typename Vehicle<Frame>::Continuity Vehicle<Frame>::continuity() const {
    return {lastLatAcc, lastDemand, lastLongAcc, lastKappa};
}

template <typename Frame>
void Vehicle<Frame>::continuity(const Continuity& snapshot) {
    lastLatAcc = snapshot.latAcc;
    lastDemand = snapshot.demand;
    lastLongAcc = snapshot.longAcc;
    lastKappa = snapshot.kappa;
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
