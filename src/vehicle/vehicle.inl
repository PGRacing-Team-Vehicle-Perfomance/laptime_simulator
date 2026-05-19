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

template <typename Frame>
Vehicle<Frame>::Vehicle(const Config& config,
                        WheelData<Positioned<std::unique_ptr<TireBase<Frame>>, Frame>>&& tires)
    : rollCenterHeightFront(config.get("Vehicle", "rollCenterHeightFront")),
      rollCenterHeightBack(config.get("Vehicle", "rollCenterHeightBack")),
      frontTrackWidth(config.get("Vehicle", "frontTrackWidth")),
      rearTrackWidth(config.get("Vehicle", "rearTrackWidth")),
      trackDistance(config.get("Vehicle", "trackDistance")),
      toeAngle(config.getAlphaWheelData<Frame>("Vehicle", "toeAngle")),
      suspendedMassAtWheels(config.getWheelData<float>("Vehicle", "suspendedMassAtWheels")),
      nonSuspendedMassAtWheels(config.getWheelData<float>("Vehicle", "nonSuspendedMassAtWheels")),
      tires(std::move(tires)) {
    brakeBiasFront = config.get("Vehicle", "brakeBiasFront", 0.6f);
    driveBiasFront = config.get("Vehicle", "driveBiasFront", 0.0f);
    longEquilibriumEnabled = config.get("Simlation", "longEquilibrium", 1.0f) > 0.5f;
    targetLongAcc = config.get("Simlation", "targetLongAcc", 0.0f);

    aero.value = {config};
    aero.position = config.getVec<Frame>("Vehicle", "claPosition");

    combinedNonSuspendedMass = {0, {0, 0, 0}};
    combinedSuspendedMass = {0, {0, 0, 0}};

    for (size_t i = 0; i < CarConstants::WHEEL_COUNT; i++) {
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

    float frontSpringWheelRate =
        config.get("Vehicle", "frontKspring") / std::pow(config.get("Vehicle", "frontSpringMotionRatio"), 2);
    float frontTorqueSpring =
        std::pow(frontTrackWidth, 2) * std::tan(M_PI / 180) * frontSpringWheelRate / 2;
    float frontArbTorque = config.get("Vehicle", "frontKarb") * std::pow(frontTrackWidth, 2) *
                           std::tan(M_PI / 180) / std::pow(config.get("Vehicle", "frontArbMotionRatio"), 2);
    antiRollStiffnessFront = frontArbTorque + frontTorqueSpring;

    float rearSpringWheelRate =
        config.get("Vehicle", "rearKspring") / std::pow(config.get("Vehicle", "rearSpringMotionRatio"), 2);
    float rearTorqueSpring =
        std::pow(rearTrackWidth, 2) * std::tan(M_PI / 180) * rearSpringWheelRate / 2;
    float rearArbTorque = config.get("Vehicle", "rearKarb") * std::pow(rearTrackWidth, 2) *
                          std::tan(M_PI / 180) / std::pow(config.get("Vehicle", "rearArbMotionRatio"), 2);
    antiRollStiffnessRear = rearArbTorque + rearTorqueSpring;
}

template <typename Frame>
WheelData<float> Vehicle<Frame>::distributeKappa(float demand) {
    // Kept for interface but no longer used in solver
    WheelData<float> kappa;
    kappa.FL = kappa.FR = kappa.RL = kappa.RR = 0;
    return kappa;
}

template <typename Frame>
std::array<float, 2> Vehicle<Frame>::calculateLatAccAndYawMoment(
    float tolerance, int maxIterations, const Config& config) {
    WheelData<X<Frame>> tireForcesX;
    WheelData<Y<Frame>> tireForcesY;
    WheelData<Z<Frame>> tireMomentsZ;
    WheelData<Alpha<Frame>> slipAngles;
    float speed = state.velocity.getLength();
    float earthAcc = config.get("Environment", "earthAcc");

    // Find kappa for a single wheel such that its Fx matches targetFx
    auto solveKappaForWheel = [&](size_t i, float load, Alpha<Frame> slip, float targetFx) -> float {
        if (load < 1.0f) return 0;
        float kLo = -0.3f, kHi = 0.3f;
        auto fxAt = [&](float k) -> float {
            tires[i].value->calculate(load, slip, k);
            return tires[i].value->getForce().value.x.v;
        };
        float fLo = fxAt(kLo) - targetFx;
        float fHi = fxAt(kHi) - targetFx;
        if ((fLo > 0) == (fHi > 0)) return 0;
        for (int j = 0; j < 30; j++) {
            float kMid = (kLo + kHi) * 0.5f;
            float fMid = fxAt(kMid) - targetFx;
            if (std::abs(fMid) < 0.5f) break;
            if ((fLo > 0) != (fMid > 0)) { kHi = kMid; fHi = fMid; }
            else { kLo = kMid; fLo = fMid; }
        }
        return (kLo + kHi) * 0.5f;
    };

    // Evaluate: given latAcc and force demand [N], compute tire forces with open diff
    // forceDemand > 0 = drive, < 0 = brake
    // Open diff: equal Fx target per wheel within each axle
    auto evaluate = [&](float testLatAcc, float forceDemand) -> float {
        state.angularVelocity.z = Z<Frame>{testLatAcc / speed};
        slipAngles = calculateSlipAngles();
        auto testLoads = totalTireLoads(Y<Frame>{testLatAcc}, config);

        WheelData<float> targetFx;
        if (forceDemand >= 0) {
            float fAxle = forceDemand * driveBiasFront / 2;
            float rAxle = forceDemand * (1 - driveBiasFront) / 2;
            targetFx.FL = fAxle;  targetFx.FR = fAxle;
            targetFx.RL = rAxle;  targetFx.RR = rAxle;
        } else {
            float fAxle = forceDemand * brakeBiasFront / 2;
            float rAxle = forceDemand * (1 - brakeBiasFront) / 2;
            targetFx.FL = fAxle;  targetFx.FR = fAxle;
            targetFx.RL = rAxle;  targetFx.RR = rAxle;
        }

        for (size_t i = 0; i < CarConstants::WHEEL_COUNT; i++) {
            if (std::abs(targetFx[i]) < 0.5f) {
                tires[i].value->calculate(testLoads[i], slipAngles[i], 0);
            } else {
                float kappa = solveKappaForWheel(i, testLoads[i], slipAngles[i], targetFx[i]);
                tires[i].value->calculate(testLoads[i], slipAngles[i], kappa);
            }
            tireForcesX[i] = tires[i].value->getForce().value.x;
            tireForcesY[i] = tires[i].value->getForce().value.y;
            tireMomentsZ[i] = tires[i].value->getTorque().z;
        }
        return calculateLatAcc(tireForcesX, tireForcesY).v;
    };

    // Inner bisection: find latAcc equilibrium for a given force demand
    float maxAcc = combinedTotalMass.value * earthAcc;
    auto solveLatAcc = [&](float forceDemand) -> float {
        float lo = lastLatAcc - 5.0f;
        float hi = lastLatAcc + 5.0f;
        float flo = evaluate(lo, forceDemand) - lo;
        float fhi = evaluate(hi, forceDemand) - hi;

        while ((flo > 0) == (fhi > 0)) {
            lo = std::max(lo - 10.0f, -maxAcc);
            hi = std::min(hi + 10.0f, maxAcc);
            flo = evaluate(lo, forceDemand) - lo;
            fhi = evaluate(hi, forceDemand) - hi;
            if (lo <= -maxAcc && hi >= maxAcc) break;
        }

        for (int i = 0; i < maxIterations; i++) {
            float mid = (lo + hi) * 0.5f;
            float fmid = evaluate(mid, forceDemand) - mid;
            if (std::abs(hi - lo) < tolerance) break;
            if ((flo > 0) != (fmid > 0)) {
                hi = mid; fhi = fmid;
            } else {
                lo = mid; flo = fmid;
            }
        }
        return (lo + hi) * 0.5f;
    };

    // Outer bisection: find force demand [N] s.t. long acceleration == targetLongAcc.
    // Disabled -> fd=0 (no Fx injected, pre-PR3 behavior).
    float fd = 0;
    if (longEquilibriumEnabled) {
        float maxForce = combinedTotalMass.value * earthAcc * 2;
        float demandLo = -maxForce, demandHi = maxForce;

        auto longAccError = [&](float demand) -> float {
            float la = solveLatAcc(demand);
            evaluate(la, demand);
            return calculateLongAcc(tireForcesX, tireForcesY).v - targetLongAcc;
        };

        float flk = longAccError(demandLo);
        float fhk = longAccError(demandHi);

        if ((flk > 0) != (fhk > 0)) {
            for (int i = 0; i < maxIterations; i++) {
                float midD = (demandLo + demandHi) * 0.5f;
                float fmk = longAccError(midD);
                if (std::abs(fmk) < tolerance * 0.1f) break;
                if ((flk > 0) != (fmk > 0)) {
                    demandHi = midD; fhk = fmk;
                } else {
                    demandLo = midD; flk = fmk;
                }
            }
        }

        fd = (demandLo + demandHi) * 0.5f;
    }
    Y<Frame> latAcc{solveLatAcc(fd)};
    evaluate(latAcc.v, fd);
    lastLatAcc = latAcc.v;

    float yawMomentFromTires = 0;
    for (size_t i = 0; i < CarConstants::WHEEL_COUNT; i++) {
        yawMomentFromTires += tireMomentsZ[i].v;
    }

    auto vehicleFx = getVehicleFxFromTireForces(tireForcesX, tireForcesY);
    auto vehicleFy = getVehicleFyFromTireForces(tireForcesX, tireForcesY);

    float yawMomentFromFy =
        ((vehicleFy.FL.v + vehicleFy.FR.v) * combinedTotalMass.position.x.v) -
        ((vehicleFy.RL.v + vehicleFy.RR.v) * (trackDistance - combinedTotalMass.position.x.v));

    float yawMomentFromFx = (frontTrackWidth / 2) * (vehicleFx.FL.v - vehicleFx.FR.v) + // if center of mass is not in the middle of track, this is wrong
                            (rearTrackWidth / 2) * (vehicleFx.RL.v - vehicleFx.RR.v);

    // TODO: aero yaw moment
    float yawMoment = yawMomentFromFy + yawMomentFromFx + yawMomentFromTires;
    return {latAcc.v, yawMoment};
}

template <typename Frame>
void Vehicle<Frame>::setSteeringAngle(Alpha<Frame> steeringAngle) {
    state.steeringAngle = steeringAngle;
    state.wheelAngles.FL = Alpha<Frame>(state.steeringAngle);
    state.wheelAngles.FR = Alpha<Frame>(state.steeringAngle);
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
    auto velocityFy =  getVelocityFyFromTireForces(tireForcesX, tireForcesY);
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
        velocityAlignedFy[i] = Y<Frame>{vehicleFy[i].v * std::cos(chassisSlipAngle) 
                                        - vehicleFx[i].v * std::sin(chassisSlipAngle)};
    }

    return velocityAlignedFy;
}

template <typename Frame>
X<Frame> Vehicle<Frame>::calculateLongAcc(const WheelData<X<Frame>>& tireForcesX,
                                          const WheelData<Y<Frame>>& tireForcesY) {
    WheelData<X<Frame>> vehicleFx = getVehicleFxFromTireForces(tireForcesX, tireForcesY);
    WheelData<Y<Frame>> vehicleFy = getVehicleFyFromTireForces(tireForcesX, tireForcesY);

    float chassisSlipAngle = std::atan2(state.velocity.y.v, state.velocity.x.v);

    float longForce = 0;
    for (size_t i = 0; i < CarConstants::WHEEL_COUNT; i++) {
        longForce += vehicleFx[i].v * std::cos(chassisSlipAngle)
                   + vehicleFy[i].v * std::sin(chassisSlipAngle);
    }
    return X<Frame>{longForce / combinedTotalMass.value};
}

template <typename Frame>
WheelData<float> Vehicle<Frame>::totalTireLoads(Y<Frame> latAcc,
                                                const Config& config) {
    float earthAcc = config.get("Environment", "earthAcc");
    auto static_load = staticLoad(earthAcc);
    auto aero = aeroLoad(config);
    auto transfer = loadTransfer(latAcc);
    WheelData<float> tireLoads;
    for (size_t i = 0; i < CarConstants::WHEEL_COUNT; i++) {
        tireLoads[i] = static_load[i] + aero[i] + transfer[i];
    }

    auto solveAxleLCP = [](float& left, float& right, float axleTotal) {
        if (axleTotal <= 0) {
            left = 0;
            right = 0;
        } else if (left < 0) {
            left = 0;
            right = axleTotal;
        } else if (right < 0) {
            right = 0;
            left = axleTotal;
        }
    };

    float frontTotal = static_load.FL + static_load.FR + aero.FL + aero.FR;
    float rearTotal = static_load.RL + static_load.RR + aero.RL + aero.RR;
    solveAxleLCP(tireLoads.FL, tireLoads.FR, frontTotal);
    solveAxleLCP(tireLoads.RL, tireLoads.RR, rearTotal);

    return tireLoads;
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
    Vec<Frame> wind = config.getVec<Frame>("Environment", "wind");

    aero.value.calculate(state, airDensityVal, wind);
    // TODO: cla sign bug — force.z > 0 = lift, negate to get downforce
    return distributeForces(-aero.value.getForce().value.z.v, aero.position.x.v, aero.position.y.v);
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
