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
#include "vehicle/tire/tirePacejka.h"
#include "vehicle/vehicleHelper.h"

template <typename Frame>
Vehicle<Frame>::Vehicle(const VehicleConfig<Frame>& vehicleConfig,
                         WheelData<Positioned<std::unique_ptr<TireBase<Frame>>, Frame>>&& tires)
    : rollCenterHeightFront(vehicleConfig.rollCenterHeightFront),
      rollCenterHeightBack(vehicleConfig.rollCenterHeightBack),
      frontTrackWidth(vehicleConfig.frontTrackWidth),
      rearTrackWidth(vehicleConfig.rearTrackWidth),
      trackDistance(vehicleConfig.trackDistance),
      toeAngle(vehicleConfig.toeAngle),
      suspendedMassAtWheels(vehicleConfig.suspendedMassAtWheels),
      nonSuspendedMassAtWheels(vehicleConfig.nonSuspendedMassAtWheels),
      tires(std::move(tires)) {
    aero.value = {vehicleConfig};
    aero.position = vehicleConfig.claPosition;

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
    combinedNonSuspendedMass.position.z.v = 0;

    combinedSuspendedMass.position.x.v /= combinedSuspendedMass.value;
    combinedSuspendedMass.position.y.v /= combinedSuspendedMass.value;
    combinedSuspendedMass.position.z.v = vehicleConfig.suspendedMassHeight;

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
        vehicleConfig.frontKspring / std::pow(vehicleConfig.frontSpringMotionRatio, 2);
    float frontTorqueSpring =
        std::pow(frontTrackWidth, 2) * std::tan(M_PI / 180) * frontSpringWheelRate / 2;
    float frontArbTorque = vehicleConfig.frontKarb * std::pow(vehicleConfig.frontTrackWidth, 2) *
                           std::tan(M_PI / 180) / std::pow(vehicleConfig.frontArbMotionRatio, 2);
    antiRollStiffnessFront = frontArbTorque + frontTorqueSpring;

    float rearSpringWheelRate =
        vehicleConfig.rearKspring / std::pow(vehicleConfig.rearSpringMotionRatio, 2);
    float rearTorqueSpring =
        std::pow(rearTrackWidth, 2) * std::tan(M_PI / 180) * rearSpringWheelRate / 2;
    float rearArbTorque = vehicleConfig.rearKarb * std::pow(vehicleConfig.rearTrackWidth, 2) *
                          std::tan(M_PI / 180) / std::pow(vehicleConfig.rearArbMotionRatio, 2);
    antiRollStiffnessRear = rearArbTorque + rearTorqueSpring;
}

template <typename Frame>
std::vector<std::array<float, 4>> Vehicle<Frame>::getYawMomentDiagramPoints(
    float speed, const EnvironmentConfig<Frame>& environmentConfig, float maxSteeringAngle,
    float steeringAngleStep, float maxSlipAngle, float slipAngleStep, float tolerance,
    int maxIterations) {
    std::vector<std::array<float, 4>> out;
    for (float steeringAngle = -maxSteeringAngle; steeringAngle <= maxSteeringAngle;
         steeringAngle += steeringAngleStep) {
        setSteering(Alpha<Frame>(steeringAngle * M_PI / 180.f));
        for (float chassisSlipAngle = -maxSlipAngle; chassisSlipAngle <= maxSlipAngle;
             chassisSlipAngle += slipAngleStep) {
            state.rotation.z = Alpha<Frame>(chassisSlipAngle * M_PI / 180.f);

            state.velocity.x = X<Frame>{speed * std::cos(state.rotation.z.v)};
            state.velocity.y = Y<Frame>{speed * std::sin(state.rotation.z.v)};
            state.velocity.z = Z<Frame>{0};

            state.angularVelocity.setLength(0);

            std::array<float, 2> diagramPoint =
                getLatAccAndYawMoment(tolerance, maxIterations, environmentConfig);
            out.push_back(
                {state.steeringAngle.v, state.rotation.z.v, diagramPoint[0], diagramPoint[1]});
        }
    }
    return out;
}

template <typename Frame>
std::array<float, 2> Vehicle<Frame>::getLatAccAndYawMoment(
    float tolerance, int maxIterations, const EnvironmentConfig<Frame>& environmentConfig) {
    WheelData<X<Frame>> tireForcesX;
    WheelData<Y<Frame>> tireForcesY;
    WheelData<Z<Frame>> tireMomentsZ;
    WheelData<Alpha<Frame>> slipAngles;
    Y<Frame> latAcc{0};
    float error;
    int iterations = 0;

    auto loads = staticLoad(environmentConfig.earthAcc);
    do {
        iterations++;
        slipAngles = calculateSlipAngles();

        for (size_t i = 0; i < CarConstants::WHEEL_COUNT; i++) {
            tires[i].value->calculate(loads[i], slipAngles[i], 0);

            tireForcesX[i] = tires[i].value->getForce().value.x;
            tireForcesY[i] = tires[i].value->getForce().value.y;
            tireMomentsZ[i] = tires[i].value->getTorque().z;
        }

        auto newLatAcc = calculateLatAcc(tireForcesX, tireForcesY);
        error = std::abs(latAcc.v - newLatAcc.v);
        latAcc = newLatAcc;
        state.angularVelocity.z = Z<Frame>{latAcc.v / state.velocity.getLength()};

        loads = totalTireLoads(latAcc, environmentConfig);
    } while (error > tolerance && iterations < maxIterations);

    float yawMomentFromTires = 0;
    for (size_t i = 0; i < CarConstants::WHEEL_COUNT; i++) {
        yawMomentFromTires += tireMomentsZ[i].v;
    }

    auto vehicleFx = getVehicleFxFromTireForces(tireForcesX, tireForcesY);
    auto vehicleFy = getVehicleFyFromTireForces(tireForcesX, tireForcesY);

    float yawMomentFromFy =
        ((vehicleFy.FL.v + vehicleFy.FR.v) * combinedTotalMass.position.x.v) -
        ((vehicleFy.RL.v + vehicleFy.RR.v) * (trackDistance - combinedTotalMass.position.x.v));

    float yawMomentFromFx = (frontTrackWidth / 2) * (vehicleFx.FL.v - vehicleFx.FR.v) +
                            (rearTrackWidth / 2) * (vehicleFx.RL.v - vehicleFx.RR.v);

    // TODO: aero yaw moment
    float yawMoment = yawMomentFromFy + yawMomentFromFx + yawMomentFromTires;
    return {latAcc.v, yawMoment};
}

template <typename Frame>
void Vehicle<Frame>::setSteering(Alpha<Frame> steeringAngle) {
    state.steeringAngle = steeringAngle;
    state.wheelAngles.FL = Alpha<Frame>(state.steeringAngle);
    state.wheelAngles.FR = Alpha<Frame>(state.steeringAngle);
    state.wheelAngles.RL = Alpha<Frame>(0);
    state.wheelAngles.RR = Alpha<Frame>(0);
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
    auto vehicleFy = getVehicleFyFromTireForces(tireForcesX, tireForcesY);
    float latForce = 0;
    for (size_t i = 0; i < CarConstants::WHEEL_COUNT; i++) {
        latForce += vehicleFy[i].v;
    }
    return Y<Frame>{latForce / combinedTotalMass.value};
}

template <typename Frame>
WheelData<Y<Frame>> Vehicle<Frame>::getVehicleFyFromTireForces(
    const WheelData<X<Frame>>& tireFx, const WheelData<Y<Frame>>& tireFy) {
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
WheelData<X<Frame>> Vehicle<Frame>::getVehicleFxFromTireForces(
    const WheelData<X<Frame>>& tireFx, const WheelData<Y<Frame>>& tireFy) {
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
WheelData<float> Vehicle<Frame>::totalTireLoads(
    Y<Frame> latAcc, const EnvironmentConfig<Frame>& environmentConfig) {
    auto static_load = staticLoad(environmentConfig.earthAcc);
    auto aero = aeroLoad(environmentConfig);
    auto transfer = loadTransfer(latAcc);
    WheelData<float> tireLoads;
    for (size_t i = 0; i < CarConstants::WHEEL_COUNT; i++) {
        tireLoads[i] = std::max(0.f, static_load[i] + aero[i] + transfer[i]);
    }
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
WheelData<float> Vehicle<Frame>::aeroLoad(const EnvironmentConfig<Frame>& environmentConfig) {
    aero.value.calculate(state, environmentConfig.airDensity, environmentConfig.wind);
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
