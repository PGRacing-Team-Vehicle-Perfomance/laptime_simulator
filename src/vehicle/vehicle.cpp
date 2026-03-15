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
#include "vehicle/tire/tireBridge.h"
#include "vehicle/tire/tirePacejka.h"
#include "vehicle/vehicle.h"
#include "vehicle/vehicleHelper.h"

Vehicle::Vehicle(const VehicleConfig& vehicleConfig, const TireConfig& tireConfig)
    : rollCenterHeightFront(vehicleConfig.rollCenterHeightFront),
      rollCenterHeightBack(vehicleConfig.rollCenterHeightBack),
      frontTrackWidth(vehicleConfig.frontTrackWidth),
      rearTrackWidth(vehicleConfig.rearTrackWidth),
      trackDistance(vehicleConfig.trackDistance),
      toeAngle(vehicleConfig.toeAngle),
      suspendedMassAtWheels(vehicleConfig.suspendedMassAtWheels),
      nonSuspendedMassAtWheels(vehicleConfig.nonSuspendedMassAtWheels) {
    aero.value = {vehicleConfig};
    aero.position = vehicleConfig.claPosition;

    tires.FL.value = std::make_unique<TirePacejka>(tireConfig, false, Left);
    tires.FR.value = std::make_unique<TirePacejka>(tireConfig, false, Right);
    tires.RL.value = std::make_unique<TirePacejka>(tireConfig, false, Left);
    tires.RR.value = std::make_unique<TirePacejka>(tireConfig, false, Right);

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

std::vector<std::array<float, 4>> Vehicle::getYawMomentDiagramPoints(
    const EnvironmentConfig& environmentConfig, float maxSteeringAngle, float steeringAngleStep,
    float maxSlipAngle, float slipAngleStep, float tolerance, int maxIterations) {
    std::vector<std::array<float, 4>> out;
    for (float steeringAngle = -maxSteeringAngle; steeringAngle <= maxSteeringAngle;
         steeringAngle += steeringAngleStep) {
        state.steeringAngle = Alpha<>(steeringAngle * M_PI / 180.f);
        for (float chassisSlipAngle = -maxSlipAngle; chassisSlipAngle <= maxSlipAngle;
             chassisSlipAngle += slipAngleStep) {
            state.rotation.z = Alpha<>(chassisSlipAngle * M_PI / 180.f);
        
            float speed = state.velocity.getLength();
            state.velocity.x = X<>{speed * std::cos(state.rotation.z.v)};
            state.velocity.y = Y<>{speed * std::sin(state.rotation.z.v)};
            state.velocity.z = Z<>{0};

            state.angularVelocity = {};

            std::array<float, 2> diagramPoint =
                getLatAccAndYawMoment(tolerance, maxIterations, environmentConfig);
            out.push_back(
                {state.steeringAngle.v, state.rotation.z.v, diagramPoint[0], diagramPoint[1]});
        }
    }
    return out;
}

std::array<float, 2> Vehicle::getLatAccAndYawMoment(float tolerance, int maxIterations,
                                                    const EnvironmentConfig& environmentConfig) {
    WheelData<X<>> tireForcesX;
    WheelData<Y<>> tireForcesY;
    WheelData<Z<>> tireMomentsZ;
    WheelData<Alpha<>> slipAngles;
    Y<> latAcc{0};
    float error;
    int iterations = 0;

    auto loads = staticLoad(environmentConfig.earthAcc);
    do {
        iterations++;
        slipAngles = calculateSlipAngles();

        for (size_t i = 0; i < CarConstants::WHEEL_COUNT; i++) {
            auto out = callTire(isoSae, *tires[i].value, loads[i], slipAngles[i]);

            tireForcesX[i] = out.Fx;
            tireForcesY[i] = out.Fy;
            tireMomentsZ[i] = out.Mz;
        }

        auto newLatAcc = calculateLatAcc(tireForcesX, tireForcesY);
        error = std::abs(latAcc.v - newLatAcc.v);
        latAcc = newLatAcc;
        state.angularVelocity.z = Z<>{latAcc.v / state.velocity.getLength()};

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

VehicleState* Vehicle::getState() { return &state; }

WheelData<Alpha<>> Vehicle::calculateSteeringAngles() {
    return {.FL = Alpha<>(state.steeringAngle),
            .FR = Alpha<>(state.steeringAngle),
            .RL = Alpha<>(0),
            .RR = Alpha<>(0)};
}

WheelData<Alpha<>> Vehicle::calculateSlipAngles() {
    float massToFront = combinedTotalMass.position.x.v;
    float massToRear = trackDistance - massToFront;

    auto steeringAngles = calculateSteeringAngles();

    WheelData<Alpha<>> slipAngle;

    slipAngle.FL =
        Alpha<>{static_cast<float>(std::atan((state.velocity.y.v + state.angularVelocity.z.v * massToFront) /
                                             (state.velocity.x.v - state.angularVelocity.z.v * frontTrackWidth / 2.0)) -
                                   steeringAngles.FL.v - toeAngle.FL.v)};

    slipAngle.FR =
        Alpha<>{static_cast<float>(std::atan((state.velocity.y.v + state.angularVelocity.z.v * massToFront) /
                                             (state.velocity.x.v + state.angularVelocity.z.v * frontTrackWidth / 2.0)) -
                                   steeringAngles.FR.v - toeAngle.FR.v)};

    slipAngle.RL =
        Alpha<>{static_cast<float>(std::atan((state.velocity.y.v - state.angularVelocity.z.v * massToRear) /
                                             (state.velocity.x.v - state.angularVelocity.z.v * rearTrackWidth / 2.0)) -
                                   steeringAngles.RL.v - toeAngle.RL.v)};

    slipAngle.RR =
        Alpha<>{static_cast<float>(std::atan((state.velocity.y.v - state.angularVelocity.z.v * massToRear) /
                                             (state.velocity.x.v + state.angularVelocity.z.v * rearTrackWidth / 2.0)) -
                                   steeringAngles.RR.v - toeAngle.RR.v)};

    return slipAngle;
}

Y<> Vehicle::calculateLatAcc(const WheelData<X<>>& tireForcesX, const WheelData<Y<>>& tireForcesY) {
    auto vehicleFy = getVehicleFyFromTireForces(tireForcesX, tireForcesY);
    float latForce = 0;
    for (size_t i = 0; i < CarConstants::WHEEL_COUNT; i++) {
        latForce += vehicleFy[i].v;
    }
    return Y<>{latForce / combinedTotalMass.value};
}

WheelData<Y<>> Vehicle::getVehicleFyFromTireForces(const WheelData<X<>>& tireFx,
                                                   const WheelData<Y<>>& tireFy) {
    auto steeringAngles = calculateSteeringAngles();

    WheelData<Y<>> vehicleFy;

    float deltaFL = steeringAngles.FL.v;
    float deltaFR = steeringAngles.FR.v;

    vehicleFy.FL = Y<>{tireFx.FL.v * std::sin(deltaFL) + tireFy.FL.v * std::cos(deltaFL)};
    vehicleFy.FR = Y<>{tireFx.FR.v * std::sin(deltaFR) + tireFy.FR.v * std::cos(deltaFR)};
    vehicleFy.RL = tireFy.RL;
    vehicleFy.RR = tireFy.RR;

    return vehicleFy;
}

WheelData<X<>> Vehicle::getVehicleFxFromTireForces(const WheelData<X<>>& tireFx,
                                                   const WheelData<Y<>>& tireFy) {
    auto steeringAngles = calculateSteeringAngles();

    WheelData<X<>> vehicleFx;

    float deltaFL = steeringAngles.FL.v;
    float deltaFR = steeringAngles.FR.v;

    vehicleFx.FL = X<>{tireFx.FL.v * std::cos(deltaFL) - tireFy.FL.v * std::sin(deltaFL)};
    vehicleFx.FR = X<>{tireFx.FR.v * std::cos(deltaFR) - tireFy.FR.v * std::sin(deltaFR)};
    vehicleFx.RL = tireFx.RL;
    vehicleFx.RR = tireFx.RR;

    return vehicleFx;
}

WheelData<float> Vehicle::totalTireLoads(Y<> latAcc, const EnvironmentConfig& environmentConfig) {
    auto static_load = staticLoad(environmentConfig.earthAcc);
    auto aero = aeroLoad(environmentConfig);
    auto transfer = loadTransfer(latAcc);
    WheelData<float> tireLoads;
    for (size_t i = 0; i < CarConstants::WHEEL_COUNT; i++) {
        tireLoads[i] = std::max(0.f, static_load[i] + aero[i] + transfer[i]);
    }
    return tireLoads;
}

WheelData<float> Vehicle::staticLoad(float earthAcc) {
    WheelData<float> loads;
    for (int i = 0; i < CarConstants::WHEEL_COUNT; i++) {
        loads[i] = (nonSuspendedMassAtWheels[i] + suspendedMassAtWheels[i]) * earthAcc;
    }
    return loads;
}

WheelData<float> Vehicle::distributeForces(float totalForce, float frontDist, float leftDist) {
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

WheelData<float> Vehicle::aeroLoad(const EnvironmentConfig& environmentConfig) {
    aero.value.calculate(state, environmentConfig.airDensity, environmentConfig.wind);
    // TODO: cla sign bug — force.z > 0 = lift, negate to get downforce
    return distributeForces(-aero.value.getForce().value.z.v, aero.position.x.v, aero.position.y.v);
}

WheelData<float> Vehicle::loadTransfer(Y<> latAcc) {
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
