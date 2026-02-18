#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <numbers>

#define _USE_MATH_DEFINES
#include <math.h>

#include "config/config.h"
#include "vehicle/aero/aero.h"
#include "vehicle/tire/tire.h"
#include "vehicle/tire/tirePacejka.h"
#include "vehicle/vehicle.h"
#include "vehicle/vehicleHelper.h"

Vehicle::Vehicle(const VehicleConfig& vehicleConfig, const TireConfig& tireConfig)
    : rollCenterHeightFront(vehicleConfig.rollCenterHeightFront),
      rollCenterHeightBack(vehicleConfig.rollCenterHeightBack),
      frontTrackWidth(vehicleConfig.frontTrackWidth),
      rearTrackWidth(vehicleConfig.rearTrackWidth),
      trackDistance(vehicleConfig.trackDistance),
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
    tires.FL.position = {0, -frontTrackWidth / 2, 0};
    tires.FR.position = {0, frontTrackWidth / 2, 0};
    tires.RL.position = {trackDistance, -rearTrackWidth / 2, 0};
    tires.RR.position = {trackDistance, rearTrackWidth / 2, 0};

    for (size_t i = 0; i < CarConstants::WHEEL_COUNT; i++) {
        combinedNonSuspendedMass.position.x += nonSuspendedMassAtWheels[i] * tires[i].position.x;
        combinedNonSuspendedMass.position.y += nonSuspendedMassAtWheels[i] * tires[i].position.y;

        combinedSuspendedMass.position.x += suspendedMassAtWheels[i] * tires[i].position.x;
        combinedSuspendedMass.position.y += suspendedMassAtWheels[i] * tires[i].position.y;
    }
    combinedNonSuspendedMass.position.x /= combinedNonSuspendedMass.value;
    combinedNonSuspendedMass.position.y /= combinedNonSuspendedMass.value;
    combinedNonSuspendedMass.position.z = 0;

    combinedSuspendedMass.position.x /= combinedSuspendedMass.value;
    combinedSuspendedMass.position.y /= combinedSuspendedMass.value;
    combinedSuspendedMass.position.z = vehicleConfig.suspendedMassHeight;

    combinedTotalMass.value = combinedSuspendedMass.value + combinedNonSuspendedMass.value;
    combinedTotalMass.position = {
        (combinedSuspendedMass.position.x * combinedSuspendedMass.value +
         combinedNonSuspendedMass.position.x * combinedNonSuspendedMass.value) /
            combinedTotalMass.value,
        (combinedSuspendedMass.position.y * combinedSuspendedMass.value +
         combinedNonSuspendedMass.position.y * combinedNonSuspendedMass.value) /
            combinedTotalMass.value,
        (combinedSuspendedMass.position.z * combinedSuspendedMass.value +
         combinedNonSuspendedMass.position.z * combinedNonSuspendedMass.value) /
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
        state.steeringAngle = static_cast<float>(steeringAngle);
        for (float chassisSlipAngle = -maxSlipAngle; chassisSlipAngle <= maxSlipAngle;
             chassisSlipAngle += slipAngleStep) {
            state.rotation.z = Angle(static_cast<float>(chassisSlipAngle));
            state.angular_velocity = {0, 0, 0};
            std::array<float, 2> diagramPoint =
                getLatAccAndYawMoment(tolerance, maxIterations, environmentConfig);
            out.push_back(
                {state.steeringAngle, state.rotation.z.get(), diagramPoint[0], diagramPoint[1]});
        }
    }
    return out;
}

std::array<float, 2> Vehicle::getLatAccAndYawMoment(float tolerance, int maxIterations,
                                                    const EnvironmentConfig& environmentConfig) {
    Angle beta = state.rotation.z;
    Vec3f velocity;
    velocity.x = state.velocity.getLength() * std::cos(beta.getRadians());
    velocity.y = state.velocity.getLength() * std::sin(beta.getRadians());
    velocity.z = 0;

    WheelData<float> tireForcesX;
    WheelData<float> tireForcesY;
    WheelData<float> tireMomentsZ{0, 0, 0, 0};
    WheelData<float> slipAngles;
    float latAcc = 0;
    float error;
    int iterations = 0;

    auto loads = staticLoad(environmentConfig.earthAcc);
    do {
        iterations++;
        slipAngles = calculateSlipAngles(state.angular_velocity.z, velocity);

        for (size_t i = 0; i < CarConstants::WHEEL_COUNT; i++) {
            tires[i].value->calculate(loads[i], slipAngles[i], 0);
            tireForcesX[i] = tires[i].value->getForce().value.x;
            tireForcesY[i] = tires[i].value->getForce().value.y;
            tireMomentsZ[i] = tires[i].value->getTorque().z;
        }

        auto newLatAcc = calculateLatAcc(tireForcesX, tireForcesY);
        error = std::abs(latAcc - newLatAcc);
        latAcc = newLatAcc;
        state.angular_velocity.z = latAcc / velocity.x;

        loads = totalTireLoads(latAcc, environmentConfig);
    } while (error > tolerance && iterations < maxIterations);

    float yawMomentFromTires = 0;
    for (size_t i = 0; i < CarConstants::WHEEL_COUNT; i++) {
        yawMomentFromTires = yawMomentFromTires + tireMomentsZ[i];
    }

    // Transform tire forces to vehicle coordinates
    auto vehicleFx = getVehicleFxFromTireForces(tireForcesX, tireForcesY);
    auto vehicleFy = getVehicleFyFromTireForces(tireForcesX, tireForcesY);

    // Yaw moment from Fy (moment arm in X direction)
    float yawMomentFromFy =
        ((vehicleFy.FL + vehicleFy.FR) * combinedTotalMass.position.x) -
        ((vehicleFy.RL + vehicleFy.RR) * (trackDistance - combinedTotalMass.position.x));

    // Yaw moment from Fx (moment arm in Y direction): Mz = -y * Fx
    float yawMomentFromFx = (frontTrackWidth / 2) * (vehicleFx.FL - vehicleFx.FR) +
                            (rearTrackWidth / 2) * (vehicleFx.RL - vehicleFx.RR);

    // TODO: aero yaw moment
    float yawMoment = yawMomentFromFy + yawMomentFromFx + yawMomentFromTires;
    return {latAcc, yawMoment};
}

VehicleState* Vehicle::getState() { return &state; }

WheelData<float> Vehicle::calculateSlipAngles(float yawVelocity, Vec3<float> velocity) {
    float massToFront = combinedTotalMass.position.x;
    float massToRear = trackDistance - massToFront;

    WheelData<float> slipAngle;

    slipAngle.FL = std::atan((velocity.y + yawVelocity * massToFront) /
                             (velocity.x - yawVelocity * frontTrackWidth / 2.0)) -
                   state.steeringAngle / 180 * M_PI;
    slipAngle.FR = std::atan((velocity.y + yawVelocity * massToFront) /
                             (velocity.x + yawVelocity * frontTrackWidth / 2.0)) -
                   state.steeringAngle / 180 * M_PI;
    slipAngle.RL = std::atan((velocity.y - yawVelocity * massToRear) /
                             (velocity.x - yawVelocity * rearTrackWidth / 2.0));
    slipAngle.RR = std::atan((velocity.y - yawVelocity * massToRear) /
                             (velocity.x + yawVelocity * rearTrackWidth / 2.0));

    return slipAngle;
}

float Vehicle::calculateLatAcc(const WheelData<float>& tireForcesX,
                               const WheelData<float>& tireForcesY) {
    auto vehicleFy = getVehicleFyFromTireForces(tireForcesX, tireForcesY);
    float latForce = 0;
    for (size_t i = 0; i < CarConstants::WHEEL_COUNT; i++) {
        latForce += vehicleFy[i];
    }
    float latAcc = latForce / combinedTotalMass.value;
    return latAcc;
}

WheelData<float> Vehicle::getVehicleFyFromTireForces(const WheelData<float>& tireFx,
                                                     const WheelData<float>& tireFy) {
    WheelData<float> vehicleFy;
    vehicleFy.RL = tireFy.RL;
    vehicleFy.RR = tireFy.RR;

    float delta = M_PI / 180 * state.steeringAngle;
    float cosDelta = std::cos(delta);
    float sinDelta = std::sin(delta);
    vehicleFy.FL = tireFx.FL * sinDelta + tireFy.FL * cosDelta;
    vehicleFy.FR = tireFx.FR * sinDelta + tireFy.FR * cosDelta;
    return vehicleFy;
}

WheelData<float> Vehicle::getVehicleFxFromTireForces(const WheelData<float>& tireFx,
                                                     const WheelData<float>& tireFy) {
    WheelData<float> vehicleFx;
    vehicleFx.RL = tireFx.RL;
    vehicleFx.RR = tireFx.RR;

    float delta = M_PI / 180 * state.steeringAngle;
    float cosDelta = std::cos(delta);
    float sinDelta = std::sin(delta);
    vehicleFx.FL = tireFx.FL * cosDelta - tireFy.FL * sinDelta;
    vehicleFx.FR = tireFx.FR * cosDelta - tireFy.FR * sinDelta;
    return vehicleFx;
}

WheelData<float> Vehicle::totalTireLoads(float latAcc, const EnvironmentConfig& environmentConfig) {
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
    // known problem described in onenote:
    // cannot calculate distribution to 4 corners from one mass center
    aero.value.calculate(state, environmentConfig.airDensity, environmentConfig.wind);
    return distributeForces(-aero.value.getForce().value.z, aero.position.x, aero.position.y);
}

WheelData<float> Vehicle::loadTransfer(float latAcc) {
    float nonSuspendedMassFront = combinedNonSuspendedMass.value *
                                  (trackDistance - combinedNonSuspendedMass.position.x) /
                                  trackDistance;
    float nonSuspendedMassRear = combinedNonSuspendedMass.value - nonSuspendedMassFront;

    float nonSuspendedWTFront =
        nonSuspendedMassFront * latAcc * combinedNonSuspendedMass.position.z / frontTrackWidth;
    float nonSuspendedWTRear =
        nonSuspendedMassRear * latAcc * combinedNonSuspendedMass.position.z / rearTrackWidth;

    float suspendedMassFront = combinedSuspendedMass.value *
                               (trackDistance - combinedSuspendedMass.position.x) / trackDistance;
    float suspendedMassRear = combinedSuspendedMass.value - suspendedMassFront;

    float geometricWTFront = suspendedMassFront * latAcc * rollCenterHeightFront / frontTrackWidth;
    float geometricWTRear = suspendedMassRear * latAcc * rollCenterHeightBack / rearTrackWidth;

    float antiRollStiffnessTotal = antiRollStiffnessFront + antiRollStiffnessRear;

    float elasticWTFront = suspendedMassFront * latAcc *
                           (combinedSuspendedMass.position.z - rollCenterHeightFront) *
                           (antiRollStiffnessFront / antiRollStiffnessTotal) / frontTrackWidth;
    float elasticWTRear = suspendedMassRear * latAcc *
                          (combinedSuspendedMass.position.z - rollCenterHeightBack) *
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
