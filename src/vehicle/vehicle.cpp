#include "vehicle/vehicle.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <numbers>

#include "config/config.h"
#include "vehicle/aero/aero.h"
#include "vehicle/tire/tire.h"
#include "vehicle/tire/tireSimple.h"
#include "vehicle/vehicleHelper.h"

Vehicle::Vehicle(const VehicleConfig& vahicleConfig, const TireConfig& tireConfig)
    : nonSuspendedMass(vahicleConfig.nonSuspendedMass),
      suspendedMass(vahicleConfig.suspendedMass),
      rollCenterHeightFront(vahicleConfig.rollCenterHeightFront),
      rollCenterHeightBack(vahicleConfig.rollCenterHeightBack),
      antiRollStiffnessFront(vahicleConfig.antiRollStiffnessFront),
      antiRollStiffnessRear(vahicleConfig.antiRollStiffnessRear),
      frontTrackWidth(vahicleConfig.frontTrackWidth),
      rearTrackWidth(vahicleConfig.rearTrackWidth),
      trackDistance(vahicleConfig.trackDistance),
      aero(vahicleConfig) {
    for (size_t i = 0; i < CarAcronyms::WHEEL_COUNT; i++) {
        tires[i] = std::make_unique<TireSimple>(tireConfig, false);
    }
    combinedTotalMass.value = suspendedMass.value + nonSuspendedMass.value;
    combinedTotalMass.position = {(suspendedMass.position.x * suspendedMass.value +
                                   nonSuspendedMass.position.x * nonSuspendedMass.value) /
                                      combinedTotalMass.value,
                                  (suspendedMass.position.y * suspendedMass.value +
                                   nonSuspendedMass.position.y * nonSuspendedMass.value) /
                                      combinedTotalMass.value,
                                  (suspendedMass.position.z * suspendedMass.value +
                                   nonSuspendedMass.position.z * nonSuspendedMass.value) /
                                      combinedTotalMass.value};
}

void Vehicle::calculateYawMomentDiagram(float tolerance,
                                        const EnvironmentConfig& environmentConfig) {
    for (int i = 0; i < 90; i++) {
        state.steeringAngle = i;
        for (int j = 0; j < 90; j++) {
            state.rotation.z = j;
            std::array<float, 2> diagramPoint = getLatAccAndYawMoment(tolerance, environmentConfig);
            printf("steering angle: %f, chassis slip angle: %f, lat acc: %f, yaw moment: %f\n",
                   state.steeringAngle, state.rotation.z.get(), diagramPoint[0], diagramPoint[1]);
        }
    }
}

std::array<float, 2> Vehicle::getLatAccAndYawMoment(float tolerance,
                                           const EnvironmentConfig& environmentConfig) {
    Angle beta = state.rotation.z;
    Vec3f velocity;
    velocity.x = state.velocity.getLength() * std::cos(beta.getRadians());
    velocity.y = state.velocity.getLength() * std::sin(beta.getRadians());
    velocity.z = 0;

    WheelData<float> tireForcesY;
    WheelData<float> tireMomentsY;
    WheelData<float> slipAngles;
    float latAcc = 0;
    float error;

    auto loads = staticLoad(environmentConfig.earthAcc);
    do {
        slipAngles = calculateSlipAngles(state.angular_velocity.z, velocity);

        for (size_t i = 0; i < CarAcronyms::WHEEL_COUNT; i++) {
            tires[i]->calculate(loads[i], slipAngles[i], 0);
            tireForcesY[i] = tires[i]->getForce().value.y;
            // recalculate for forces relative to chassis or speed vector?
            // tire longitudinal forces with slip ratio = 0
        }

        auto newLatAcc = calculateLatAcc(tireForcesY);
        error = std::abs(latAcc - newLatAcc);
        latAcc = newLatAcc;  // can be different -> latAcc = f(error) 391
        state.angular_velocity.z = latAcc / velocity.x;

        loads = totalTireLoads(latAcc, environmentConfig);
        springing(loads);
    } while (error > tolerance);

    float mz = 0;
    for (size_t i = 0; i < CarAcronyms::WHEEL_COUNT; i++) {
        mz += tireMomentsY[i] = tires[i]->getTorque().y;
    }

    // recalculate for forces relative to chassis
    // same for x moments
    // aero yaw moment
    float yawMoment =
        ((tireForcesY[CarAcronyms::FL] + tireForcesY[CarAcronyms::FR]) *
         combinedTotalMass.position.x) -
        ((tireForcesY[CarAcronyms::RL] + tireForcesY[CarAcronyms::RR]) *
         (trackDistance - combinedTotalMass.position.x)) +
        ((tireForcesY[CarAcronyms::FR] + tireForcesY[CarAcronyms::RR]) * rearTrackWidth / 2) -
        ((tireForcesY[CarAcronyms::FL] + tireForcesY[CarAcronyms::RL]) * frontTrackWidth / 2) + mz;
    return {latAcc, yawMoment};
}

VehicleState* Vehicle::getState() { return &state; }

WheelData<float> Vehicle::calculateSlipAngles(float r, Vec3<float> velocity) {
    float a = combinedTotalMass.position.x;
    float b = trackDistance - a;

    WheelData<float> slipAngle;

    slipAngle[CarAcronyms::FL] =
        (velocity.x + r * a) / velocity.x - r * frontTrackWidth / 2 - state.steeringAngle;
    slipAngle[CarAcronyms::FR] =
        (velocity.x + r * a) / velocity.x + r * frontTrackWidth / 2 - state.steeringAngle;
    slipAngle[CarAcronyms::RL] = (velocity.x - r * b) / velocity.x - r * rearTrackWidth / 2;
    slipAngle[CarAcronyms::RR] = (velocity.x - r * b) / velocity.x + r * rearTrackWidth / 2;

    return slipAngle;
}

float Vehicle::calculateLatAcc(WheelData<float> tireForcesY) {
    float latAcc = 0;
    float latForce = 0;
    for (size_t i = 0; i < CarAcronyms::WHEEL_COUNT; i++) {
        latForce += tireForcesY[i];
    }
    latAcc = latForce / combinedTotalMass.value;
    return latAcc;
}

WheelData<float> Vehicle::totalTireLoads(float latAcc,
                                            const EnvironmentConfig& environmentConfig) {
    auto static_load = staticLoad(environmentConfig.earthAcc);
    auto aero = aeroLoad(environmentConfig);
    auto transfer = loadTransfer(latAcc);
    WheelData<float> ret;
    for (size_t i = 0; i < CarAcronyms::WHEEL_COUNT; i++) {
        ret[i] = std::max(0.f, static_load[i] + aero[i] + transfer[i]);
    }
    return ret;
}

WheelData<float> Vehicle::staticLoad(float earthAcc) {
    // known problem described in onenote - cannot calculate distribution to 4 corners from one mass
    // center here we can fix it by providing 4 masses - on each wheel instead of mass center in
    // vehicle config
    return distributeForces(combinedTotalMass.value * earthAcc, combinedTotalMass.position.x,
                            combinedTotalMass.position.y);
}

VehicleState Vehicle::springing(WheelData<float> loads) { return state; }

WheelData<float> Vehicle::distributeForces(float totalForce, float frontDist, float leftDist) {
    WheelData<float> forces;
    forces[CarAcronyms::FL] = totalForce * (trackDistance - frontDist) / trackDistance *
                              (frontTrackWidth / 2 + leftDist) / frontTrackWidth;
    forces[CarAcronyms::FR] = totalForce * (trackDistance - frontDist) / trackDistance *
                              (frontTrackWidth / 2 - leftDist) / frontTrackWidth;
    forces[CarAcronyms::RL] =
        totalForce * frontDist / trackDistance * (rearTrackWidth / 2 + leftDist) / rearTrackWidth;
    forces[CarAcronyms::RR] =
        totalForce * frontDist / trackDistance * (rearTrackWidth / 2 - leftDist) / rearTrackWidth;
    return forces;
}

WheelData<float> Vehicle::aeroLoad(const EnvironmentConfig& environmentConfig) {
    // known problem described in onenote - cannot calculate distribution to 4 corners from one mass
    // center
    //force = aero.getForce()
    
    aero.value.calculate(state, environmentConfig.airDensity, environmentConfig.wind);
    return distributeForces(aero.value.getForce().value.z, aero.position.x,
                            aero.position.y);
}

WheelData<float> Vehicle::loadTransfer(float latAcc) {
    // steady state
    float nonSuspendedMassFront =
        nonSuspendedMass.value * (trackDistance - nonSuspendedMass.position.x) / trackDistance;
    float nonSuspendedMassRear = nonSuspendedMass.value - nonSuspendedMassFront;

    float nonSuspendedWTFront =
        nonSuspendedMassFront * latAcc * nonSuspendedMass.position.z / frontTrackWidth;
    float nonSuspendedWTRear =
        nonSuspendedMassRear * latAcc * nonSuspendedMass.position.z / rearTrackWidth;

    float suspendedMassFront =
        suspendedMass.value * (trackDistance - suspendedMass.position.x) / trackDistance;
    float suspendedMassRear = suspendedMass.value - suspendedMassFront;

    float geometricWTFront = suspendedMassFront * latAcc * rollCenterHeightFront / frontTrackWidth;
    float geometricWTRear = suspendedMassRear * latAcc * rollCenterHeightBack / rearTrackWidth;

    float antiRollStiffnessTotal = antiRollStiffnessFront + antiRollStiffnessRear;

    float elasticWTFront = suspendedMassFront * latAcc *
                           (suspendedMass.position.z - rollCenterHeightFront) *
                           (antiRollStiffnessFront / antiRollStiffnessTotal) / frontTrackWidth;
    float elasticWTRear = suspendedMassRear * latAcc *
                          (suspendedMass.position.z - rollCenterHeightBack) *
                          (antiRollStiffnessRear / antiRollStiffnessTotal) / rearTrackWidth;

    float frontTransfer = nonSuspendedWTFront + geometricWTFront + elasticWTFront;
    float rearTransfer = nonSuspendedWTRear + geometricWTRear + elasticWTRear;

    WheelData<float> loads;

    loads[CarAcronyms::FL] = -frontTransfer;
    loads[CarAcronyms::FR] = frontTransfer;
    loads[CarAcronyms::RL] = -rearTransfer;
    loads[CarAcronyms::RR] = rearTransfer;

    return loads;
}
