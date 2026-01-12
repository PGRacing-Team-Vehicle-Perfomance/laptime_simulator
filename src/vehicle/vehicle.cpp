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

Vehicle::Vehicle(const VehicleConfig& config)
    : nonSuspendedMass(config.nonSuspendedMass),
      suspendedMass(config.suspendedMass),
      rollCenterHeightFront(config.rollCenterHeightFront),
      rollCenterHeightBack(config.rollCenterHeightBack),
      antiRollStiffnessFront(config.antiRollStiffnessFront),
      antiRollStiffnessRear(config.antiRollStiffnessRear),
      frontTrackWidth(config.frontTrackWidth),
      rearTrackWidth(config.rearTrackWidth),
      trackDistance(config.trackDistance),
      aero(config) {
    for (size_t i = 0; i < CarAcronyms::WHEEL_COUNT; i++) {
        tires[i] = std::make_unique<TireSimple>(0, 0, 0, false);
    }
    combinedTotalMass.mass = suspendedMass.mass + nonSuspendedMass.mass;
    combinedTotalMass.position = {(suspendedMass.position.x * suspendedMass.mass +
                                   nonSuspendedMass.position.x * nonSuspendedMass.mass) /
                                      combinedTotalMass.mass,
                                  (suspendedMass.position.y * suspendedMass.mass +
                                   nonSuspendedMass.position.y * nonSuspendedMass.mass) /
                                      combinedTotalMass.mass,
                                  (suspendedMass.position.z * suspendedMass.mass +
                                   nonSuspendedMass.position.z * nonSuspendedMass.mass) /
                                      combinedTotalMass.mass};
}

void Vehicle::calculateYawMomentDiagram(float tolerance,
                                        const EnvironmentConfig& environmentConfig) {
    for (int i = 0; i < 90; i++) {
        state.steeringAngle = i;
        for (int j = 0; j < 90; j++) {
            state.rotation.z = j;
            vec2<float> diagramPoint = getLatAccAndYawMoment(tolerance, environmentConfig);
            printf("steering angle: %f, chassis slip angle: %f, lat acc: %f, yaw moment: %f\n",
                   state.steeringAngle, state.rotation.z.get(), diagramPoint.x, diagramPoint.y);
        }
    }
}

vec2<float> Vehicle::getLatAccAndYawMoment(float tolerance,
                                           const EnvironmentConfig& environmentConfig) {
    Angle beta = state.rotation.z;
    vec2<float> velocity;
    velocity.x = state.velocity.amplitude * std::cos(beta.getRadians());
    velocity.y = state.velocity.amplitude * std::sin(beta.getRadians());

    CarWheelBase<float> tireForcesY;
    CarWheelBase<float> tireMomentsY;
    CarWheelBase<float> slipAngles;
    float latAcc = 0;
    float error;

    auto loads = staticLoad(environmentConfig.earthAcc);
    do {
        slipAngles = calculateSlipAngles(state.angular_velocity.z, velocity);

        for (size_t i = 0; i < CarAcronyms::WHEEL_COUNT; i++) {
            tireForcesY[i] = tires[i]->getLateralForce(loads[i], slipAngles[i]);
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
        mz += tireMomentsY[i] = tires[i]->getLateralMoment(loads[i], slipAngles[i]);
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

vehicleState* Vehicle::getState() { return &state; }

CarWheelBase<float> Vehicle::calculateSlipAngles(float r, vec2<float> velocity) {
    float a = combinedTotalMass.position.x;
    float b = trackDistance - a;

    CarWheelBase<float> slipAngle;

    slipAngle[CarAcronyms::FL] =
        (velocity.x + r * a) / velocity.x - r * frontTrackWidth / 2 - state.steeringAngle;
    slipAngle[CarAcronyms::FR] =
        (velocity.x + r * a) / velocity.x + r * frontTrackWidth / 2 - state.steeringAngle;
    slipAngle[CarAcronyms::RL] = (velocity.x - r * b) / velocity.x - r * rearTrackWidth / 2;
    slipAngle[CarAcronyms::RR] = (velocity.x - r * b) / velocity.x + r * rearTrackWidth / 2;

    return slipAngle;
}

float Vehicle::calculateLatAcc(CarWheelBase<float> tireForcesY) {
    float latAcc = 0;
    float latForce = 0;
    for (size_t i = 0; i < CarAcronyms::WHEEL_COUNT; i++) {
        latForce += tireForcesY[i];
    }
    latAcc = latForce / combinedTotalMass.mass;
    return latAcc;
}

CarWheelBase<float> Vehicle::totalTireLoads(float latAcc,
                                            const EnvironmentConfig& environmentConfig) {
    getState()->velocity.amplitude = state.velocity.amplitude;
    auto static_load = staticLoad(environmentConfig.earthAcc);
    auto aero = aeroLoad(environmentConfig.airDensity);
    auto transfer = loadTransfer(latAcc);
    CarWheelBase<float> ret;
    for (size_t i = 0; i < CarAcronyms::WHEEL_COUNT; i++) {
        ret[i] = std::max(0.f, static_load[i] + aero[i] + transfer[i]);
    }
    return ret;
}

CarWheelBase<float> Vehicle::staticLoad(float earthAcc) {
    // known problem described in onenote - cannot calculate distribution to 4 corners from one mass
    // center here we can fix it by providing 4 masses - on each wheel instead of mass center in
    // vehicle config
    return distributeForces(combinedTotalMass.mass * earthAcc, combinedTotalMass.position.x,
                            combinedTotalMass.position.y);
}

vehicleState Vehicle::springing(CarWheelBase<float> loads) { return state; }

CarWheelBase<float> Vehicle::distributeForces(float totalForce, float frontDist, float leftDist) {
    CarWheelBase<float> forces;
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

CarWheelBase<float> Vehicle::aeroLoad(float airDensity) {
    // known problem described in onenote - cannot calculate distribution to 4 corners from one mass
    // center
    aero.calculteLoads(state, airDensity);
    return distributeForces(aero.getLoads().force.z.amplitude, aero.getLoads().force.z.origin.x,
                            aero.getLoads().force.z.origin.y);
}

CarWheelBase<float> Vehicle::loadTransfer(float latAcc) {
    // steady state
    float nonSuspendedMassFront =
        nonSuspendedMass.mass * (trackDistance - nonSuspendedMass.position.x) / trackDistance;
    float nonSuspendedMassRear = nonSuspendedMass.mass - nonSuspendedMassFront;

    float nonSuspendedWTFront =
        nonSuspendedMassFront * latAcc * nonSuspendedMass.position.z / frontTrackWidth;
    float nonSuspendedWTRear =
        nonSuspendedMassRear * latAcc * nonSuspendedMass.position.z / rearTrackWidth;

    float suspendedMassFront =
        suspendedMass.mass * (trackDistance - suspendedMass.position.x) / trackDistance;
    float suspendedMassRear = suspendedMass.mass - suspendedMassFront;

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

    CarWheelBase<float> loads;

    loads[CarAcronyms::FL] = -frontTransfer;
    loads[CarAcronyms::FR] = frontTransfer;
    loads[CarAcronyms::RL] = -rearTransfer;
    loads[CarAcronyms::RR] = rearTransfer;

    return loads;
}
