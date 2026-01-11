#pragma once

#include <optional>

#include "config/config.h"
#include "vehicle/aero/aero.h"
#include "vehicle/tire/tire.h"
#include "vehicle/vehicleHelper.h"
class Vehicle {
    Body combinedTotalMass;

    Body nonSuspendedMass;
    Body suspendedMass;

    float rollCenterHeightFront;
    float rollCenterHeightBack;

    float antiRollStiffnessFront;
    float antiRollStiffnessRear;

    float frontTrackWidth;
    float rearTrackWidth;
    float trackDistance;
    vehicleState state;

    Aero aero;

    CarWheelBase<std::unique_ptr<Tire>> tires;

    float steeringAngle;
    float chassisSlipAngle;

    vec2<float> getLatAccAndYawMoment(float tolerance, const EnvironmentConfig& environmentConfig);
    CarWheelBase<float> calculateSlipAngles(float r, vec2<float> velocity);
    CarWheelBase<float> staticLoad(float earthAcc);
    float calculateLatAcc(CarWheelBase<float> tireForcesY);
    CarWheelBase<float> distributeForces(float totalForce, float frontDist, float leftDist);
    CarWheelBase<float> totalTireLoads(float latAcc, const EnvironmentConfig& environmentConfig);
    CarWheelBase<float> aeroLoad(float airDensity);
    CarWheelBase<float> loadTransfer(float latAcc);

   public:
    vehicleState* getState();
    Vehicle(const VehicleConfig& config);
    Vehicle() = default;
    void calculateYawMomentDiagram(float tolerance, const EnvironmentConfig& environmentConfig);
};
