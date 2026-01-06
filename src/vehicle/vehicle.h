#pragma once

#include "config/config.h"
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

    float cla;
    vec2<float> claPosition;

    CarWheelBase<std::unique_ptr<Tire>> tires;

    float steeringAngle;
    float chassisSlipAngle;

    vec2<float> getLatAccAndYawMoment(float speed, float tolerance,
                                      const EnvironmentConfig& environmentConfig);
    CarWheelBase<float> calculateSlipAngles(float r, vec2<float> velocity);
    CarWheelBase<float> staticLoad(float earthAcc);
    float calculateLatAcc(CarWheelBase<float> tireForcesY);
    CarWheelBase<float> distributeForces(float totalForce, float frontDist, float leftDist);
    CarWheelBase<float> totalTireLoads(float speed, float latAcc,
                                       const EnvironmentConfig& environmentConfig);
    CarWheelBase<float> aeroLoad(float speed, float airDensity);
    CarWheelBase<float> loadTransfer(float latAcc);

   public:
    Vehicle(const VehicleConfig& config);
    Vehicle() = default;
    void calculateYawMomentDiagram(float speed, float tolerance,
                                   const EnvironmentConfig& environmentConfig);
};
